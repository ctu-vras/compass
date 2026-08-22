// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert magnetometer and IMU measurements to azimuth.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <map>
#include <memory>
#include <optional>
#include <string>

#include <angles/angles.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/format.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <magnetometer_compass/magnetometer_compass.hpp>
#include <magnetometer_compass/tf2_sensor_msgs.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>

namespace magnetometer_compass {

using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Field = sensor_msgs::msg::MagneticField;

struct MagnetometerCompassPrivate {
  std::shared_ptr<tf2_ros::Buffer const> tf_;
  std::string frame_;
  std::optional<tf2::Quaternion> last_azimuth_;
  double variance_ {0.0};
  double initial_variance_ {0.0};
  double low_pass_ratio_ {0.95};
};

MagnetometerCompass::MagnetometerCompass(
    RequiredInterfaces node, const std::string& frame, const std::shared_ptr<tf2_ros::Buffer>& tf)
    : data_(new MagnetometerCompassPrivate{}), node_(node) {
  this->data_->tf_ = tf;
  this->data_->frame_ = frame;
}

MagnetometerCompass::~MagnetometerCompass() = default;

geometry_msgs::msg::Quaternion MagnetometerCompass::getRotationBetweenFrames(const sensor_msgs::msg::Imu& imu_msg) {
  try {
    // Lookup transform from IMU frame to target frame at the IMU's timestamp
    const auto transform_stamped = this->data_->tf_->lookupTransform(
      imu_msg.header.frame_id, this->data_->frame_, imu_msg.header.stamp, tf2::durationFromSec(0.1));

    // Extract rotation (quaternion) from the transform
    last_imu_orientation_ = transform_stamped.transform.rotation;
    return transform_stamped.transform.rotation;
  } catch (const tf2::TransformException& ex) {
    const auto& log = this->node_.get_node_logging_interface();
    RCLCPP_WARN(log->get_logger(), "Transform failed: %s returning last known transform", ex.what());
    return last_imu_orientation_;  // Return identity or handle failure appropriately
  }
}

void MagnetometerCompass::configFromParams() {
  const auto& params = this->node_.get_node_parameters_interface();
  if (params->has_parameter("initial_variance") && params->get_parameter("initial_variance").as_double() != -1.) {
    this->data_->variance_ = this->data_->initial_variance_ = params->get_parameter("initial_variance").as_double();
  }

  if (params->has_parameter("low_pass_ratio") && params->get_parameter("low_pass_ratio").as_double() != -1.) {
    this->data_->low_pass_ratio_ = params->get_parameter("low_pass_ratio").as_double();
  }
}

void MagnetometerCompass::setLowPassRatio(const double ratio) {
  this->data_->low_pass_ratio_ = ratio;
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> MagnetometerCompass::computeAzimuth(
    const sensor_msgs::msg::Imu& imu, const sensor_msgs::msg::MagneticField& mag_unbiased) {
  Imu imu_in_body;
  try {
    this->data_->tf_->transform(imu, imu_in_body, this->data_->frame_, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException& e) {
    return cras::make_unexpected(cras::format(
      "Could not transform IMU data to frame {} because: {}", this->data_->frame_.c_str(), e.what()));
  }

  Field mag_unbiased_in_body;
  try {
    this->data_->tf_->transform(mag_unbiased, mag_unbiased_in_body, this->data_->frame_, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException& e) {
    return cras::make_unexpected(cras::format(
      "Could not transform magnetometer to frame {} because: {}", this->data_->frame_.c_str(), e.what()));
  }

  auto imu_q = getRotationBetweenFrames(imu);
  this->last_imu_orientation_ = imu_q;

  double imu_roll, imu_pitch, imu_yaw;
  cras::getRPY(imu_q, imu_roll, imu_pitch, imu_yaw);

  // Compensate attitude in the magnetometer measurements

  double roll, pitch, yaw;
  cras::getRPY(imu_in_body.orientation, roll, pitch, yaw);

  tf2::Quaternion rot, imu_rot;
  rot.setRPY(roll, pitch, yaw);
  imu_rot.setRPY(imu_roll, imu_pitch, imu_yaw);
  auto corrected_rot = imu_rot.inverse() * rot;
  double corrected_roll, corrected_pitch, corrected_yaw;
  cras::getRPY(corrected_rot, corrected_roll, corrected_pitch, corrected_yaw);

#if 1
  tf2::Quaternion final_rot;
  final_rot.setRPY(corrected_roll, corrected_pitch, 0.);
  final_rot.normalize();

  tf2::Vector3 mag_no_attitude;
  tf2::convert(mag_unbiased_in_body.magnetic_field, mag_no_attitude);
  mag_no_attitude = tf2::quatRotate(final_rot, mag_no_attitude);

  const auto mag_north = mag_no_attitude.x();
  const auto mag_east = mag_no_attitude.y();
#else
  // Copied from INSO, not sure where do the numbers come from
  const auto mag_north =
    mag_unbiased_in_body.magnetic_field.x * cos(pitch) +
    mag_unbiased_in_body.magnetic_field.y * sin(pitch) * sin(roll) +
    mag_unbiased_in_body.magnetic_field.z * sin(pitch) * cos(roll);

  const auto mag_east =
    mag_unbiased_in_body.magnetic_field.y * cos(roll) -
    mag_unbiased_in_body.magnetic_field.z * sin(roll);
#endif

  // This formula gives north-referenced clockwise-increasing azimuth
  const auto mag_azimuth_now = atan2(mag_east, mag_north);
  tf2::Quaternion mag_azimuth_now_quat;
  mag_azimuth_now_quat.setRPY(0, 0, mag_azimuth_now);

  if (!this->data_->last_azimuth_.has_value()) {
    this->data_->last_azimuth_ = mag_azimuth_now_quat;
  } else {
    // low-pass filter
    this->data_->last_azimuth_ =
      this->data_->last_azimuth_->slerp(mag_azimuth_now_quat, 1 - this->data_->low_pass_ratio_);
  }
  this->updateVariance();

  compass_interfaces::msg::Azimuth ned_azimuth_msg;
  ned_azimuth_msg.header.stamp = mag_unbiased.header.stamp;
  ned_azimuth_msg.header.frame_id = this->data_->frame_;
  ned_azimuth_msg.azimuth = angles::normalize_angle_positive(cras::getYaw(*this->data_->last_azimuth_));
  ned_azimuth_msg.variance = this->data_->variance_;
  ned_azimuth_msg.unit = Az::UNIT_RAD;
  ned_azimuth_msg.orientation = Az::ORIENTATION_NED;
  ned_azimuth_msg.reference = Az::REFERENCE_MAGNETIC;

  return ned_azimuth_msg;
}

void MagnetometerCompass::reset() {
  this->data_->variance_ = this->data_->initial_variance_;
  this->data_->last_azimuth_.reset();
}

void MagnetometerCompass::updateVariance() {
  // TODO: measure consistency of IMU rotation and azimuth increase similar to
  // https://www.sciencedirect.com/science/article/pii/S2405959519302929

  // OR sample (e.g. 8 in cube) magnetometer vectors (with imu/mag covariance) compute azimuth angles and compute
  // their std...
}

}  // namespace magnetometer_compass
