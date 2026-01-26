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

namespace magnetometer_compass
{

using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Field = sensor_msgs::msg::MagneticField;

struct MagnetometerCompassPrivate
{
  std::shared_ptr<tf2_ros::Buffer const> tf;
  std::string frame;
  std::optional<tf2::Quaternion> lastAzimuth;
  double variance {0.0};
  double initialVariance {0.0};
  double lowPassRatio {0.95};
};

MagnetometerCompass::MagnetometerCompass(
  RequiredInterfaces node, const std::string& frame, const std::shared_ptr<tf2_ros::Buffer>& tf)
  : data(new MagnetometerCompassPrivate{}), node(node)
{
  this->data->tf = tf;
  this->data->frame = frame;
}

MagnetometerCompass::~MagnetometerCompass() = default;

geometry_msgs::msg::Quaternion MagnetometerCompass::getRotationBetweenFrames(const sensor_msgs::msg::Imu& imu_msg)
{
  try
  {
    // Lookup transform from IMU frame to target frame at the IMU's timestamp
    const auto transform_stamped = this->data->tf->lookupTransform(
      imu_msg.header.frame_id, this->data->frame, imu_msg.header.stamp, tf2::durationFromSec(0.1));

    // Extract rotation (quaternion) from the transform
    last_imu_orientation = transform_stamped.transform.rotation;
    return transform_stamped.transform.rotation;
  }
  catch (const tf2::TransformException& ex)
  {
    const auto& log = this->node.get_node_logging_interface();
    RCLCPP_WARN(log->get_logger(), "Transform failed: %s returning last known transform", ex.what());
    return last_imu_orientation;  // Return identity or handle failure appropriately
  }
}

void MagnetometerCompass::configFromParams()
{
  const auto& params = this->node.get_node_parameters_interface();
  if (params->has_parameter("initial_variance") && params->get_parameter("initial_variance").as_double() != -1.)
    this->data->variance = this->data->initialVariance = params->get_parameter("initial_variance").as_double();

  if (params->has_parameter("low_pass_ratio") && params->get_parameter("low_pass_ratio").as_double() != -1.)
    this->data->lowPassRatio = params->get_parameter("low_pass_ratio").as_double();
}

void MagnetometerCompass::setLowPassRatio(const double ratio)
{
  this->data->lowPassRatio = ratio;
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> MagnetometerCompass::computeAzimuth(
  const sensor_msgs::msg::Imu& imu, const sensor_msgs::msg::MagneticField& magUnbiased)
{
  Imu imuInBody;
  try
  {
    this->data->tf->transform(imu, imuInBody, this->data->frame, tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    return cras::make_unexpected(cras::format(
      "Could not transform IMU data to frame {} because: {}", this->data->frame.c_str(), e.what()));
  }

  Field magUnbiasedInBody;
  try
  {
    this->data->tf->transform(magUnbiased, magUnbiasedInBody, this->data->frame, tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    return cras::make_unexpected(cras::format(
      "Could not transform magnetometer to frame {} because: {}", this->data->frame.c_str(), e.what()));
  }

  auto imu_q = getRotationBetweenFrames(imu);
  this->last_imu_orientation = imu_q;

  double imu_roll, imu_pitch, imu_yaw;
  cras::getRPY(imu_q, imu_roll, imu_pitch, imu_yaw);

  // Compensate attitude in the magnetometer measurements

  double roll, pitch, yaw;
  cras::getRPY(imuInBody.orientation, roll, pitch, yaw);

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

  tf2::Vector3 magNoAttitude;
  tf2::convert(magUnbiasedInBody.magnetic_field, magNoAttitude);
  magNoAttitude = tf2::quatRotate(final_rot, magNoAttitude);

  const auto magNorth = magNoAttitude.x();
  const auto magEast = magNoAttitude.y();
#else
  // Copied from INSO, not sure where do the numbers come from
  const auto magNorth =
    magUnbiasedInBody.magnetic_field.x * cos(pitch) +
    magUnbiasedInBody.magnetic_field.y * sin(pitch) * sin(roll) +
    magUnbiasedInBody.magnetic_field.z * sin(pitch) * cos(roll);

  const auto magEast =
    magUnbiasedInBody.magnetic_field.y * cos(roll) -
    magUnbiasedInBody.magnetic_field.z * sin(roll);
#endif

  // This formula gives north-referenced clockwise-increasing azimuth
  const auto magAzimuthNow = atan2(magEast, magNorth);
  tf2::Quaternion magAzimuthNowQuat;
  magAzimuthNowQuat.setRPY(0, 0, magAzimuthNow);

  if (!this->data->lastAzimuth.has_value())
    this->data->lastAzimuth = magAzimuthNowQuat;
  else
    // low-pass filter
    this->data->lastAzimuth = this->data->lastAzimuth->slerp(magAzimuthNowQuat, 1 - this->data->lowPassRatio);
  this->updateVariance();

  compass_interfaces::msg::Azimuth nedAzimuthMsg;
  nedAzimuthMsg.header.stamp = magUnbiased.header.stamp;
  nedAzimuthMsg.header.frame_id = this->data->frame;
  nedAzimuthMsg.azimuth = angles::normalize_angle_positive(cras::getYaw(*this->data->lastAzimuth));
  nedAzimuthMsg.variance = this->data->variance;
  nedAzimuthMsg.unit = Az::UNIT_RAD;
  nedAzimuthMsg.orientation = Az::ORIENTATION_NED;
  nedAzimuthMsg.reference = Az::REFERENCE_MAGNETIC;

  return nedAzimuthMsg;
}

void MagnetometerCompass::reset()
{
  this->data->variance = this->data->initialVariance;
  this->data->lastAzimuth.reset();
}

void MagnetometerCompass::updateVariance()
{
  // TODO: measure consistency of IMU rotation and azimuth increase similar to
  // https://www.sciencedirect.com/science/article/pii/S2405959519302929

  // OR sample (e.g. 8 in cube) magnetometer vectors (with imu/mag covariance) compute azimuth angles and compute
  // their std...
}

}
