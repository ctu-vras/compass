// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert magnetometer and IMU measurements to azimuth.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <Eigen/Core>

#include <angles/angles.h>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <imu_transformer/tf2_sensor_msgs.h>
#include <magnetometer_compass/magnetometer_compass.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>

namespace magnetometer_compass
{
using Az = compass_msgs::Azimuth;
using Imu = sensor_msgs::Imu;
using Field = sensor_msgs::MagneticField;

struct MagnetometerCompassPrivate
{
  std::shared_ptr<cras::InterruptibleTFBuffer const> tf;
  std::string frame;
  cras::optional<tf2::Quaternion> lastAzimuth;
  double variance {0.0};
  double initialVariance {0.0};
  double lowPassRatio {0.95};
};

MagnetometerCompass::MagnetometerCompass(
  const cras::LogHelperPtr& log, const std::string& frame, const std::shared_ptr<tf2::BufferCore>& tf) :
  MagnetometerCompass(log, frame, std::make_shared<cras::InterruptibleTFBuffer>(tf))
{
}

MagnetometerCompass::MagnetometerCompass(const cras::LogHelperPtr& log, const std::string& frame,
  const std::shared_ptr<cras::InterruptibleTFBuffer>& tf) :
  cras::HasLogger(log), data(new MagnetometerCompassPrivate{})
{
  this->data->tf = tf;
  this->data->frame = frame;
}

MagnetometerCompass::~MagnetometerCompass() = default;

void MagnetometerCompass::configFromParams(const cras::BoundParamHelper& params)
{
  this->data->variance = this->data->initialVariance = params.getParam("initial_variance", this->data->variance);
  this->data->lowPassRatio = params.getParam("low_pass_ratio", this->data->lowPassRatio);
}

void MagnetometerCompass::setLowPassRatio(const double ratio)
{
  this->data->lowPassRatio = ratio;
}

cras::expected<compass_msgs::Azimuth, std::string> MagnetometerCompass::computeAzimuth(
  const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& magUnbiased)
{
  Imu imuInBody;

  try
  {
    this->data->tf->transform(imu, imuInBody, this->data->frame, ros::Duration(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    return cras::make_unexpected(cras::format(
      "Could not transform IMU data to frame %s because: %s", this->data->frame.c_str(), e.what()));
  }

  Field magUnbiasedInBody;
  try
  {
    this->data->tf->transform(magUnbiased, magUnbiasedInBody, this->data->frame, ros::Duration(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    return cras::make_unexpected(cras::format(
      "Could not transform magnetometer to frame %s because: %s", this->data->frame.c_str(), e.what()));
  }

  // Compensate attitude in the magnetometer measurements

  double roll, pitch, yaw;
  cras::getRPY(imuInBody.orientation, roll, pitch, yaw);

#if 0
  tf2::Quaternion rot;
  rot.setRPY(roll, pitch, 0);
  tf2::Vector3 magNoAttitude;
  tf2::convert(magUnbiasedInBody.magnetic_field, magNoAttitude);
  magNoAttitude = tf2::quatRotate(rot, magNoAttitude);

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
  else  // low-pass filter
    this->data->lastAzimuth = this->data->lastAzimuth->slerp(magAzimuthNowQuat, 1 - this->data->lowPassRatio);
  this->updateVariance();

  compass_msgs::Azimuth nedAzimuthMsg;
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
  //  https://www.sciencedirect.com/science/article/pii/S2405959519302929
}

}
