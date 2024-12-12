// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer_compass.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>

#include <angles/angles.h>
#include <class_loader/class_loader_core.hpp>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <imu_transformer/tf2_sensor_msgs.h>
#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>
#include <ros/names.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

using Az = compass_msgs::Azimuth;
using Quat = geometry_msgs::QuaternionStamped;
using Pose = geometry_msgs::PoseWithCovarianceStamped;
using Imu = sensor_msgs::Imu;
using Field = sensor_msgs::MagneticField;
using Fix = sensor_msgs::NavSatFix;

ros::V_string my_argv;

template<typename NodeletType = cras::Nodelet>
std::unique_ptr<NodeletType> createNodelet(const cras::LogHelperPtr& log,
  const ros::M_string& remaps = {},
  const std::shared_ptr<tf2_ros::Buffer>& tf = nullptr)
{
  // Declaration order of these variables is important to make sure they can be properly stopped and destroyed.
  auto nodelet = class_loader::impl::createInstance<nodelet::Nodelet>(
    "magnetometer_compass::MagnetometerCompassNodelet", nullptr);
  if (nodelet == nullptr)
    return nullptr;

  {
    const auto paramHelper = dynamic_cast<cras::ParamHelper*>(nodelet);
    if (paramHelper != nullptr)
      paramHelper->setLogger(log);
  }

  const auto targetNodelet = dynamic_cast<NodeletType*>(nodelet);
  if (targetNodelet == nullptr)
  {
    delete nodelet;
    return nullptr;
  }

  if (tf != nullptr)
    targetNodelet->setBuffer(tf);

  nodelet->init(ros::this_node::getName(), remaps, my_argv, nullptr, nullptr);

  return std::unique_ptr<NodeletType>(targetNodelet);
}

double det(const boost::array<double, 9>& mat)
{
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(mat.data()).determinant();
}

TEST(MagnetometerCompassNodelet, BasicConversion)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("publish_mag_azimuth_enu_rad", true);
  pnh.setParam("publish_mag_azimuth_ned_deg", true);
  pnh.setParam("publish_true_azimuth_enu_rad", true);
  pnh.setParam("publish_true_azimuth_enu_imu", true);
  pnh.setParam("publish_utm_azimuth_enu_rad", true);
  pnh.setParam("publish_utm_azimuth_ned_quat", true);
  pnh.setParam("publish_utm_azimuth_ned_pose", true);
  pnh.setParam("publish_mag_unbiased", true);
  pnh.setParam("low_pass_ratio", 0.0);  // Disable smoothing

  std::map<std::tuple<decltype(Az::unit), decltype(Az::orientation), decltype(Az::reference)>, cras::optional<Az>> az;
  auto azCb = [&az](decltype(Az::unit) unit, decltype(Az::orientation) orientation, decltype(Az::reference) reference,
    const Az::ConstPtr& msg)
  {
    az[std::make_tuple(unit, orientation, reference)] = *msg;
  };

  cras::optional<Imu> lastImu;
  auto imuCb = [&lastImu](const Imu::ConstPtr& msg)
  {
    lastImu = *msg;
  };

  cras::optional<Quat> lastQuat;
  auto quatCb = [&lastQuat](const Quat::ConstPtr& msg)
  {
    lastQuat = *msg;
  };

  cras::optional<Pose> lastPose;
  auto poseCb = [&lastPose](const Pose::ConstPtr& msg)
  {
    lastPose = *msg;
  };

  cras::optional<Field> lastField;
  auto magCb = [&lastField](const Field::ConstPtr& msg)
  {
    lastField = *msg;
  };

  std::list<ros::Publisher> pubs;
  auto imuPub = nh.advertise<Imu>("imu/data", 1); pubs.push_back(imuPub);
  auto magPub = nh.advertise<Field>("imu/mag", 1); pubs.push_back(magPub);
  auto magBiasPub = nh.advertise<Field>("imu/mag_bias", 1, true); pubs.push_back(magBiasPub);
  auto fixPub = nh.advertise<Fix>("gps/fix", 1, true); pubs.push_back(fixPub);

  std::list<ros::Subscriber> subs;
  size_t numAzimuths {0u};
  auto magUnbiasedSub = nh.subscribe<Field>("imu/mag_unbiased", 1, magCb); subs.push_back(magUnbiasedSub);
  auto azMagEnuRadSub = nh.subscribe<Az>("compass/mag/enu/rad", 1,
    cras::bind_front(azCb, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC));
  subs.push_back(azMagEnuRadSub); numAzimuths++;
  auto azMagNedDegSub = nh.subscribe<Az>("compass/mag/ned/deg", 1,
    cras::bind_front(azCb, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC));
  subs.push_back(azMagNedDegSub); numAzimuths++;
  auto azTrueEnuRadSub = nh.subscribe<Az>("compass/true/enu/rad", 1,
    cras::bind_front(azCb, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC));
  subs.push_back(azTrueEnuRadSub); numAzimuths++;
  auto azUtmEnuRadSub = nh.subscribe<Az>("compass/utm/enu/rad", 1,
    cras::bind_front(azCb, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM));
  subs.push_back(azUtmEnuRadSub); numAzimuths++;
  auto azTrueEnuImuSub = nh.subscribe<Imu>("compass/true/enu/imu", 1, imuCb); subs.push_back(azTrueEnuImuSub);
  auto azUtmNedQuatSub = nh.subscribe<Quat>("compass/utm/ned/quat", 1, quatCb); subs.push_back(azUtmNedQuatSub);
  auto azUtmNedPoseSub = nh.subscribe<Pose>("compass/utm/ned/pose", 1, poseCb); subs.push_back(azUtmNedPoseSub);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto tf = std::make_shared<tf2_ros::Buffer>();
  tf->setUsingDedicatedThread(true);

  auto nodelet = createNodelet(log, {}, tf);
  ASSERT_NE(nullptr, nodelet);

  const auto pubTest = [](const ros::Publisher& p) {return p.getNumSubscribers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pubTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for publisher connections.");
  }

  const auto subTest = [](const ros::Subscriber& p) {return p.getNumPublishers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pubTest));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  ros::Time time(1664286802, 187375068);

  // First, publish imu + mag before bias + fix + tf, this should do nothing

  Imu imu;
  imu.header.stamp = time;
  imu.header.frame_id = "imu";
  imu.angular_velocity.x = -0.002507;
  imu.angular_velocity.y = 0.015959;
  imu.angular_velocity.z = 0.044427;
  imu.linear_acceleration.x = 0.108412;
  imu.linear_acceleration.y = 0.520543;
  imu.linear_acceleration.z = -9.605243;
  imu.orientation.x = 0.747476;
  imu.orientation.y = -0.664147;
  imu.orientation.z = 0.013337;
  imu.orientation.w = -0.003273;
  imu.angular_velocity_covariance[0 * 3 + 0] = 0.000436;
  imu.angular_velocity_covariance[1 * 3 + 1] = 0.000436;
  imu.angular_velocity_covariance[2 * 3 + 2] = 0.000436;
  imu.linear_acceleration_covariance[0 * 3 + 0] = 0.0004;
  imu.linear_acceleration_covariance[1 * 3 + 1] = 0.0004;
  imu.linear_acceleration_covariance[2 * 3 + 2] = 0.0004;
  imu.orientation_covariance[0 * 3 + 0] = 0.017453;
  imu.orientation_covariance[1 * 3 + 1] = 0.017453;
  imu.orientation_covariance[2 * 3 + 2] = 0.157080;
  imuPub.publish(imu);

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093;
  mag.magnetic_field.y = -0.538677;
  mag.magnetic_field.z = 0.157033;
  magPub.publish(mag);

  for (
    size_t i = 0;
    i < 5 && (!lastField || !lastImu || !lastQuat || !lastPose || az.size() < numAzimuths)
      && ros::ok() && nodelet->ok();
    ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  // Missing bias + fix + tf, nothing published
  ASSERT_FALSE(lastImu.has_value());
  ASSERT_FALSE(lastQuat.has_value());
  ASSERT_FALSE(lastPose.has_value());
  ASSERT_FALSE(lastField.has_value());
  ASSERT_TRUE(az.empty());

  // Now, publish bias + fix, but not yet tf

  Field bias;
  bias.header.stamp = time;
  bias.header.frame_id = "imu";
  bias.magnetic_field.x = -0.097227663;
  bias.magnetic_field.y = -0.692264333;
  bias.magnetic_field.z = 0;
  magBiasPub.publish(bias);

  Fix fix;
  fix.header.stamp = time;
  fix.header.frame_id = "gps";
  fix.latitude = 50.090806436;
  fix.longitude = 14.133202857;
  fix.altitude = 445.6146;
  fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  fixPub.publish(fix);

  ros::spinOnce();

  // Wait until the latched messages are received
  ros::WallDuration(0.2).sleep();
  ros::spinOnce();

  imuPub.publish(imu);
  magPub.publish(mag);

  for (
    size_t i = 0;
    i < 5 && (!lastField || !lastImu || !lastQuat || !lastPose || az.size() < numAzimuths)
      && ros::ok() && nodelet->ok();
    ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  // Missing tf, nothing published except unbiased magnetometer
  ASSERT_FALSE(lastImu.has_value());
  ASSERT_FALSE(lastQuat.has_value());
  ASSERT_FALSE(lastPose.has_value());
  ASSERT_TRUE(lastField.has_value());
  ASSERT_TRUE(az.empty());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.360320, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, lastField->magnetic_field.z, 1e-6);

  // Publish tf. Now it should have everything.

  lastField.reset();

  geometry_msgs::TransformStamped baseLinkImuTf;
  baseLinkImuTf.header.stamp = time;
  baseLinkImuTf.header.frame_id = "base_link";
  baseLinkImuTf.child_frame_id = "imu";
  baseLinkImuTf.transform.translation.x = 0;
  baseLinkImuTf.transform.translation.y = 0;
  baseLinkImuTf.transform.translation.z = 0.15;
  baseLinkImuTf.transform.rotation.x = 0.7071067811882787;
  baseLinkImuTf.transform.rotation.y = -0.7071067811848163;
  baseLinkImuTf.transform.rotation.z = 7.312301077167311e-14;
  baseLinkImuTf.transform.rotation.w = -7.312301077203115e-14;
  tf->setTransform(baseLinkImuTf, "test", true);

  imuPub.publish(imu);
  magPub.publish(mag);

  for (
    size_t i = 0;
    i < 10 && (!lastField || !lastImu || !lastQuat || !lastPose || az.size() < numAzimuths)
      && ros::ok() && nodelet->ok();
    ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastImu.has_value());
  ASSERT_TRUE(lastQuat.has_value());
  ASSERT_TRUE(lastPose.has_value());
  ASSERT_TRUE(lastField.has_value());
  ASSERT_EQ(numAzimuths, az.size());
  for (const auto& [key, a] : az)
    ASSERT_TRUE(a.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.360320, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, lastField->magnetic_field.z, 1e-6);

  const auto radEnuMag = std::make_tuple(Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  EXPECT_EQ(time, az[radEnuMag]->header.stamp);
  EXPECT_EQ("base_link", az[radEnuMag]->header.frame_id);
  EXPECT_NEAR(3.534008, az[radEnuMag]->azimuth, 1e-6);
  EXPECT_EQ(0.0, az[radEnuMag]->variance);
  EXPECT_EQ(Az::UNIT_RAD, az[radEnuMag]->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, az[radEnuMag]->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, az[radEnuMag]->reference);

  const auto degNedMag = std::make_tuple(Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  EXPECT_EQ(time, az[degNedMag]->header.stamp);
  EXPECT_EQ("base_link", az[degNedMag]->header.frame_id);
  EXPECT_NEAR(90 - angles::to_degrees(3.534008) + 360, az[degNedMag]->azimuth, 1e-4);
  EXPECT_EQ(0.0, az[degNedMag]->variance);
  EXPECT_EQ(Az::UNIT_DEG, az[degNedMag]->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, az[degNedMag]->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, az[degNedMag]->reference);

  const auto radEnuTrue = std::make_tuple(Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  EXPECT_EQ(time, az[radEnuTrue]->header.stamp);
  EXPECT_EQ("base_link", az[radEnuTrue]->header.frame_id);
  EXPECT_NEAR(3.452292, az[radEnuTrue]->azimuth, 1e-6);
  EXPECT_EQ(0.0, az[radEnuTrue]->variance);
  EXPECT_EQ(Az::UNIT_RAD, az[radEnuTrue]->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, az[radEnuTrue]->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, az[radEnuTrue]->reference);

  const auto radEnuUtm = std::make_tuple(Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  EXPECT_EQ(time, az[radEnuUtm]->header.stamp);
  EXPECT_EQ("base_link", az[radEnuUtm]->header.frame_id);
  EXPECT_NEAR(3.440687, az[radEnuUtm]->azimuth, 1e-6);
  EXPECT_EQ(0.0, az[radEnuUtm]->variance);
  EXPECT_EQ(Az::UNIT_RAD, az[radEnuUtm]->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, az[radEnuUtm]->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, az[radEnuUtm]->reference);

  EXPECT_EQ(time, lastImu->header.stamp);
  EXPECT_EQ("base_link", lastImu->header.frame_id);
  // The IMU comes out rotated to be georeferenced, so we can't directly compare the values, but their magnitude should
  // remain the same. Determinants of the covariances should be the same, too.
  tf2::Vector3 v1, v2;
  tf2::fromMsg(imu.angular_velocity, v1);
  tf2::fromMsg(lastImu->angular_velocity, v2);
  EXPECT_NEAR(v1.length(), v2.length(), 1e-6);
  tf2::fromMsg(imu.linear_acceleration, v1);
  tf2::fromMsg(lastImu->linear_acceleration, v2);
  EXPECT_NEAR(v1.length(), v2.length(), 1e-6);
  Imu transImu;
  tf->transform(imu, transImu, "base_link");
  EXPECT_NEAR(cras::getRoll(transImu.orientation), cras::getRoll(lastImu->orientation), 1e-4);
  EXPECT_NEAR(cras::getPitch(transImu.orientation), cras::getPitch(lastImu->orientation), 1e-4);
  EXPECT_NEAR(az[radEnuTrue]->azimuth, angles::normalize_angle_positive(cras::getYaw(lastImu->orientation)), 1e-4);
  EXPECT_NEAR(det(imu.angular_velocity_covariance), det(lastImu->angular_velocity_covariance), 1e-6);
  EXPECT_NEAR(det(imu.linear_acceleration_covariance), det(lastImu->linear_acceleration_covariance), 1e-6);
  // We can't check orientation covariance, it could change

  EXPECT_EQ(time, lastQuat->header.stamp);
  EXPECT_EQ("base_link", lastQuat->header.frame_id);
  EXPECT_NEAR(M_PI_2 - 3.440687 + 2 * M_PI, angles::normalize_angle_positive(cras::getYaw(lastQuat->quaternion)), 1e-6);

  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_NEAR(M_PI_2 - 3.440687 + 2 * M_PI,
    angles::normalize_angle_positive(cras::getYaw(lastPose->pose.pose.orientation)), 1e-6);
  EXPECT_NEAR(0.0, lastPose->pose.covariance[5 * 6 + 5], 1e-6);

  // New data

  az.clear();
  lastImu.reset();
  lastQuat.reset();
  lastPose.reset();
  lastField.reset();
  time = {1664286802, 197458028};

  imu.header.stamp = time;
  imu.angular_velocity.x = -0.007707;
  imu.angular_velocity.y = 0.003923;
  imu.angular_velocity.z = 0.041115;
  imu.linear_acceleration.x = -0.206485;
  imu.linear_acceleration.y = -0.538767;
  imu.linear_acceleration.z = -9.894861;
  imu.orientation.x = 0.747334;
  imu.orientation.y = -0.664305;
  imu.orientation.z = 0.013382;
  imu.orientation.w = -0.003285;
  imuPub.publish(imu);

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200;
  mag.magnetic_field.y = -0.533960;
  mag.magnetic_field.z = 0.149800;
  magPub.publish(mag);

  for (size_t i = 0;
    i < 10 && (!lastField.has_value() || !lastImu.has_value() || az.size() < numAzimuths) && ros::ok() && nodelet->ok();
    ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastImu.has_value());
  ASSERT_TRUE(lastField.has_value());
  ASSERT_EQ(numAzimuths, az.size());
  for (const auto& [key, a] : az)
    ASSERT_TRUE(a.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.361427, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, lastField->magnetic_field.z, 1e-6);

  EXPECT_EQ(time, az[radEnuMag]->header.stamp);
  EXPECT_EQ("base_link", az[radEnuMag]->header.frame_id);
  EXPECT_NEAR(3.544417, az[radEnuMag]->azimuth, 1e-6);
  EXPECT_EQ(0.0, az[radEnuMag]->variance);
  EXPECT_EQ(Az::UNIT_RAD, az[radEnuMag]->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, az[radEnuMag]->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, az[radEnuMag]->reference);

  EXPECT_EQ(time, az[degNedMag]->header.stamp);
  EXPECT_EQ("base_link", az[degNedMag]->header.frame_id);
  EXPECT_NEAR(90 - angles::to_degrees(3.544417) + 360, az[degNedMag]->azimuth, 1e-4);
  EXPECT_EQ(0.0, az[degNedMag]->variance);
  EXPECT_EQ(Az::UNIT_DEG, az[degNedMag]->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, az[degNedMag]->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, az[degNedMag]->reference);

  EXPECT_EQ(time, az[radEnuTrue]->header.stamp);
  EXPECT_EQ("base_link", az[radEnuTrue]->header.frame_id);
  EXPECT_NEAR(3.462701, az[radEnuTrue]->azimuth, 1e-6);
  EXPECT_EQ(0.0, az[radEnuTrue]->variance);
  EXPECT_EQ(Az::UNIT_RAD, az[radEnuTrue]->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, az[radEnuTrue]->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, az[radEnuTrue]->reference);

  EXPECT_EQ(time, az[radEnuUtm]->header.stamp);
  EXPECT_EQ("base_link", az[radEnuUtm]->header.frame_id);
  EXPECT_NEAR(3.451096, az[radEnuUtm]->azimuth, 1e-6);
  EXPECT_EQ(0.0, az[radEnuUtm]->variance);
  EXPECT_EQ(Az::UNIT_RAD, az[radEnuUtm]->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, az[radEnuUtm]->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, az[radEnuUtm]->reference);

  EXPECT_EQ(time, lastImu->header.stamp);
  EXPECT_EQ("base_link", lastImu->header.frame_id);
  // The IMU comes out rotated to be georeferenced, so we can't directly compare the values, but their magnitude should
  // remain the same. Determinants of the covariances should be the same, too.
  tf2::fromMsg(imu.angular_velocity, v1);
  tf2::fromMsg(lastImu->angular_velocity, v2);
  EXPECT_NEAR(v1.length(), v2.length(), 1e-6);
  tf2::fromMsg(imu.linear_acceleration, v1);
  tf2::fromMsg(lastImu->linear_acceleration, v2);
  EXPECT_NEAR(v1.length(), v2.length(), 1e-6);
  tf->transform(imu, transImu, "base_link");
  EXPECT_NEAR(cras::getRoll(transImu.orientation), cras::getRoll(lastImu->orientation), 1e-4);
  EXPECT_NEAR(cras::getPitch(transImu.orientation), cras::getPitch(lastImu->orientation), 1e-4);
  EXPECT_NEAR(az[radEnuTrue]->azimuth, angles::normalize_angle_positive(cras::getYaw(lastImu->orientation)), 1e-4);
  EXPECT_NEAR(det(imu.angular_velocity_covariance), det(lastImu->angular_velocity_covariance), 1e-6);
  EXPECT_NEAR(det(imu.linear_acceleration_covariance), det(lastImu->linear_acceleration_covariance), 1e-6);
  // We can't check orientation covariance, it could change

  EXPECT_EQ(time, lastQuat->header.stamp);
  EXPECT_EQ("base_link", lastQuat->header.frame_id);
  EXPECT_NEAR(M_PI_2 - 3.451096 + 2 * M_PI, angles::normalize_angle_positive(cras::getYaw(lastQuat->quaternion)), 1e-6);

  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_NEAR(M_PI_2 - 3.451096 + 2 * M_PI,
    angles::normalize_angle_positive(cras::getYaw(lastPose->pose.pose.orientation)), 1e-6);
  EXPECT_NEAR(0.0, lastPose->pose.covariance[5 * 6 + 5], 1e-6);
}

TEST(MagnetometerCompassNodelet, InitFromParams)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("publish_utm_azimuth_ned_quat", true);
  pnh.setParam("publish_mag_unbiased", true);
  pnh.setParam("low_pass_ratio", 0.0);  // Disable smoothing
  pnh.setParam("initial_mag_bias_x", -0.097227663);
  pnh.setParam("initial_mag_bias_y", -0.692264333);
  pnh.setParam("initial_mag_bias_z", 0.0);
  pnh.setParam("initial_lat", 50.090806436);
  pnh.setParam("initial_lon", 14.133202857);
  pnh.setParam("initial_alt", 445.6146);

  cras::optional<Quat> lastQuat;
  auto quatCb = [&lastQuat](const Quat::ConstPtr& msg)
  {
    lastQuat = *msg;
  };

  cras::optional<Field> lastField;
  auto magCb = [&lastField](const Field::ConstPtr& msg)
  {
    lastField = *msg;
  };

  std::list<ros::Publisher> pubs;
  auto imuPub = nh.advertise<Imu>("imu/data", 1); pubs.push_back(imuPub);
  auto magPub = nh.advertise<Field>("imu/mag", 1); pubs.push_back(magPub);

  std::list<ros::Subscriber> subs;
  size_t numAzimuths {0u};
  auto magUnbiasedSub = nh.subscribe<Field>("imu/mag_unbiased", 1, magCb); subs.push_back(magUnbiasedSub);
  auto azUtmNedQuatSub = nh.subscribe<Quat>("compass/utm/ned/quat", 1, quatCb); subs.push_back(azUtmNedQuatSub);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto tf = std::make_shared<tf2_ros::Buffer>();
  tf->setUsingDedicatedThread(true);

  auto nodelet = createNodelet(log, {}, tf);
  ASSERT_NE(nullptr, nodelet);

  const auto pubTest = [](const ros::Publisher& p) {return p.getNumSubscribers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pubTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for publisher connections.");
  }

  const auto subTest = [](const ros::Subscriber& p) {return p.getNumPublishers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pubTest));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  ros::Time time(1664286802, 187375068);

  geometry_msgs::TransformStamped baseLinkImuTf;
  baseLinkImuTf.header.stamp = time;
  baseLinkImuTf.header.frame_id = "base_link";
  baseLinkImuTf.child_frame_id = "imu";
  baseLinkImuTf.transform.translation.x = 0;
  baseLinkImuTf.transform.translation.y = 0;
  baseLinkImuTf.transform.translation.z = 0.15;
  baseLinkImuTf.transform.rotation.x = 0.7071067811882787;
  baseLinkImuTf.transform.rotation.y = -0.7071067811848163;
  baseLinkImuTf.transform.rotation.z = 7.312301077167311e-14;
  baseLinkImuTf.transform.rotation.w = -7.312301077203115e-14;
  tf->setTransform(baseLinkImuTf, "test", true);

  // Publish imu + mag + tf without bias + fix, which should be substituted by params

  Imu imu;
  imu.header.stamp = time;
  imu.header.frame_id = "imu";
  imu.angular_velocity.x = -0.002507;
  imu.angular_velocity.y = 0.015959;
  imu.angular_velocity.z = 0.044427;
  imu.linear_acceleration.x = 0.108412;
  imu.linear_acceleration.y = 0.520543;
  imu.linear_acceleration.z = -9.605243;
  imu.orientation.x = 0.747476;
  imu.orientation.y = -0.664147;
  imu.orientation.z = 0.013337;
  imu.orientation.w = -0.003273;
  imu.angular_velocity_covariance[0 * 3 + 0] = 0.000436;
  imu.angular_velocity_covariance[1 * 3 + 1] = 0.000436;
  imu.angular_velocity_covariance[2 * 3 + 2] = 0.000436;
  imu.linear_acceleration_covariance[0 * 3 + 0] = 0.0004;
  imu.linear_acceleration_covariance[1 * 3 + 1] = 0.0004;
  imu.linear_acceleration_covariance[2 * 3 + 2] = 0.0004;
  imu.orientation_covariance[0 * 3 + 0] = 0.017453;
  imu.orientation_covariance[1 * 3 + 1] = 0.017453;
  imu.orientation_covariance[2 * 3 + 2] = 0.157080;
  imuPub.publish(imu);

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093;
  mag.magnetic_field.y = -0.538677;
  mag.magnetic_field.z = 0.157033;
  magPub.publish(mag);

  for (size_t i = 0; i < 10 && (!lastField || !lastQuat) && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastQuat.has_value());
  ASSERT_TRUE(lastField.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.360320, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, lastField->magnetic_field.z, 1e-6);

  EXPECT_EQ(time, lastQuat->header.stamp);
  EXPECT_EQ("base_link", lastQuat->header.frame_id);
  EXPECT_NEAR(M_PI_2 - 3.440687 + 2 * M_PI, angles::normalize_angle_positive(cras::getYaw(lastQuat->quaternion)), 1e-6);
}

TEST(MagnetometerCompassNodelet, SubscribeMagUnbiased)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("publish_utm_azimuth_ned_quat", true);
  pnh.setParam("subscribe_mag_unbiased", true);
  pnh.setParam("low_pass_ratio", 0.0);  // Disable smoothing
  pnh.setParam("initial_lat", 50.090806436);
  pnh.setParam("initial_lon", 14.133202857);
  pnh.setParam("initial_alt", 445.6146);

  cras::optional<Quat> lastQuat;
  auto quatCb = [&lastQuat](const Quat::ConstPtr& msg)
  {
    lastQuat = *msg;
  };

  std::list<ros::Publisher> pubs;
  auto imuPub = nh.advertise<Imu>("imu/data", 1); pubs.push_back(imuPub);
  auto magPub = nh.advertise<Field>("imu/mag_unbiased", 1); pubs.push_back(magPub);

  std::list<ros::Subscriber> subs;
  auto azUtmNedQuatSub = nh.subscribe<Quat>("compass/utm/ned/quat", 1, quatCb); subs.push_back(azUtmNedQuatSub);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto tf = std::make_shared<tf2_ros::Buffer>();
  tf->setUsingDedicatedThread(true);

  auto nodelet = createNodelet(log, {}, tf);
  ASSERT_NE(nullptr, nodelet);

  const auto pubTest = [](const ros::Publisher& p) {return p.getNumSubscribers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pubTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for publisher connections.");
  }

  const auto subTest = [](const ros::Subscriber& p) {return p.getNumPublishers() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pubTest));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  ros::Time time(1664286802, 187375068);

  geometry_msgs::TransformStamped baseLinkImuTf;
  baseLinkImuTf.header.stamp = time;
  baseLinkImuTf.header.frame_id = "base_link";
  baseLinkImuTf.child_frame_id = "imu";
  baseLinkImuTf.transform.translation.x = 0;
  baseLinkImuTf.transform.translation.y = 0;
  baseLinkImuTf.transform.translation.z = 0.15;
  baseLinkImuTf.transform.rotation.x = 0.7071067811882787;
  baseLinkImuTf.transform.rotation.y = -0.7071067811848163;
  baseLinkImuTf.transform.rotation.z = 7.312301077167311e-14;
  baseLinkImuTf.transform.rotation.w = -7.312301077203115e-14;
  tf->setTransform(baseLinkImuTf, "test", true);

  // Publish imu + mag_unbiased + tf without fix, which should be substituted by params

  Imu imu;
  imu.header.stamp = time;
  imu.header.frame_id = "imu";
  imu.angular_velocity.x = -0.002507;
  imu.angular_velocity.y = 0.015959;
  imu.angular_velocity.z = 0.044427;
  imu.linear_acceleration.x = 0.108412;
  imu.linear_acceleration.y = 0.520543;
  imu.linear_acceleration.z = -9.605243;
  imu.orientation.x = 0.747476;
  imu.orientation.y = -0.664147;
  imu.orientation.z = 0.013337;
  imu.orientation.w = -0.003273;
  imu.angular_velocity_covariance[0 * 3 + 0] = 0.000436;
  imu.angular_velocity_covariance[1 * 3 + 1] = 0.000436;
  imu.angular_velocity_covariance[2 * 3 + 2] = 0.000436;
  imu.linear_acceleration_covariance[0 * 3 + 0] = 0.0004;
  imu.linear_acceleration_covariance[1 * 3 + 1] = 0.0004;
  imu.linear_acceleration_covariance[2 * 3 + 2] = 0.0004;
  imu.orientation_covariance[0 * 3 + 0] = 0.017453;
  imu.orientation_covariance[1 * 3 + 1] = 0.017453;
  imu.orientation_covariance[2 * 3 + 2] = 0.157080;
  imuPub.publish(imu);

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093 - -0.097227663;
  mag.magnetic_field.y = -0.538677 - -0.692264333;
  mag.magnetic_field.z = 0.157033 - 0.0;
  magPub.publish(mag);

  for (size_t i = 0; i < 10 && (!lastQuat) && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastQuat.has_value());

  EXPECT_EQ(time, lastQuat->header.stamp);
  EXPECT_EQ("base_link", lastQuat->header.frame_id);
  EXPECT_NEAR(M_PI_2 - 3.440687 + 2 * M_PI, angles::normalize_angle_positive(cras::getYaw(lastQuat->quaternion)), 1e-6);
}


TEST(MagnetometerCompassNodelet, ThrowWhenSubPubBias)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("subscribe_mag_unbiased", true);
  pnh.setParam("publish_mag_unbiased", true);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  EXPECT_THROW(createNodelet(log), std::runtime_error);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Remove the program name from argv because the nodelet handling code does not expect it
  argc -= 1;
  argv += 1;
  ros::removeROSArgs(argc, argv, my_argv);
  ros::init(argc, argv, "test_magnetometer_compass_nodelet");

  ros::NodeHandle nh;  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
