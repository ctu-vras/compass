// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for transformations of compass_msgs.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cmath>
#include <memory>
#include <string>

#include <angles/angles.h>
#include <class_loader/class_loader_core.hpp>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>
#include <ros/names.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

namespace ros
{
namespace names
{
extern void init(const ros::M_string& remappings);
}
}

using Az = compass_msgs::Azimuth;

ros::V_string my_argv;

template<typename NodeletType = cras::Nodelet>
std::unique_ptr<NodeletType> createNodelet(const cras::LogHelperPtr& log,
  const ros::M_string& remaps = {},
  const std::shared_ptr<tf2_ros::Buffer>& tf = nullptr)
{
  // Declaration order of these variables is important to make sure they can be properly stopped and destroyed.
  auto nodelet = class_loader::impl::createInstance<nodelet::Nodelet>(
    "compass_conversions::CompassTransformerNodelet", nullptr);
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

TEST(CompassTransformerNodelet, BasicConversion)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, TfConversion)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "deg");
  pnh.setParam("target_orientation", "ned");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_frame", "test2");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>();
  tfBuffer->setTransform(tf, "test", true);

  auto nodelet = createNodelet(log, {}, tfBuffer);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ("test2", lastAz->header.frame_id);
  EXPECT_NEAR(180.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_DEG, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, TfConversionFail)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "deg");
  pnh.setParam("target_orientation", "ned");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_frame", "test_nonexistent");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>();
  tfBuffer->setTransform(tf, "test", true);

  auto nodelet = createNodelet(log, {}, tfBuffer);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_FALSE(lastAz.has_value());
}

TEST(CompassTransformerNodelet, FixMissing)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_reference", "utm");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_FALSE(lastAz.has_value());
}

TEST(CompassTransformerNodelet, FixFromParams)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_reference", "geographic");
  pnh.setParam("initial_lat", 51.0);
  pnh.setParam("initial_lon", 15.0);
  pnh.setParam("initial_alt", 200.0);

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = cras::parseTime("2024-11-18T13:00:00Z");
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(-5.333 + 360, angles::to_degrees(lastAz->azimuth), 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, FixFromMsg)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_reference", "geographic");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto fixPub = nh.advertise<sensor_msgs::NavSatFix>("fix", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 &&
    (azimuthPub.getNumSubscribers() == 0 || fixPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for fix and azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(fixPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");

  sensor_msgs::NavSatFix fix;
  fix.header.stamp = time;
  fix.header.frame_id = "test";
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  fix.altitude = 200.0;
  fixPub.publish(fix);

  // Wait until the fix message is received
  for (size_t i = 0; i < 10; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  Az in;
  in.header.stamp = time;
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(-5.333 + 360, angles::to_degrees(lastAz->azimuth), 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubImuNameDetect)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = nh.advertise<sensor_msgs::Imu>("imu/data/mag/ned/imu", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  const ros::M_string remaps = {
    {pnh.resolveName("azimuth_in"), nh.resolveName("imu/data/mag/ned/imu")},
  };

  auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  sensor_msgs::Imu in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubImuNoDetect)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("input_orientation", "ned");
  pnh.setParam("input_reference", "magnetic");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<sensor_msgs::Imu>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  sensor_msgs::Imu in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubPoseNameDetect)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose/mag/ned/pose", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  const ros::M_string remaps = {
    {pnh.resolveName("azimuth_in"), nh.resolveName("pose/mag/ned/pose")},
  };

  auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  geometry_msgs::PoseWithCovarianceStamped in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.pose.pose.orientation);
  in.pose.covariance[5 * 6 + 5] = 4;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubPoseNoDetect)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("input_orientation", "ned");
  pnh.setParam("input_reference", "magnetic");

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<geometry_msgs::PoseWithCovarianceStamped>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  geometry_msgs::PoseWithCovarianceStamped in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.pose.pose.orientation);
  in.pose.covariance[5 * 6 + 5] = 4.0;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubQuatNameDetect)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("input_variance", 4.0);

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = nh.advertise<geometry_msgs::QuaternionStamped>("quat/mag/ned/quat", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  const ros::M_string remaps = {
    {pnh.resolveName("azimuth_in"), nh.resolveName("quat/mag/ned/quat")},
  };

  auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  geometry_msgs::QuaternionStamped in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.quaternion);

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubQuatNoDetect)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("input_orientation", "ned");
  pnh.setParam("input_reference", "magnetic");
  pnh.setParam("input_variance", 4.0);

  cras::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<geometry_msgs::QuaternionStamped>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<Az>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  geometry_msgs::QuaternionStamped in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.quaternion);

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, PubImu)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_type", "imu");

  cras::optional<sensor_msgs::Imu> lastAz;
  auto cb = [&lastAz](const sensor_msgs::Imu::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<sensor_msgs::Imu>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->orientation_covariance[2 * 3 + 2]);
}

TEST(CompassTransformerNodelet, PubImuSuffix)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_type", "imu");
  pnh.setParam("target_append_suffix", true);

  cras::optional<sensor_msgs::Imu> lastAz;
  auto cb = [&lastAz](const sensor_msgs::Imu::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<sensor_msgs::Imu>("azimuth_out/mag/enu/imu", 1, cb);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->orientation_covariance[2 * 3 + 2]);
}

TEST(CompassTransformerNodelet, PubPose)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_type", "pose");

  cras::optional<geometry_msgs::PoseWithCovarianceStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->pose.pose.orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->pose.covariance[5 * 6 + 5]);
}

TEST(CompassTransformerNodelet, PubPoseSuffix)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_type", "pose");
  pnh.setParam("target_append_suffix", true);

  cras::optional<geometry_msgs::PoseWithCovarianceStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("azimuth_out/mag/enu/pose", 1, cb);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->pose.pose.orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->pose.covariance[5 * 6 + 5]);
}

TEST(CompassTransformerNodelet, PubQuat)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_type", "quaternion");

  cras::optional<geometry_msgs::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::QuaternionStamped::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<geometry_msgs::QuaternionStamped>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->quaternion), 1e-6);
}

TEST(CompassTransformerNodelet, PubQuatSuffix)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_type", "quaternion");
  pnh.setParam("target_append_suffix", true);

  cras::optional<geometry_msgs::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::QuaternionStamped::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<Az>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<geometry_msgs::QuaternionStamped>("azimuth_out/mag/enu/quat", 1, cb);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  Az in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->quaternion), 1e-6);
}

TEST(CompassTransformerNodelet, CrossType)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  pnh.deleteParam("");
  pnh.setParam("target_unit", "rad");
  pnh.setParam("target_orientation", "enu");
  pnh.setParam("target_reference", "magnetic");
  pnh.setParam("target_type", "quaternion");
  pnh.setParam("input_orientation", "ned");
  pnh.setParam("input_reference", "magnetic");

  cras::optional<geometry_msgs::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::QuaternionStamped::ConstPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = pnh.advertise<sensor_msgs::Imu>("azimuth_in", 1);
  auto azimuthSub = pnh.subscribe<geometry_msgs::QuaternionStamped>("azimuth_out", 1, cb);

  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, nodelet);

  for (size_t i = 0; i < 1000 && (azimuthPub.getNumSubscribers() == 0 || azimuthSub.getNumPublishers() == 0); ++i)
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(0.2, "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub.getNumSubscribers(), 0);
  ASSERT_GT(azimuthSub.getNumPublishers(), 0);

  sensor_msgs::Imu in;
  in.header.stamp = ros::Time::now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub.publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && ros::ok() && nodelet->ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->quaternion), 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Remove the program name from argv because the nodelet handling code does not expect it
  argc -= 1;
  argv += 1;
  ros::removeROSArgs(argc, argv, my_argv);
  uint32_t initOptions = ros::init_options::AnonymousName;
  ros::init(argc, argv, "test_compass_transformer_nodelet", initOptions);
  // Anonymous nodes have a problem that topic remappings of type ~a:=b are resolved against the node name without the
  // anonymous part. Fix that by running names::init() again after ros::init() finishes and the full node name is known.
  // This was reported and a fix provided in https://github.com/ros/ros_comm/issues/2324, but the fix never landed.
  ros::names::init(ros::names::getUnresolvedRemappings());

  ros::NodeHandle nh;  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
