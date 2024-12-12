// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for azimuth visualization.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using Az = compass_msgs::Azimuth;
using Quat = geometry_msgs::QuaternionStamped;
using Pose = geometry_msgs::PoseWithCovarianceStamped;
using Fix = sensor_msgs::NavSatFix;

TEST(VisualizeAzimuthNodelet, Basic)  // NOLINT
{
  ros::NodeHandle nh, pnh("~");

  cras::optional<Pose> lastPose;
  auto poseCb = [&lastPose](const Pose::ConstPtr& msg)
  {
    lastPose = *msg;
  };

  std::list<ros::Publisher> pubs;
  auto azPub = nh.advertise<Az>("visualize_azimuth/azimuth", 1); pubs.push_back(azPub);
  auto fixPub = nh.advertise<Fix>("gps/fix", 1, true); pubs.push_back(fixPub);

  std::list<ros::Subscriber> subs;
  auto visSub = nh.subscribe<Pose>("visualize_azimuth/azimuth_vis", 1, poseCb); subs.push_back(visSub);

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

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

  ros::Time time = cras::parseTime("2024-11-18T13:00:00Z");

  Az azimuth;
  azimuth.header.frame_id = "base_link";
  azimuth.header.stamp = time;
  azimuth.azimuth = 0;
  azimuth.variance = 0;
  azimuth.unit = Az::UNIT_RAD;
  azimuth.orientation = Az::ORIENTATION_ENU;
  azimuth.reference = Az::REFERENCE_UTM;
  azPub.publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && ros::ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.w, 1e-6);

  lastPose.reset();
  azimuth.azimuth = M_PI_2;
  azPub.publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && ros::ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(0, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, lastPose->pose.pose.orientation.w, 1e-6);

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azPub.publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && ros::ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(-M_SQRT1_2, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.w, 1e-6);

  // We haven't yet supplied a fix, so change of reference is not possible

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azimuth.reference = Az::REFERENCE_MAGNETIC;
  azPub.publish(azimuth);

  for (size_t i = 0; i < 5 && !lastPose.has_value() && ros::ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_FALSE(lastPose.has_value());

  Fix fix;
  fix.header.stamp = time;
  fix.header.frame_id = "gps";
  fix.latitude = 50.090806436;
  fix.longitude = 14.133202857;
  fix.altitude = 445.6146;
  fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  fixPub.publish(fix);

  // Wait until the fix arrives
  ros::WallDuration(0.2).sleep();

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azPub.publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && ros::ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(-0.671109, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(0.741358, lastPose->pose.pose.orientation.w, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_visualize_azimuth_nodelet");
  ros::NodeHandle nh;  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
