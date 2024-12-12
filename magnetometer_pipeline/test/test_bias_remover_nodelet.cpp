// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer_bias_remover.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>

#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

using Field = sensor_msgs::MagneticField;

TEST(MagnetometerBiasRemoverNodelet, Basic)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  ros::NodeHandle nh, pnh("~");

  cras::optional<Field> lastField;
  auto magCb = [&lastField](const Field::ConstPtr& msg)
  {
    lastField = *msg;
  };

  std::list<ros::Publisher> pubs;
  auto magPub = nh.advertise<Field>("imu/mag", 1); pubs.push_back(magPub);
  auto magBiasPub = nh.advertise<Field>("imu/mag_bias", 1, true); pubs.push_back(magBiasPub);

  std::list<ros::Subscriber> subs;
  auto magUnbiasedSub = nh.subscribe<Field>("imu/mag_unbiased", 1, magCb); subs.push_back(magUnbiasedSub);

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

  ros::Time time(1664286802, 187375068);

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093;
  mag.magnetic_field.y = -0.538677;
  mag.magnetic_field.z = 0.157033;
  magPub.publish(mag);

  for (size_t i = 0; i < 5 && !lastField.has_value() && ros::ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  // Missing bias, nothing published
  ASSERT_FALSE(lastField.has_value());

  // Publish bias. Now it should have everything.

  Field bias;
  bias.header.stamp = time;
  bias.header.frame_id = "imu";
  bias.magnetic_field.x = -0.097227663;
  bias.magnetic_field.y = -0.692264333;
  bias.magnetic_field.z = 0;
  magBiasPub.publish(bias);

  ros::spinOnce();

  // Wait until the latched messages are received
  ros::WallDuration(0.2).sleep();
  ros::spinOnce();

  magPub.publish(mag);

  for (size_t i = 0; i < 10 && !lastField.has_value() && ros::ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastField.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.360320, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, lastField->magnetic_field.z, 1e-6);

  // New data

  lastField.reset();
  time = {1664286802, 197458028};

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200;
  mag.magnetic_field.y = -0.533960;
  mag.magnetic_field.z = 0.149800;
  magPub.publish(mag);

  for (size_t i = 0; i < 10 && !lastField.has_value() && ros::ok(); ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  ASSERT_TRUE(lastField.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.361427, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, lastField->magnetic_field.z, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_magnetometer_bias_remover");
  ros::NodeHandle nh;  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
