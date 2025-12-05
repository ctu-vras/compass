// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for MagnetometerCompass.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <builtin_interfaces/msg/time.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <magnetometer_compass/magnetometer_compass.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/buffer.hpp>

using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Field = sensor_msgs::msg::MagneticField;

TEST(MagnetometerCompass, ComputeAzimuth)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  auto tf = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  tf->setUsingDedicatedThread(true);

  magnetometer_compass::MagnetometerCompass compass(node, "base_link", tf);
  compass.setLowPassRatio(0.0);

  builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  // First, publish imu + mag before tf, this should do nothing

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

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093 - -0.097227663;
  mag.magnetic_field.y = -0.538677 - -0.692264333;
  mag.magnetic_field.z = 0.157033 - 0;

  auto maybeAzimuth = compass.computeAzimuth(imu, mag);

  // Missing tf, nothing published
  ASSERT_FALSE(maybeAzimuth.has_value());
  ASSERT_FALSE(maybeAzimuth.error().empty());

  // Publish tf. Now it should have everything.

  geometry_msgs::msg::TransformStamped baseLinkImuTf;
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

  maybeAzimuth = compass.computeAzimuth(imu, mag);
  ASSERT_TRUE(maybeAzimuth.has_value());

  EXPECT_EQ(time, maybeAzimuth->header.stamp);
  EXPECT_EQ("base_link", maybeAzimuth->header.frame_id);
  EXPECT_NEAR(3.534008 - M_PI_2, maybeAzimuth->azimuth, 1e-6);
  EXPECT_EQ(0.0, maybeAzimuth->variance);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // New data
  time.sec = 1664286802;
  time.nanosec = 197458028;

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

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200 - -0.097227663;
  mag.magnetic_field.y = -0.533960 - -0.692264333;
  mag.magnetic_field.z = 0.149800 - 0;

  maybeAzimuth = compass.computeAzimuth(imu, mag);

  ASSERT_TRUE(maybeAzimuth.has_value());

  EXPECT_EQ(time, maybeAzimuth->header.stamp);
  EXPECT_EQ("base_link", maybeAzimuth->header.frame_id);
  EXPECT_NEAR(3.544417 - M_PI_2, maybeAzimuth->azimuth, 1e-6);
  EXPECT_EQ(0.0, maybeAzimuth->variance);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);
}

TEST(MagnetometerCompass, ConfigFromParams)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node", rclcpp::NodeOptions().allow_undeclared_parameters(true));

  auto tf = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  tf->setUsingDedicatedThread(true);

  magnetometer_compass::MagnetometerCompass compass(node, "base_link", tf);

  rclcpp::Parameter parameter1("low_pass_ratio", 0.0);
  node.set_parameter(parameter1);
  rclcpp::Parameter parameter2("initial_variance", 4.0);
  node.set_parameter(parameter2);

  compass.configFromParams();

  builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  geometry_msgs::msg::TransformStamped baseLinkImuTf;
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

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093 - -0.097227663;
  mag.magnetic_field.y = -0.538677 - -0.692264333;
  mag.magnetic_field.z = 0.157033 - 0;

  auto maybeAzimuth = compass.computeAzimuth(imu, mag);

  ASSERT_TRUE(maybeAzimuth.has_value());

  EXPECT_EQ(time, maybeAzimuth->header.stamp);
  EXPECT_EQ("base_link", maybeAzimuth->header.frame_id);
  EXPECT_NEAR(3.534008 - M_PI_2, maybeAzimuth->azimuth, 1e-6);
  EXPECT_EQ(4.0, maybeAzimuth->variance);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // New data
  time.sec = 1664286802;
  time.nanosec = 197458028;

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

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200 - -0.097227663;
  mag.magnetic_field.y = -0.533960 - -0.692264333;
  mag.magnetic_field.z = 0.149800 - 0;

  maybeAzimuth = compass.computeAzimuth(imu, mag);

  ASSERT_TRUE(maybeAzimuth.has_value());

  EXPECT_EQ(time, maybeAzimuth->header.stamp);
  EXPECT_EQ("base_link", maybeAzimuth->header.frame_id);
  EXPECT_NEAR(3.544417 - M_PI_2, maybeAzimuth->azimuth, 1e-6);
  EXPECT_EQ(4.0, maybeAzimuth->variance);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);
}

TEST(MagnetometerCompass, Reset)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  auto tf = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  tf->setUsingDedicatedThread(true);

  magnetometer_compass::MagnetometerCompass compass(node, "base_link", tf);
  compass.setLowPassRatio(0.5);

  builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  geometry_msgs::msg::TransformStamped baseLinkImuTf;
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

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093 - -0.097227663;
  mag.magnetic_field.y = -0.538677 - -0.692264333;
  mag.magnetic_field.z = 0.157033 - 0;

  auto maybeAzimuth = compass.computeAzimuth(imu, mag);

  ASSERT_TRUE(maybeAzimuth.has_value());

  EXPECT_EQ(time, maybeAzimuth->header.stamp);
  EXPECT_EQ("base_link", maybeAzimuth->header.frame_id);
  EXPECT_NEAR(3.534008 - M_PI_2, maybeAzimuth->azimuth, 1e-6);
  EXPECT_EQ(0.0, maybeAzimuth->variance);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // Without resetting the filter, the next azimuth would be a mean of the two.
  compass.reset();

  // New data
  time.sec = 1664286802;
  time.nanosec = 197458028;

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

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200 - -0.097227663;
  mag.magnetic_field.y = -0.533960 - -0.692264333;
  mag.magnetic_field.z = 0.149800 - 0;

  maybeAzimuth = compass.computeAzimuth(imu, mag);

  ASSERT_TRUE(maybeAzimuth.has_value());

  EXPECT_EQ(time, maybeAzimuth->header.stamp);
  EXPECT_EQ("base_link", maybeAzimuth->header.frame_id);
  EXPECT_NEAR(3.544417 - M_PI_2, maybeAzimuth->azimuth, 1e-6);
  EXPECT_EQ(0.0, maybeAzimuth->variance);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}
