// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for transformations of compass_interfaces.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <angles/angles.h>
#include <builtin_interfaces/msg/time.hpp>
#include <compass_conversions/compass_transformer.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/test_utils.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_ros.hpp>

using Az = compass_interfaces::msg::Azimuth;
using namespace std::chrono_literals;

std::shared_ptr<compass_conversions::CompassTransformerNodelet> createNodelet(
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
{
  return std::make_shared<compass_conversions::CompassTransformerNodelet>(node_options);
}

class CompassTransformerNodelet : public cras::RclcppTestFixture {};

TEST_F(CompassTransformerNodelet, BasicConversion)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, TfConversion)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "deg");
  node_options.append_parameter_override("target_orientation", "ned");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_frame", "test2");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tfBuffer->setCreateTimerInterface(timer_interface);

  tfBuffer->setTransform(tf, "test", true);

  node->setBuffer(tfBuffer, false);
  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastAz.has_value());

  // TODO(BUG) buffer->transform timestamp loses some precision so not equal anymore...
  // EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ("test2", lastAz->header.frame_id);
  EXPECT_NEAR(180.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_DEG, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST_F(CompassTransformerNodelet, TfConversionFail)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "deg");
  node_options.append_parameter_override("target_orientation", "ned");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_frame", "test_nonexistent");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tfBuffer->setCreateTimerInterface(timer_interface);

  tfBuffer->setTransform(tf, "test", true);

  node->setBuffer(tfBuffer, false);
  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
  ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  // TODO(BUG) when the spinning is uncommented, there is a crash
  /*
  for (size_t i = 0; i != 500 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(10ms);
  }
  */
  ASSERT_FALSE(lastAz.has_value());
}

TEST_F(CompassTransformerNodelet, FixMissing)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_reference", "utm");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_FALSE(lastAz.has_value());
}

TEST_F(CompassTransformerNodelet, FixFromParams)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_reference", "geographic");
  node_options.append_parameter_override("initial_lat", 51.0);
  node_options.append_parameter_override("initial_lon", 15.0);
  node_options.append_parameter_override("initial_alt", 200.0);

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = cras::parseTime("2024-11-18T13:00:00Z");
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, FixFromMsg)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_reference", "geographic");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto fixPub = node->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 &&
    (azimuthPub->get_subscription_count() == 0 || fixPub->get_subscription_count() == 0 ||
      azimuthSub->get_publisher_count() == 0); ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for fix and azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(fixPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");

  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = time;
  fix.header.frame_id = "test";
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  fix.altitude = 200.0;
  fixPub->publish(fix);

  // Wait until the fix message is received
  for (size_t i = 0; i < 10; ++i)
  {
    executor.spin_all(10ms);
  }

  Az in;
  in.header.stamp = time;
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, SubImuNameDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.arguments({"--ros-args", "-r", "azimuth_in/imu:=imu/data/mag/ned/imu"});

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>(
    "imu/data/mag/ned/imu", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  sensor_msgs::msg::Imu in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 1000 && !lastAz.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, SubImuNoDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("input_orientation", "ned");
  node_options.append_parameter_override("input_reference", "magnetic");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("azimuth_in/imu", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  sensor_msgs::msg::Imu in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, SubPoseNameDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.arguments({"--ros-args", "-r", "azimuth_in/pose:=pose/mag/ned/pose"});

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose/mag/ned/pose", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  geometry_msgs::msg::PoseWithCovarianceStamped in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.pose.pose.orientation);
  in.pose.covariance[5 * 6 + 5] = 4;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, SubPoseNoDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("input_orientation", "ned");
  node_options.append_parameter_override("input_reference", "magnetic");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "azimuth_in/pose", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  geometry_msgs::msg::PoseWithCovarianceStamped in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.pose.pose.orientation);
  in.pose.covariance[5 * 6 + 5] = 4.0;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, SubQuatNameDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("input_variance", 4.0);
  node_options.arguments({"--ros-args", "-r", "azimuth_in/quat:=quat/mag/ned/quat"});

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "quat/mag/ned/quat", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  geometry_msgs::msg::QuaternionStamped in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.quaternion);

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, SubQuatNoDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("input_orientation", "ned");
  node_options.append_parameter_override("input_reference", "magnetic");
  node_options.append_parameter_override("input_variance", 4.0);

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;

  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("azimuth_in/quat", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  geometry_msgs::msg::QuaternionStamped in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.quaternion);

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
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

TEST_F(CompassTransformerNodelet, PubImu)  // NOLINT
{
  rclcpp::NodeOptions node_options;

  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "imu");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<sensor_msgs::msg::Imu> lastAz;
  auto cb = [&lastAz](const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<sensor_msgs::msg::Imu>("azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->orientation_covariance[2 * 3 + 2]);
}

TEST_F(CompassTransformerNodelet, PubImuSuffix)  // NOLINT
{
  rclcpp::NodeOptions node_options;

  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "imu");
  node_options.append_parameter_override("target_append_suffix", true);

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<sensor_msgs::msg::Imu> lastAz;
  auto cb = [&lastAz](const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<sensor_msgs::msg::Imu>(
    "azimuth_out/mag/enu/imu", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->orientation_covariance[2 * 3 + 2]);
}

TEST_F(CompassTransformerNodelet, PubPose)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "pose");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->pose.pose.orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->pose.covariance[5 * 6 + 5]);
}

TEST_F(CompassTransformerNodelet, PubPoseSuffix)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "pose");
  node_options.append_parameter_override("target_append_suffix", true);

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };
  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "azimuth_out/mag/enu/pose", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->pose.pose.orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->pose.covariance[5 * 6 + 5]);
}

TEST_F(CompassTransformerNodelet, PubQuat)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "quaternion");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->quaternion), 1e-6);
}

TEST_F(CompassTransformerNodelet, PubQuatSuffix)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "quaternion");
  node_options.append_parameter_override("target_append_suffix", true);

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };
  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "azimuth_out/mag/enu/quat", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->quaternion), 1e-6);
}

TEST_F(CompassTransformerNodelet, CrossType)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "quaternion");
  node_options.append_parameter_override("input_orientation", "ned");
  node_options.append_parameter_override("input_reference", "magnetic");

  auto node = createNodelet(node_options);
  ASSERT_NE(nullptr, node);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };
  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("azimuth_in/imu", rclcpp::SystemDefaultsQoS(pub_qos));
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "azimuth_out", rclcpp::SensorDataQoS(sub_qos), cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0);
    ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200.,
      "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  sensor_msgs::msg::Imu in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, cras::getYaw(lastAz->quaternion), 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
