// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer_bias_remover.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include <cras_cpp_common/test_utils.hpp>
#include <magnetometer_pipeline/magnetometer_bias_remover_nodelet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

using Field = sensor_msgs::msg::MagneticField;

using namespace std::chrono_literals;

std::shared_ptr<magnetometer_pipeline::MagnetometerBiasRemoverNodelet> createNodelet(
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
{
  return std::make_shared<magnetometer_pipeline::MagnetometerBiasRemoverNodelet>(node_options);
}

class MagnetometerBiasRemoverNodelet : public cras::RclcppTestFixture {};

TEST_F(MagnetometerBiasRemoverNodelet, Basic)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  auto node = createNodelet();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Field> lastField;
  auto magCb = [&lastField](const Field::ConstSharedPtr& msg)
  {
    lastField = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  std::list<rclcpp::PublisherBase::SharedPtr> pubs;
  auto magPub = node->create_publisher<Field>("imu/mag", rclcpp::SystemDefaultsQoS(pub_qos));
  pubs.push_back(magPub);
  auto magBiasPub = node->create_publisher<Field>("imu/mag_bias", rclcpp::SystemDefaultsQoS(pub_qos).transient_local());
  pubs.push_back(magBiasPub);

  std::list<rclcpp::SubscriptionBase::SharedPtr> subs;

  auto magUnbiasedSub = node->create_subscription<Field>("imu/mag_unbiased", rclcpp::SensorDataQoS(sub_qos), magCb);
  subs.push_back(magUnbiasedSub);

  const auto pubTest = [](const rclcpp::PublisherBase::SharedPtr p) {return p->get_subscription_count() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pubTest); ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for publisher connections.");
  }

  const auto subTest = [](const rclcpp::SubscriptionBase::SharedPtr p) {return p->get_publisher_count() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pubTest));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093;
  mag.magnetic_field.y = -0.538677;
  mag.magnetic_field.z = 0.157033;
  magPub->publish(mag);

  for (size_t i = 0; i < 5 && !lastField.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
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
  magBiasPub->publish(bias);

  executor.spin_once();

  // Wait until the latched messages are received
  executor.spin_all(200ms);

  magPub->publish(mag);

  for (size_t i = 0; i < 10 && !lastField.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastField.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.360320, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, lastField->magnetic_field.z, 1e-6);

  // New data
  lastField.reset();
  time.sec = 1664286802;
  time.nanosec = 197458028;

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200;
  mag.magnetic_field.y = -0.533960;
  mag.magnetic_field.z = 0.149800;
  magPub->publish(mag);

  for (size_t i = 0; i < 10 && !lastField.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastField.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.361427, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, lastField->magnetic_field.z, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
