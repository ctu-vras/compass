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
    rclcpp::NodeOptions node_options = rclcpp::NodeOptions()) {
  return std::make_shared<magnetometer_pipeline::MagnetometerBiasRemoverNodelet>(node_options);
}

class MagnetometerBiasRemoverNodelet : public cras::RclcppTestFixture {};

TEST_F(MagnetometerBiasRemoverNodelet, Basic)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  auto node = createNodelet();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Field> last_field;
  auto mag_cb =
    [&last_field](const Field::ConstSharedPtr& msg) {
      last_field = *msg;
    };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  std::list<rclcpp::PublisherBase::SharedPtr> pubs;
  auto mag_pub = node->create_publisher<Field>("imu/mag", rclcpp::SystemDefaultsQoS(pub_qos));
  pubs.push_back(mag_pub);
  auto mag_bias_pub = node->create_publisher<Field>(
    "imu/mag_bias", rclcpp::SystemDefaultsQoS(pub_qos).transient_local());
  pubs.push_back(mag_bias_pub);

  std::list<rclcpp::SubscriptionBase::SharedPtr> subs;

  auto mag_unbiased_sub = node->create_subscription<Field>("imu/mag_unbiased", rclcpp::SensorDataQoS(sub_qos), mag_cb);
  subs.push_back(mag_unbiased_sub);

  const auto pub_test = [](const rclcpp::PublisherBase::SharedPtr p) {return p->get_subscription_count() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for publisher connections.");
  }

  const auto sub_test = [](const rclcpp::SubscriptionBase::SharedPtr p) {return p->get_publisher_count() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), sub_test); ++i) {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pub_test));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), sub_test));

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
  mag_pub->publish(mag);

  for (size_t i = 0; i < 5 && !last_field.has_value() && rclcpp::ok(); ++i) {
    executor.spin_all(100ms);
  }

  // Missing bias, nothing published
  ASSERT_FALSE(last_field.has_value());

  // Publish bias. Now it should have everything.
  Field bias;
  bias.header.stamp = time;
  bias.header.frame_id = "imu";
  bias.magnetic_field.x = -0.097227663;
  bias.magnetic_field.y = -0.692264333;
  bias.magnetic_field.z = 0;
  mag_bias_pub->publish(bias);

  executor.spin_once();

  // Wait until the latched messages are received
  executor.spin_all(200ms);

  mag_pub->publish(mag);

  for (size_t i = 0; i < 10 && !last_field.has_value() && rclcpp::ok(); ++i) {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(last_field.has_value());

  EXPECT_EQ(time, last_field->header.stamp);
  EXPECT_EQ("imu", last_field->header.frame_id);
  EXPECT_NEAR(0.360320, last_field->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, last_field->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, last_field->magnetic_field.z, 1e-6);

  // New data
  last_field.reset();
  time.sec = 1664286802;
  time.nanosec = 197458028;

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200;
  mag.magnetic_field.y = -0.533960;
  mag.magnetic_field.z = 0.149800;
  mag_pub->publish(mag);

  for (size_t i = 0; i < 10 && !last_field.has_value() && rclcpp::ok(); ++i) {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(last_field.has_value());

  EXPECT_EQ(time, last_field->header.stamp);
  EXPECT_EQ("imu", last_field->header.frame_id);
  EXPECT_NEAR(0.361427, last_field->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, last_field->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, last_field->magnetic_field.z, 1e-6);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
