// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer bias removing message filter.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <magnetometer_pipeline/message_filter.hpp>
#include <message_filters/message_event.hpp>
#include <message_filters/simple_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

using Field = sensor_msgs::msg::MagneticField;

template<class T>
class TestInput : public message_filters::SimpleFilter<T> {
public:
  void add(const typename T::ConstSharedPtr& msg) {
    // Pass a complete MessageEvent to avoid calling node->now() to determine the missing timestamp
    this->signalMessage(message_filters::MessageEvent<T const>(msg, msg->header.stamp));
  }

  void subscribe() {
  }
};

TEST(MessageFilter, Basic)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  TestInput<Field> mag_input;
  TestInput<Field> mag_bias_input;
  magnetometer_pipeline::BiasRemoverFilter filter(node, mag_input, mag_bias_input);

  Field::ConstSharedPtr out_message;
  const auto cb =
    [&out_message](const message_filters::MessageEvent<Field const>& filtered_message) {
      out_message = filtered_message.getConstMessage();
    };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Field const>&)>(cb));

  builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  Field::SharedPtr mag(new Field);
  mag->header.stamp = time;
  mag->header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.263093;
  mag->magnetic_field.y = -0.538677;
  mag->magnetic_field.z = 0.157033;

  Field::SharedPtr bias(new Field);
  bias->header.stamp = time;
  bias->header.frame_id = "imu";
  bias->magnetic_field.x = -0.097227663;
  bias->magnetic_field.y = -0.692264333;
  bias->magnetic_field.z = 0;

  out_message.reset();
  mag_input.add(mag);

  EXPECT_EQ(nullptr, out_message);

  out_message.reset();
  mag_bias_input.add(bias);
  EXPECT_EQ(nullptr, out_message);

  out_message.reset();
  mag_input.add(mag);
  ASSERT_NE(nullptr, out_message);
  EXPECT_EQ(time, out_message->header.stamp);
  EXPECT_EQ("imu", out_message->header.frame_id);
  EXPECT_NEAR(0.360320, out_message->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, out_message->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, out_message->magnetic_field.z, 1e-6);

  // New data

  time.sec = 1664286802;
  time.nanosec = 197458028;

  mag->header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.264200;
  mag->magnetic_field.y = -0.533960;
  mag->magnetic_field.z = 0.149800;

  out_message.reset();
  mag_input.add(mag);
  ASSERT_NE(nullptr, out_message);
  EXPECT_EQ(time, out_message->header.stamp);
  EXPECT_EQ("imu", out_message->header.frame_id);
  EXPECT_NEAR(0.361427, out_message->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, out_message->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, out_message->magnetic_field.z, 1e-6);
}

TEST(MessageFilter, ConfigFromParams)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  TestInput<Field> mag_input;
  TestInput<Field> mag_bias_input;
  magnetometer_pipeline::BiasRemoverFilter filter(node, mag_input, mag_bias_input);
  Field::ConstSharedPtr out_message;
  const auto cb =
    [&out_message](const message_filters::MessageEvent<Field const>& filteredMessage) {
      out_message = filteredMessage.getConstMessage();
    };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Field const>&)>(cb));

  node.declare_parameter("initial_mag_bias_x", -0.097227663);
  rclcpp::Parameter param1("initial_mag_bias_x", -0.097227663);
  node.set_parameter(param1);
  node.declare_parameter("initial_mag_bias_y", -0.692264333);
  rclcpp::Parameter param2("initial_mag_bias_y", -0.692264333);
  node.set_parameter(param2);
  node.declare_parameter("initial_mag_bias_z", 0.0);
  rclcpp::Parameter param3("initial_mag_bias_z", 0.0);  // Using 0.0 for double type
  node.set_parameter(param3);

  filter.configFromParams();

  builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  Field::SharedPtr mag(new Field);
  mag->header.stamp = time;
  mag->header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.263093;
  mag->magnetic_field.y = -0.538677;
  mag->magnetic_field.z = 0.157033;

  out_message.reset();
  mag_input.add(mag);
  ASSERT_NE(nullptr, out_message);
  EXPECT_EQ(time, out_message->header.stamp);
  EXPECT_EQ("imu", out_message->header.frame_id);
  EXPECT_NEAR(0.360320, out_message->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, out_message->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, out_message->magnetic_field.z, 1e-6);

  // New data

  time.sec = 1664286802;
  time.nanosec = 197458028;

  mag->header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.264200;
  mag->magnetic_field.y = -0.533960;
  mag->magnetic_field.z = 0.149800;

  out_message.reset();
  mag_input.add(mag);
  ASSERT_NE(nullptr, out_message);
  EXPECT_EQ(time, out_message->header.stamp);
  EXPECT_EQ("imu", out_message->header.frame_id);
  EXPECT_NEAR(0.361427, out_message->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, out_message->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, out_message->magnetic_field.z, 1e-6);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}
