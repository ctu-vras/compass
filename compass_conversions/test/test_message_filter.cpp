// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for compass message filter.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <angles/angles.h>
#include <compass_conversions/message_filter.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <message_filters/message_event.hpp>
#include <message_filters/simple_filter.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using Az = compass_interfaces::msg::Azimuth;

template<class T>
class TestInput : public message_filters::SimpleFilter<T> {
public:
  void add(const typename T::ConstSharedPtr& msg) {
    // Pass a complete MessageEvent to avoid calling node->now() to determine the missing timestamp
    this->signalMessage(message_filters::MessageEvent<T const>(msg, msg->header.stamp));
  }
};

TEST(MessageFilter, NoNavSatNeeded) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");

  TestInput<Az> azimuth_input;
  compass_conversions::CompassFilter filter(
      node, nullptr, azimuth_input, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstSharedPtr out_message;
  const auto cb =
    [&out_message](const message_filters::MessageEvent<Az const>& filteredMessage) {
      out_message = filteredMessage.getConstMessage();
    };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::SharedPtr in_message(new Az);
  in_message->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message->unit = Az::UNIT_DEG;
  in_message->orientation = Az::ORIENTATION_NED;
  in_message->reference = Az::REFERENCE_GEOGRAPHIC;
  in_message->azimuth = 90;

  out_message.reset();
  azimuth_input.add(in_message);

  ASSERT_NE(nullptr, out_message);
  EXPECT_NEAR(0, out_message->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, out_message->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, out_message->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, out_message->reference);
}

TEST(MessageFilter, NavSatNeededButNotGiven) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  TestInput<Az> azimuth_input;
  TestInput<sensor_msgs::msg::NavSatFix> fix_input;
  compass_conversions::CompassFilter filter(
      node, nullptr, azimuth_input, fix_input, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstSharedPtr out_message;
  const auto cb =
    [&out_message](const message_filters::MessageEvent<Az const>& filteredMessage) {
      out_message = filteredMessage.getConstMessage();
    };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::SharedPtr in_message(new Az);
  in_message->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message->unit = Az::UNIT_DEG;
  in_message->orientation = Az::ORIENTATION_NED;
  in_message->reference = Az::REFERENCE_MAGNETIC;
  in_message->azimuth = 90;

  out_message.reset();
  azimuth_input.add(in_message);

  ASSERT_EQ(nullptr, out_message);
}

TEST(MessageFilter, NavSatNeeded) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  TestInput<Az> azimuth_input;
  TestInput<sensor_msgs::msg::NavSatFix> fix_input;
  compass_conversions::CompassFilter filter(
      node, nullptr, azimuth_input, fix_input, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstSharedPtr out_message;
  const auto cb =
    [&out_message](const message_filters::MessageEvent<Az const>& filteredMessage) {
      out_message = filteredMessage.getConstMessage();
    };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::SharedPtr in_message(new Az);
  in_message->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message->unit = Az::UNIT_DEG;
  in_message->orientation = Az::ORIENTATION_NED;
  in_message->reference = Az::REFERENCE_MAGNETIC;
  in_message->azimuth = 90;

  out_message.reset();
  azimuth_input.add(in_message);

  ASSERT_EQ(nullptr, out_message);

  sensor_msgs::msg::NavSatFix::SharedPtr fix_message(new sensor_msgs::msg::NavSatFix);
  fix_message->header.stamp = in_message->header.stamp;
  fix_message->latitude = 51;
  fix_message->longitude = 10;
  fix_message->altitude = 200;
  fix_input.add(fix_message);

  ASSERT_EQ(nullptr, out_message);

  azimuth_input.add(in_message);

  ASSERT_NE(nullptr, out_message);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, out_message->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, out_message->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, out_message->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, out_message->reference);

  azimuth_input.add(in_message);

  ASSERT_NE(nullptr, out_message);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, out_message->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, out_message->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, out_message->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, out_message->reference);
}

TEST(MessageFilter, NavSatNeededAndGivenAsInitValue) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  TestInput<Az> azimuth_input;
  auto converter = std::make_shared<compass_conversions::CompassConverter>(node, true);
  compass_conversions::CompassFilter filter(
      node, converter, azimuth_input, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstSharedPtr out_message;
  const auto cb =
    [&out_message](const message_filters::MessageEvent<Az const>& filteredMessage) {
      out_message = filteredMessage.getConstMessage();
    };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::SharedPtr in_message(new Az);
  in_message->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message->unit = Az::UNIT_DEG;
  in_message->orientation = Az::ORIENTATION_NED;
  in_message->reference = Az::REFERENCE_MAGNETIC;
  in_message->azimuth = 90;

  out_message.reset();
  azimuth_input.add(in_message);

  ASSERT_EQ(nullptr, out_message);

  sensor_msgs::msg::NavSatFix::SharedPtr fix_message(new sensor_msgs::msg::NavSatFix);
  fix_message->header.stamp = in_message->header.stamp;
  fix_message->latitude = 51;
  fix_message->longitude = 10;
  fix_message->altitude = 200;
  converter->setNavSatPos(*fix_message);

  ASSERT_EQ(nullptr, out_message);

  azimuth_input.add(in_message);

  ASSERT_NE(nullptr, out_message);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, out_message->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, out_message->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, out_message->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, out_message->reference);

  azimuth_input.add(in_message);

  ASSERT_NE(nullptr, out_message);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, out_message->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, out_message->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, out_message->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, out_message->reference);
}

TEST(MessageFilter, ForcedDeclination) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  TestInput<Az> azimuth_input;
  auto converter = std::make_shared<compass_conversions::CompassConverter>(node, true);
  compass_conversions::CompassFilter filter(
      node, converter, azimuth_input, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstSharedPtr out_message;
  const auto cb =
    [&out_message](const message_filters::MessageEvent<Az const>& filteredMessage) {
      out_message = filteredMessage.getConstMessage();
    };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::SharedPtr in_message(new Az);
  in_message->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message->unit = Az::UNIT_DEG;
  in_message->orientation = Az::ORIENTATION_NED;
  in_message->reference = Az::REFERENCE_MAGNETIC;
  in_message->azimuth = 90;

  out_message.reset();
  azimuth_input.add(in_message);

  ASSERT_EQ(nullptr, out_message);

  converter->forceMagneticDeclination(angles::from_degrees(4.04));

  ASSERT_EQ(nullptr, out_message);

  azimuth_input.add(in_message);

  ASSERT_NE(nullptr, out_message);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, out_message->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, out_message->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, out_message->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, out_message->reference);

  azimuth_input.add(in_message);

  ASSERT_NE(nullptr, out_message);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, out_message->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, out_message->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, out_message->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, out_message->reference);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}
