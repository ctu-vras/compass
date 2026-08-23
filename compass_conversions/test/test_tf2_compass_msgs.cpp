// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for transformations of compass_interfaces.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <cmath>
#include <list>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <angles/angles.h>
#include <compass_conversions/tf2_compass_msgs.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/convert.hpp>

using Az = compass_interfaces::msg::Azimuth;

TEST(TF2CompassMsgs, Tf2MessageTraits)  // NOLINT
{
  Az in_message;
  in_message.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message.header.frame_id = "test";

  EXPECT_EQ(in_message.header.frame_id, tf2::getFrameId(in_message));
  EXPECT_NEAR(in_message.header.stamp.sec + in_message.header.stamp.nanosec * 1e-9,
    tf2::timeToSec(tf2::getTimestamp(in_message)), 1e-9);
  EXPECT_EQ(in_message, tf2::toMsg(in_message));

  Az msg2;
  tf2::fromMsg(in_message, msg2);
  EXPECT_EQ(in_message, msg2);
}

TEST(TF2CompassMsgs, TransformRadNed)  // NOLINT
{
  Az in_message;
  in_message.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message.header.frame_id = "test";
  in_message.unit = Az::UNIT_RAD;
  in_message.orientation = Az::ORIENTATION_NED;
  in_message.azimuth = M_PI_2;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = in_message.header.stamp;
  transform.header.frame_id = in_message.header.frame_id;
  transform.child_frame_id = "test2";
  // Translation has no effect
  transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = 1;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);

  Az out_message;
  for (const auto ref : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM}) {
    SCOPED_TRACE(ref);
    in_message.reference = ref;
    tf2::doTransform(in_message, out_message, transform);

    EXPECT_NEAR(M_PI_4, out_message.azimuth, 1e-6);
    EXPECT_EQ(Az::UNIT_RAD, out_message.unit);
    EXPECT_EQ(Az::ORIENTATION_NED, out_message.orientation);
    EXPECT_EQ(ref, out_message.reference);
  }
}

TEST(TF2CompassMsgs, TransformRadEnu)  // NOLINT
{
  Az in_message;
  in_message.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message.header.frame_id = "test";
  in_message.unit = Az::UNIT_RAD;
  in_message.orientation = Az::ORIENTATION_ENU;
  in_message.azimuth = M_PI_2;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = in_message.header.stamp;
  transform.header.frame_id = in_message.header.frame_id;
  transform.child_frame_id = "test2";
  // Translation has no effect
  transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = 1;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);

  Az out_message;
  for (const auto ref : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM}) {
    SCOPED_TRACE(ref);
    in_message.reference = ref;
    tf2::doTransform(in_message, out_message, transform);

    EXPECT_NEAR(3 * M_PI_4, out_message.azimuth, 1e-6);
    EXPECT_EQ(Az::UNIT_RAD, out_message.unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, out_message.orientation);
    EXPECT_EQ(ref, out_message.reference);
  }
}

TEST(TF2CompassMsgs, TransformDegNed)  // NOLINT
{
  Az in_message;
  in_message.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message.header.frame_id = "test";
  in_message.unit = Az::UNIT_DEG;
  in_message.orientation = Az::ORIENTATION_NED;
  in_message.azimuth = 90;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = in_message.header.stamp;
  transform.header.frame_id = in_message.header.frame_id;
  transform.child_frame_id = "test2";
  // Translation has no effect
  transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = 1;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);

  Az out_message;
  for (const auto ref : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM}) {
    SCOPED_TRACE(ref);
    in_message.reference = ref;
    tf2::doTransform(in_message, out_message, transform);

    EXPECT_NEAR(45, out_message.azimuth, 1e-6);
    EXPECT_EQ(Az::UNIT_DEG, out_message.unit);
    EXPECT_EQ(Az::ORIENTATION_NED, out_message.orientation);
    EXPECT_EQ(ref, out_message.reference);
  }
}

TEST(TF2CompassMsgs, TransformDegEnu)  // NOLINT
{
  Az in_message;
  in_message.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  in_message.header.frame_id = "test";
  in_message.unit = Az::UNIT_DEG;
  in_message.orientation = Az::ORIENTATION_ENU;
  in_message.azimuth = 90;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = in_message.header.stamp;
  transform.header.frame_id = in_message.header.frame_id;
  transform.child_frame_id = "test2";
  // Translation has no effect
  transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = 1;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);

  Az out_message;
  for (const auto ref : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM}) {
    SCOPED_TRACE(ref);
    in_message.reference = ref;
    tf2::doTransform(in_message, out_message, transform);

    EXPECT_NEAR(3 * 45, out_message.azimuth, 1e-6);
    EXPECT_EQ(Az::UNIT_DEG, out_message.unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, out_message.orientation);
    EXPECT_EQ(ref, out_message.reference);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
