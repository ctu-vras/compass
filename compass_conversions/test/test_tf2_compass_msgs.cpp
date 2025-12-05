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
  Az inMessage;
  inMessage.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage.header.frame_id = "test";

  EXPECT_EQ(inMessage.header.frame_id, tf2::getFrameId(inMessage));
  EXPECT_NEAR(inMessage.header.stamp.sec + inMessage.header.stamp.nanosec * 1e-9,
    tf2::timeToSec(tf2::getTimestamp(inMessage)), 1e-9);
  EXPECT_EQ(inMessage, tf2::toMsg(inMessage));

  Az msg2;
  tf2::fromMsg(inMessage, msg2);
  EXPECT_EQ(inMessage, msg2);
}

TEST(TF2CompassMsgs, TransformRadNed)  // NOLINT
{
  Az inMessage;
  inMessage.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage.header.frame_id = "test";
  inMessage.unit = Az::UNIT_RAD;
  inMessage.orientation = Az::ORIENTATION_NED;
  inMessage.azimuth = M_PI_2;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = inMessage.header.stamp;
  transform.header.frame_id = inMessage.header.frame_id;
  transform.child_frame_id = "test2";
  // Translation has no effect
  transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = 1;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);

  Az outMessage;
  for (const auto ref : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM})
  {
    SCOPED_TRACE(ref);
    inMessage.reference = ref;
    tf2::doTransform(inMessage, outMessage, transform);

    EXPECT_NEAR(M_PI_4, outMessage.azimuth, 1e-6);
    EXPECT_EQ(Az::UNIT_RAD, outMessage.unit);
    EXPECT_EQ(Az::ORIENTATION_NED, outMessage.orientation);
    EXPECT_EQ(ref, outMessage.reference);
  }
}

TEST(TF2CompassMsgs, TransformRadEnu)  // NOLINT
{
  Az inMessage;
  inMessage.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage.header.frame_id = "test";
  inMessage.unit = Az::UNIT_RAD;
  inMessage.orientation = Az::ORIENTATION_ENU;
  inMessage.azimuth = M_PI_2;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = inMessage.header.stamp;
  transform.header.frame_id = inMessage.header.frame_id;
  transform.child_frame_id = "test2";
  // Translation has no effect
  transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = 1;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);

  Az outMessage;
  for (const auto ref : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM})
  {
    SCOPED_TRACE(ref);
    inMessage.reference = ref;
    tf2::doTransform(inMessage, outMessage, transform);

    EXPECT_NEAR(3 * M_PI_4, outMessage.azimuth, 1e-6);
    EXPECT_EQ(Az::UNIT_RAD, outMessage.unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, outMessage.orientation);
    EXPECT_EQ(ref, outMessage.reference);
  }
}

TEST(TF2CompassMsgs, TransformDegNed)  // NOLINT
{
  Az inMessage;
  inMessage.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage.header.frame_id = "test";
  inMessage.unit = Az::UNIT_DEG;
  inMessage.orientation = Az::ORIENTATION_NED;
  inMessage.azimuth = 90;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = inMessage.header.stamp;
  transform.header.frame_id = inMessage.header.frame_id;
  transform.child_frame_id = "test2";
  // Translation has no effect
  transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = 1;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);

  Az outMessage;
  for (const auto ref : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM})
  {
    SCOPED_TRACE(ref);
    inMessage.reference = ref;
    tf2::doTransform(inMessage, outMessage, transform);

    EXPECT_NEAR(45, outMessage.azimuth, 1e-6);
    EXPECT_EQ(Az::UNIT_DEG, outMessage.unit);
    EXPECT_EQ(Az::ORIENTATION_NED, outMessage.orientation);
    EXPECT_EQ(ref, outMessage.reference);
  }
}

TEST(TF2CompassMsgs, TransformDegEnu)  // NOLINT
{
  Az inMessage;
  inMessage.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage.header.frame_id = "test";
  inMessage.unit = Az::UNIT_DEG;
  inMessage.orientation = Az::ORIENTATION_ENU;
  inMessage.azimuth = 90;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = inMessage.header.stamp;
  transform.header.frame_id = inMessage.header.frame_id;
  transform.child_frame_id = "test2";
  // Translation has no effect
  transform.transform.translation.x = transform.transform.translation.y = transform.transform.translation.z = 1;
  transform.transform.rotation.z = std::sin(M_PI_4 / 2);
  transform.transform.rotation.w = std::cos(M_PI_4 / 2);

  Az outMessage;
  for (const auto ref : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM})
  {
    SCOPED_TRACE(ref);
    inMessage.reference = ref;
    tf2::doTransform(inMessage, outMessage, transform);

    EXPECT_NEAR(3 * 45, outMessage.azimuth, 1e-6);
    EXPECT_EQ(Az::UNIT_DEG, outMessage.unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, outMessage.orientation);
    EXPECT_EQ(ref, outMessage.reference);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
