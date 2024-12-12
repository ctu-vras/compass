// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for transformations of compass_msgs.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cmath>
#include <memory>
#include <string>

#include <angles/angles.h>
#include <compass_conversions/tf2_compass_msgs.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>

using Az = compass_msgs::Azimuth;

TEST(TF2CompassMsgs, Tf2MessageTraits)  // NOLINT
{
  Az inMessage;
  inMessage.header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage.header.frame_id = "test";

  EXPECT_EQ(inMessage.header.frame_id, tf2::getFrameId(inMessage));
  EXPECT_EQ(inMessage.header.stamp, tf2::getTimestamp(inMessage));
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

  geometry_msgs::TransformStamped transform;
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

  geometry_msgs::TransformStamped transform;
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

  geometry_msgs::TransformStamped transform;
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

  geometry_msgs::TransformStamped transform;
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
