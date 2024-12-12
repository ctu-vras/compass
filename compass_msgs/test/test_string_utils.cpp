// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for testing string utilities for compass_msgs.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <string>

#include <compass_msgs/string_utils.h>

using Az = compass_msgs::Azimuth;

TEST(CompassMsgs, Unit)  // NOLINT
{
  EXPECT_EQ(Az::UNIT_RAD, compass_msgs::parseUnit("rad"));
  EXPECT_EQ(Az::UNIT_RAD, compass_msgs::parseUnit("RAD"));
  EXPECT_EQ(Az::UNIT_RAD, compass_msgs::parseUnit("Rad"));

  EXPECT_EQ(Az::UNIT_DEG, compass_msgs::parseUnit("deg"));
  EXPECT_EQ(Az::UNIT_DEG, compass_msgs::parseUnit("DEG"));
  EXPECT_EQ(Az::UNIT_DEG, compass_msgs::parseUnit("Deg"));

  EXPECT_THROW(compass_msgs::parseUnit("foo"), std::runtime_error);

  EXPECT_EQ("rad", compass_msgs::unitToString(Az::UNIT_RAD));
  EXPECT_EQ("deg", compass_msgs::unitToString(Az::UNIT_DEG));
  EXPECT_THROW(compass_msgs::unitToString(10), std::runtime_error);
}

TEST(CompassMsgs, Orientation)  // NOLINT
{
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_msgs::parseOrientation("enu"));
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_msgs::parseOrientation("ENU"));
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_msgs::parseOrientation("Enu"));

  EXPECT_EQ(Az::ORIENTATION_NED, compass_msgs::parseOrientation("ned"));
  EXPECT_EQ(Az::ORIENTATION_NED, compass_msgs::parseOrientation("NED"));
  EXPECT_EQ(Az::ORIENTATION_NED, compass_msgs::parseOrientation("Ned"));

  EXPECT_THROW(compass_msgs::parseOrientation("foo"), std::runtime_error);

  EXPECT_EQ("ENU", compass_msgs::orientationToString(Az::ORIENTATION_ENU));
  EXPECT_EQ("NED", compass_msgs::orientationToString(Az::ORIENTATION_NED));
  EXPECT_THROW(compass_msgs::orientationToString(10), std::runtime_error);
}

TEST(CompassMsgs, Reference)  // NOLINT
{
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_msgs::parseReference("magnetic"));
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_msgs::parseReference("MAGNETIC"));
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_msgs::parseReference("Magnetic"));

  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_msgs::parseReference("geographic"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_msgs::parseReference("GEOGRAPHIC"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_msgs::parseReference("Geographic"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_msgs::parseReference("true"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_msgs::parseReference("TRUE"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_msgs::parseReference("True"));

  EXPECT_EQ(Az::REFERENCE_UTM, compass_msgs::parseReference("utm"));
  EXPECT_EQ(Az::REFERENCE_UTM, compass_msgs::parseReference("UTM"));
  EXPECT_EQ(Az::REFERENCE_UTM, compass_msgs::parseReference("Utm"));

  EXPECT_THROW(compass_msgs::parseReference("foo"), std::runtime_error);

  EXPECT_EQ("magnetic", compass_msgs::referenceToString(Az::REFERENCE_MAGNETIC));
  EXPECT_EQ("geographic", compass_msgs::referenceToString(Az::REFERENCE_GEOGRAPHIC));
  EXPECT_EQ("UTM", compass_msgs::referenceToString(Az::REFERENCE_UTM));
  EXPECT_THROW(compass_msgs::referenceToString(10), std::runtime_error);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
