// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for testing string utilities for compass_interfaces.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <gtest/gtest.h>

#include <string>

#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_interfaces/string_utils.hpp>
#include <cras_cpp_common/string_utils.hpp>

using Az = compass_interfaces::msg::Azimuth;

TEST(CompassMsgs, Unit)  // NOLINT
{
  EXPECT_EQ(Az::UNIT_RAD, compass_interfaces::parseUnit("rad"));
  EXPECT_EQ(Az::UNIT_RAD, compass_interfaces::parseUnit("RAD"));
  EXPECT_EQ(Az::UNIT_RAD, compass_interfaces::parseUnit("Rad"));

  EXPECT_EQ(Az::UNIT_DEG, compass_interfaces::parseUnit("deg"));
  EXPECT_EQ(Az::UNIT_DEG, compass_interfaces::parseUnit("DEG"));
  EXPECT_EQ(Az::UNIT_DEG, compass_interfaces::parseUnit("Deg"));

  EXPECT_THROW(compass_interfaces::parseUnit("foo"), std::runtime_error);

  EXPECT_EQ("rad", compass_interfaces::unitToString(Az::UNIT_RAD));
  EXPECT_EQ("deg", compass_interfaces::unitToString(Az::UNIT_DEG));
  EXPECT_THROW(compass_interfaces::unitToString(10), std::runtime_error);
}

TEST(CompassMsgs, Orientation)  // NOLINT
{
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_interfaces::parseOrientation("enu"));
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_interfaces::parseOrientation("ENU"));
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_interfaces::parseOrientation("Enu"));

  EXPECT_EQ(Az::ORIENTATION_NED, compass_interfaces::parseOrientation("ned"));
  EXPECT_EQ(Az::ORIENTATION_NED, compass_interfaces::parseOrientation("NED"));
  EXPECT_EQ(Az::ORIENTATION_NED, compass_interfaces::parseOrientation("Ned"));

  EXPECT_THROW(compass_interfaces::parseOrientation("foo"), std::runtime_error);

  EXPECT_EQ("ENU", compass_interfaces::orientationToString(Az::ORIENTATION_ENU));
  EXPECT_EQ("NED", compass_interfaces::orientationToString(Az::ORIENTATION_NED));
  EXPECT_THROW(compass_interfaces::orientationToString(10), std::runtime_error);
}

TEST(CompassMsgs, Reference)  // NOLINT
{
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_interfaces::parseReference("magnetic"));
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_interfaces::parseReference("MAGNETIC"));
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_interfaces::parseReference("Magnetic"));

  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_interfaces::parseReference("geographic"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_interfaces::parseReference("GEOGRAPHIC"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_interfaces::parseReference("Geographic"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_interfaces::parseReference("true"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_interfaces::parseReference("TRUE"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_interfaces::parseReference("True"));

  EXPECT_EQ(Az::REFERENCE_UTM, compass_interfaces::parseReference("utm"));
  EXPECT_EQ(Az::REFERENCE_UTM, compass_interfaces::parseReference("UTM"));
  EXPECT_EQ(Az::REFERENCE_UTM, compass_interfaces::parseReference("Utm"));

  EXPECT_THROW(compass_interfaces::parseReference("foo"), std::runtime_error);

  EXPECT_EQ("magnetic", compass_interfaces::referenceToString(Az::REFERENCE_MAGNETIC));
  EXPECT_EQ("geographic", compass_interfaces::referenceToString(Az::REFERENCE_GEOGRAPHIC));
  EXPECT_EQ("UTM", compass_interfaces::referenceToString(Az::REFERENCE_UTM));
  EXPECT_THROW(compass_interfaces::referenceToString(10), std::runtime_error);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
