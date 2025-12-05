// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for CompassConverter.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <list>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <angles/angles.h>
#include <compass_conversions/compass_converter.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

using Az = compass_interfaces::msg::Azimuth;

TEST(CompassConverter, Construct)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  ASSERT_NO_THROW(compass_conversions::CompassConverter converter(node, true));
  ASSERT_NO_THROW(compass_conversions::CompassConverter converter(node, false));
}

TEST(CompassConverter, ConfigFromParams)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  compass_conversions::CompassConverter converter(node, true);

  converter.configFromParams();

  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  node.declare_parameter("magnetic_declination", 1.0, desc);
  rclcpp::Parameter parameter1("magnetic_declination", 1.0);
  node.set_parameter(parameter1);

  converter.configFromParams();

  node.declare_parameter("utm_grid_convergence", 2.0, desc);
  rclcpp::Parameter parameter2("utm_grid_convergence", 2.0);
  node.set_parameter(parameter2);
  converter.configFromParams();

  node.undeclare_parameter("magnetic_declination");
  node.undeclare_parameter("utm_grid_convergence");

  node.declare_parameter("initial_lat", 0.0);
  rclcpp::Parameter parameter3("initial_lat", 0.0);
  node.set_parameter(parameter3);

  node.declare_parameter("initial_lon", 0.0);
  rclcpp::Parameter parameter4("initial_lon", 0.0);
  node.set_parameter(parameter4);

  converter.configFromParams();

  node.declare_parameter("alt", 0.0);
  rclcpp::Parameter parameter5("alt", 0.0);
  node.set_parameter(parameter5);

  converter.configFromParams();

  node.declare_parameter("use_wall_time_for_declination", true);
  rclcpp::Parameter parameter6("use_wall_time_for_declination", true);
  node.set_parameter(parameter6);

  converter.configFromParams();
}

TEST(CompassConverter, ComputeMagneticDeclination)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  auto time = cras::parseTime("2024-11-18T13:00:00Z");
  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  fix.altitude = 200.0;
  auto maybeDeclination = converter.computeMagneticDeclination(fix, time);
  if (!maybeDeclination.has_value())
    RCLCPP_ERROR(node.get_logger(), "%s", maybeDeclination.error().c_str());
  ASSERT_TRUE(maybeDeclination.has_value());
  EXPECT_NEAR(5.333, angles::to_degrees(*maybeDeclination), 1e-3);

  time = cras::parseTime("2019-11-18T13:00:00Z");
  maybeDeclination = converter.computeMagneticDeclination(fix, time);
  if (!maybeDeclination.has_value())
    RCLCPP_ERROR(node.get_logger(), "%s", maybeDeclination.error().c_str());
  ASSERT_TRUE(maybeDeclination.has_value());
  EXPECT_NEAR(4.507, angles::to_degrees(*maybeDeclination), 1e-3);

  // No magnetic model for 2031
  time = cras::parseTime("2031-11-18T13:00:00Z");
  maybeDeclination = converter.computeMagneticDeclination(fix, time);
  EXPECT_FALSE(maybeDeclination.has_value());

  // Magnetic model for wall time is used.
  converter.setUseWallTimeForDeclination(true);
  time = cras::parseTime("2000-11-18T13:00:00Z");
  maybeDeclination = converter.computeMagneticDeclination(fix, time);
  EXPECT_TRUE(maybeDeclination.has_value());
}

TEST(CompassConverter, GetMagneticDeclination)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  auto time = cras::parseTime("2024-11-18T13:00:00Z");

  auto maybeDeclination = converter.getMagneticDeclination(time);
  EXPECT_FALSE(maybeDeclination.has_value());

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  fix.altitude = 200.0;
  converter.setNavSatPos(fix);

  maybeDeclination = converter.getMagneticDeclination(time);
  if (!maybeDeclination.has_value())
    RCLCPP_ERROR(node.get_logger(), "%s", maybeDeclination.error().c_str());
  ASSERT_TRUE(maybeDeclination.has_value());
  EXPECT_NEAR(5.333, angles::to_degrees(*maybeDeclination), 1e-3);

  time = cras::parseTime("2019-11-18T13:00:00Z");
  maybeDeclination = converter.getMagneticDeclination(time);
  if (!maybeDeclination.has_value())
    RCLCPP_ERROR(node.get_logger(), "%s", maybeDeclination.error().c_str());
  ASSERT_TRUE(maybeDeclination.has_value());
  EXPECT_NEAR(4.507, angles::to_degrees(*maybeDeclination), 1e-3);

  // No magnetic model for 2031
  time = cras::parseTime("2031-11-18T13:00:00Z");
  maybeDeclination = converter.getMagneticDeclination(time);
  EXPECT_FALSE(maybeDeclination.has_value());

  // Magnetic model for wall time is used.
  converter.setUseWallTimeForDeclination(true);
  time = cras::parseTime("2000-11-18T13:00:00Z");
  maybeDeclination = converter.getMagneticDeclination(time);
  EXPECT_TRUE(maybeDeclination.has_value());
}

TEST(CompassConverter, ComputeUTMGridConvergence)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  auto maybeConvergenceAndZone = converter.computeUTMGridConvergenceAndZone(fix, std::nullopt);
  ASSERT_TRUE(maybeConvergenceAndZone.has_value());
  EXPECT_NEAR(0, angles::to_degrees(maybeConvergenceAndZone->first), 1e-3);
  EXPECT_EQ(33, maybeConvergenceAndZone->second);

  fix.latitude = 51.0;
  fix.longitude = 10.0;
  maybeConvergenceAndZone = converter.computeUTMGridConvergenceAndZone(fix, std::nullopt);
  ASSERT_TRUE(maybeConvergenceAndZone.has_value());
  EXPECT_NEAR(0.777177, angles::to_degrees(maybeConvergenceAndZone->first), 1e-5);
  EXPECT_EQ(32, maybeConvergenceAndZone->second);

  fix.latitude = -51.0;
  fix.longitude = 10.0;
  maybeConvergenceAndZone = converter.computeUTMGridConvergenceAndZone(fix, std::nullopt);
  ASSERT_TRUE(maybeConvergenceAndZone.has_value());
  EXPECT_NEAR(-0.777177, angles::to_degrees(maybeConvergenceAndZone->first), 1e-5);
  EXPECT_EQ(32, maybeConvergenceAndZone->second);

  // Force the neighbor zone (this should be zone 32).
  fix.latitude = 51.0;
  fix.longitude = 10.0;
  maybeConvergenceAndZone = converter.computeUTMGridConvergenceAndZone(fix, 33);
  ASSERT_TRUE(maybeConvergenceAndZone.has_value());
  EXPECT_NEAR(-3.8896687, angles::to_degrees(maybeConvergenceAndZone->first), 1e-5);
  EXPECT_EQ(33, maybeConvergenceAndZone->second);
}

TEST(CompassConverter, GetUTMGridConvergence)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);
  converter.setKeepUTMZone(false);

  auto maybeConvergence = converter.getUTMGridConvergence();
  auto maybeZone = converter.getUTMZone();
  EXPECT_FALSE(maybeConvergence.has_value());
  EXPECT_FALSE(maybeZone.has_value());

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  converter.setNavSatPos(fix);
  maybeConvergence = converter.getUTMGridConvergence();
  maybeZone = converter.getUTMZone();
  ASSERT_TRUE(maybeConvergence.has_value());
  EXPECT_NEAR(0, angles::to_degrees(*maybeConvergence), 1e-3);
  EXPECT_EQ(33, *maybeZone);

  fix.latitude = 51.0;
  fix.longitude = 10.0;
  converter.setNavSatPos(fix);
  maybeConvergence = converter.getUTMGridConvergence();
  maybeZone = converter.getUTMZone();
  ASSERT_TRUE(maybeConvergence.has_value());
  EXPECT_NEAR(0.777177, angles::to_degrees(*maybeConvergence), 1e-5);
  EXPECT_EQ(32, *maybeZone);

  fix.latitude = -51.0;
  fix.longitude = 10.0;
  converter.setNavSatPos(fix);
  maybeConvergence = converter.getUTMGridConvergence();
  maybeZone = converter.getUTMZone();
  ASSERT_TRUE(maybeConvergence.has_value());
  EXPECT_NEAR(-0.777177, angles::to_degrees(*maybeConvergence), 1e-5);
  EXPECT_EQ(32, *maybeZone);

  // Force the neighbor zone (this should be zone 32).
  fix.latitude = 51.0;
  fix.longitude = 10.0;
  converter.forceUTMZone(33);
  converter.setNavSatPos(fix);
  maybeConvergence = converter.getUTMGridConvergence();
  maybeZone = converter.getUTMZone();
  ASSERT_TRUE(maybeConvergence.has_value());
  EXPECT_NEAR(-3.8896687, angles::to_degrees(*maybeConvergence), 1e-5);
  EXPECT_EQ(33, *maybeZone);
}

TEST(CompassConverter, ConvertNotRequiresNavSat)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  for (const auto reference : std::list{Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM})
  {
    SCOPED_TRACE(reference);
    azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = reference;
    azimuth.azimuth = M_PI_2;
    auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, reference);
    ASSERT_TRUE(maybeAzimuth.has_value());
    EXPECT_EQ(azimuth.header, maybeAzimuth->header);
    EXPECT_EQ(M_PI_2, maybeAzimuth->azimuth);
    EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
    EXPECT_EQ(reference, maybeAzimuth->reference);

    azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = reference;
    azimuth.azimuth = M_PI_2;
    maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, reference);
    ASSERT_TRUE(maybeAzimuth.has_value());
    EXPECT_EQ(azimuth.header, maybeAzimuth->header);
    EXPECT_NEAR(90, maybeAzimuth->azimuth, 1e-9);
    EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
    EXPECT_EQ(reference, maybeAzimuth->reference);

    azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = reference;
    azimuth.azimuth = 90;
    maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, reference);
    ASSERT_TRUE(maybeAzimuth.has_value());
    EXPECT_EQ(azimuth.header, maybeAzimuth->header);
    EXPECT_NEAR(M_PI_2, maybeAzimuth->azimuth, 1e-9);
    EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
    EXPECT_EQ(reference, maybeAzimuth->reference);

    azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = reference;
    azimuth.azimuth = M_PI_2;
    maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, reference);
    ASSERT_TRUE(maybeAzimuth.has_value());
    EXPECT_EQ(azimuth.header, maybeAzimuth->header);
    EXPECT_NEAR(0, maybeAzimuth->azimuth, 1e-9);
    EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
    EXPECT_EQ(reference, maybeAzimuth->reference);

    azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = reference;
    azimuth.azimuth = 90;
    maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, reference);
    ASSERT_TRUE(maybeAzimuth.has_value());
    EXPECT_EQ(azimuth.header, maybeAzimuth->header);
    EXPECT_NEAR(0, maybeAzimuth->azimuth, 1e-9);
    EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
    EXPECT_EQ(reference, maybeAzimuth->reference);
  }
}

TEST(CompassConverter, ConvertNavSatMissing)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;
  azimuth.azimuth = M_PI_2;

  for (const auto fromUnit : std::list{Az::UNIT_DEG, Az::UNIT_RAD})
  {
    SCOPED_TRACE(fromUnit);
    for (const auto toUnit : std::list{Az::UNIT_DEG, Az::UNIT_RAD})
    {
      SCOPED_TRACE(toUnit);
      for (const auto fromOrientation : std::list{Az::ORIENTATION_ENU, Az::ORIENTATION_NED})
      {
        SCOPED_TRACE(fromOrientation);
        for (const auto toOrientation : std::list{Az::ORIENTATION_ENU, Az::ORIENTATION_NED})
        {
          SCOPED_TRACE(toOrientation);

          azimuth.unit = fromUnit; azimuth.orientation = fromOrientation;

          azimuth.reference = Az::REFERENCE_MAGNETIC;
          auto maybeAzimuth = converter.convertAzimuth(azimuth, toUnit, toOrientation, Az::REFERENCE_GEOGRAPHIC);
          EXPECT_FALSE(maybeAzimuth.has_value());

          azimuth.reference = Az::REFERENCE_UTM;
          maybeAzimuth = converter.convertAzimuth(azimuth, toUnit, toOrientation, Az::REFERENCE_GEOGRAPHIC);
          EXPECT_FALSE(maybeAzimuth.has_value());

          azimuth.reference = Az::REFERENCE_MAGNETIC;
          maybeAzimuth = converter.convertAzimuth(azimuth, toUnit, toOrientation, Az::REFERENCE_UTM);
          EXPECT_FALSE(maybeAzimuth.has_value());

          azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
          maybeAzimuth = converter.convertAzimuth(azimuth, toUnit, toOrientation, Az::REFERENCE_UTM);
          EXPECT_FALSE(maybeAzimuth.has_value());
        }
      }
    }
  }
}

TEST(CompassConverter, ConvertRequiresNavSatFromMag)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  sensor_msgs::msg::NavSatFix fix;
  fix.header = azimuth.header;
  fix.latitude = 51;
  fix.longitude = 10;
  fix.altitude = 200;

  double declinationDeg = 4.04;
  double declinationRad = angles::from_degrees(declinationDeg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  converter.setNavSatPos(fix);

  //
  // From: MAG, RAD, ENU
  //

  // To: MAG->GEO, RAD, ENU
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_MAGNETIC;
  azimuth.azimuth = M_PI_2;
  auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, RAD->DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - declinationDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, RAD->DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - declinationDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->UTM, RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad + convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, RAD->DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - declinationDeg + convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad + convergenceRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, RAD->DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - declinationDeg + convergenceDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  //
  // From: MAG, RAD, NED
  //

  // To: MAG->GEO, RAD, NED
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_MAGNETIC;
  azimuth.azimuth = M_PI_2;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, RAD->DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + declinationDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, RAD->DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + declinationDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->UTM, RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, RAD->DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + declinationDeg - convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad - convergenceRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, RAD->DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + declinationDeg - convergenceDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);


  //
  // From: MAG, DEG, ENU
  //

  // To: MAG->GEO, DEG, ENU
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_MAGNETIC;
  azimuth.azimuth = 90;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - declinationDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, DEG->RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - declinationDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, DEG->RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->UTM, DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - declinationDeg + convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, DEG->RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad + convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - declinationDeg + convergenceDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, DEG->RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad + convergenceRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  //
  // From: MAG, DEG, NED
  //

  // To: MAG->GEO, DEG, NED
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_MAGNETIC;
  azimuth.azimuth = 90;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + declinationDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, DEG->RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + declinationDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->GEO, DEG->RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: MAG->UTM, DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + declinationDeg - convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, DEG->RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + declinationDeg - convergenceDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: MAG->UTM, DEG->RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad - convergenceRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);
}

TEST(CompassConverter, ConvertRequiresNavSatFromGeo)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  sensor_msgs::msg::NavSatFix fix;
  fix.header = azimuth.header;
  fix.latitude = 51;
  fix.longitude = 10;
  fix.altitude = 200;

  double declinationDeg = 4.04;
  double declinationRad = angles::from_degrees(declinationDeg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  converter.setNavSatPos(fix);

  //
  // From: GEO, RAD, ENU
  //

  // To: GEO->MAG, RAD, ENU
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
  azimuth.azimuth = M_PI_2;
  auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, RAD->DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + declinationDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, RAD->DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + declinationDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->UTM, RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, RAD->DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + convergenceRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, RAD->DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + convergenceDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  //
  // From: GEO, RAD, NED
  //

  // To: GEO->MAG, RAD, NED
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
  azimuth.azimuth = M_PI_2;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, RAD->DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - declinationDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, RAD->DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - declinationDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->UTM, RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, RAD->DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - convergenceRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, RAD->DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - convergenceDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);


  //
  // From: GEO, DEG, ENU
  //

  // To: GEO->MAG, DEG, ENU
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
  azimuth.azimuth = 90;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + declinationDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, DEG->RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + declinationDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, DEG->RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->UTM, DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, DEG->RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + convergenceDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, DEG->RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + convergenceRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  //
  // From: GEO, DEG, NED
  //

  // To: GEO->MAG, DEG, NED
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
  azimuth.azimuth = 90;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - declinationDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, DEG->RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - declinationDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->MAG, DEG->RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: GEO->UTM, DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, DEG->RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - convergenceDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);

  // To: GEO->UTM, DEG->RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - convergenceRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybeAzimuth->reference);
}

TEST(CompassConverter, ConvertRequiresNavSatFromUTM)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  sensor_msgs::msg::NavSatFix fix;
  fix.header = azimuth.header;
  fix.latitude = 51;
  fix.longitude = 10;
  fix.altitude = 200;

  double declinationDeg = 4.04;
  double declinationRad = angles::from_degrees(declinationDeg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  converter.setNavSatPos(fix);

  //
  // From: UTM, RAD, ENU
  //

  // To: UTM->MAG, RAD, ENU
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, RAD->DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + declinationDeg - convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad - convergenceRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, RAD->DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + declinationDeg - convergenceDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->GEO, RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, RAD->DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - convergenceRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, RAD->DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - convergenceDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  //
  // From: UTM, RAD, NED
  //

  // To: UTM->MAG, RAD, NED
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad + convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, RAD->DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - declinationDeg + convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad + convergenceRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, RAD->DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - declinationDeg + convergenceDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->GEO, RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, RAD->DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + convergenceRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, RAD->DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + convergenceDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);


  //
  // From: UTM, DEG, ENU
  //

  // To: UTM->MAG, DEG, ENU
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = 90;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + declinationDeg - convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, DEG->RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + declinationDeg - convergenceDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, DEG->RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad - convergenceRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->GEO, DEG, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, DEG->RAD, ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, DEG, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - convergenceDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, DEG->RAD, ENU->NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - convergenceRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  //
  // From: UTM, DEG, NED
  //

  // To: UTM->MAG, DEG, NED
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = 90;
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - declinationDeg + convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, DEG->RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad + convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 - declinationDeg + convergenceDeg), maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->MAG, DEG->RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad + convergenceRad), maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);

  // To: UTM->GEO, DEG, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 + convergenceDeg, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, DEG->RAD, NED
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + convergenceRad, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, DEG, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(90 - (90 + convergenceDeg) + 360, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);

  // To: UTM->GEO, DEG->RAD, NED->ENU
  maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + convergenceRad) + 2 * M_PI, maybeAzimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybeAzimuth->reference);
}

TEST(CompassConverter, ConvertWithInitialVals)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 10.0;
  fix.altitude = 200.0;

  const auto clk = rclcpp::Clock();
  compass_conversions::CompassConverter converter(node, true);
  converter.setNavSatPos(fix);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  double declinationDeg = 4.04;
  double declinationRad = angles::from_degrees(declinationDeg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);
}

TEST(CompassConverter, ConvertWithInitialValsZeroTime)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 10.0;
  fix.altitude = 200.0;

  const auto clk = rclcpp::Clock();
  compass_conversions::CompassConverter converter(node, true);
  converter.setNavSatPos(fix);
  converter.setUseWallTimeForDeclination(true);

  const auto time = cras::parseTime("1970-01-01T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);
}

TEST(CompassConverter, ConvertForcedDeclination)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 10.0;
  fix.altitude = 200.0;

  const auto clk = rclcpp::Clock();
  compass_conversions::CompassConverter converter(node, true);
  converter.forceMagneticDeclination(angles::from_degrees(5.0));
  converter.setNavSatPos(fix);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  double declinationDeg = 5.0;
  double declinationRad = angles::from_degrees(declinationDeg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);
}

TEST(CompassConverter, ConvertForcedConvergence)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 10.0;
  fix.altitude = 200.0;

  const auto clk = rclcpp::Clock();
  compass_conversions::CompassConverter converter(node, true);
  converter.forceUTMGridConvergence(angles::from_degrees(5.0));
  converter.setNavSatPos(fix);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  double declinationDeg = 4.04;
  double declinationRad = angles::from_degrees(declinationDeg);
  double convergenceDeg = 5.0;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);
}

TEST(CompassConverter, ConvertForcedBoth)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);
  converter.forceMagneticDeclination(angles::from_degrees(5.0));
  converter.forceUTMGridConvergence(angles::from_degrees(1.0));

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  double declinationDeg = 5.0;
  double declinationRad = angles::from_degrees(declinationDeg);
  double convergenceDeg = 1.0;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybeAzimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(azimuth.header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybeAzimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_RAD, maybeAzimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybeAzimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybeAzimuth->reference);
}

TEST(CompassConverter, ConvertQuaternion)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");

  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;
  azimuth.unit = Az::UNIT_RAD;
  azimuth.orientation = Az::ORIENTATION_ENU;
  azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;

  const auto maybeQuat = converter.convertToQuaternion(azimuth);
  ASSERT_TRUE(maybeQuat.has_value());
  EXPECT_EQ(azimuth.header, maybeQuat->header);
  EXPECT_NEAR(0, maybeQuat->quaternion.x, 1e-4);
  EXPECT_NEAR(0, maybeQuat->quaternion.y, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybeQuat->quaternion.z, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybeQuat->quaternion.w, 1e-4);

  auto maybeAzimuth = converter.convertQuaternion(
    *maybeQuat, 0.0, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(maybeQuat->header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2, maybeAzimuth->azimuth, 1e-6);
  EXPECT_NEAR(0, maybeAzimuth->variance, 1e-6);
  EXPECT_EQ(maybeAzimuth->unit, Az::UNIT_RAD);
  EXPECT_EQ(maybeAzimuth->orientation, Az::ORIENTATION_ENU);
  EXPECT_EQ(maybeAzimuth->reference, Az::REFERENCE_MAGNETIC);

  maybeAzimuth = converter.convertQuaternion(
    maybeQuat->quaternion, maybeQuat->header, 0.0, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybeAzimuth.has_value());
  EXPECT_EQ(maybeQuat->header, maybeAzimuth->header);
  EXPECT_NEAR(M_PI_2, maybeAzimuth->azimuth, 1e-6);
  EXPECT_NEAR(0, maybeAzimuth->variance, 1e-6);
  EXPECT_EQ(maybeAzimuth->unit, Az::UNIT_RAD);
  EXPECT_EQ(maybeAzimuth->orientation, Az::ORIENTATION_ENU);
  EXPECT_EQ(maybeAzimuth->reference, Az::REFERENCE_MAGNETIC);
}

TEST(CompassConverter, ConvertToPose)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;
  azimuth.unit = Az::UNIT_RAD;
  azimuth.orientation = Az::ORIENTATION_ENU;
  azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  azimuth.variance = 4.0;

  const auto maybePose = converter.convertToPose(azimuth);
  ASSERT_TRUE(maybePose.has_value());
  EXPECT_EQ(azimuth.header, maybePose->header);
  EXPECT_NEAR(0, maybePose->pose.pose.orientation.x, 1e-4);
  EXPECT_NEAR(0, maybePose->pose.pose.orientation.y, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybePose->pose.pose.orientation.z, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybePose->pose.pose.orientation.w, 1e-4);
  EXPECT_TRUE(std::isfinite(maybePose->pose.covariance[0 * 6 + 0]));
  EXPECT_TRUE(std::isfinite(maybePose->pose.covariance[1 * 6 + 1]));
  EXPECT_TRUE(std::isfinite(maybePose->pose.covariance[2 * 6 + 2]));
  EXPECT_NE(0.0, maybePose->pose.covariance[3 * 6 + 3]);
  EXPECT_NE(0.0, maybePose->pose.covariance[4 * 6 + 4]);
  EXPECT_NEAR(4.0, maybePose->pose.covariance[5 * 6 + 5], 1e-4);
}

TEST(CompassConverter, ConvertToImu)  // NOLINT
{
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;
  azimuth.unit = Az::UNIT_RAD;
  azimuth.orientation = Az::ORIENTATION_ENU;
  azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  azimuth.variance = 4.0;

  const auto maybeImu = converter.convertToImu(azimuth);
  ASSERT_TRUE(maybeImu.has_value());
  EXPECT_EQ(azimuth.header, maybeImu->header);
  EXPECT_NEAR(0, maybeImu->orientation.x, 1e-4);
  EXPECT_NEAR(0, maybeImu->orientation.y, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybeImu->orientation.z, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybeImu->orientation.w, 1e-4);
  EXPECT_NE(0.0, maybeImu->orientation_covariance[0 * 3 + 0]);
  EXPECT_NE(0.0, maybeImu->orientation_covariance[1 * 3 + 1]);
  EXPECT_NEAR(4.0, maybeImu->orientation_covariance[2 * 3 + 2], 1e-4);
  EXPECT_EQ(-1.0, maybeImu->angular_velocity_covariance[0]);
  EXPECT_EQ(-1.0, maybeImu->linear_acceleration_covariance[0]);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}
