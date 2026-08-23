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

TEST(CompassConverter, Construct) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  ASSERT_NO_THROW(compass_conversions::CompassConverter converter(node, true));
  ASSERT_NO_THROW(compass_conversions::CompassConverter converter(node, false));
}

TEST(CompassConverter, ConfigFromParams) {  // NOLINT
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

TEST(CompassConverter, ComputeMagneticDeclination) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  auto time = cras::parseTime("2024-11-18T13:00:00Z");
  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  fix.altitude = 200.0;
  auto maybe_declination = converter.computeMagneticDeclination(fix, time);
  if (!maybe_declination.has_value()) {
    RCLCPP_ERROR(node.get_logger(), "%s", maybe_declination.error().c_str());
  }
  ASSERT_TRUE(maybe_declination.has_value());
  EXPECT_NEAR(5.333, angles::to_degrees(*maybe_declination), 1e-3);

  time = cras::parseTime("2019-11-18T13:00:00Z");
  maybe_declination = converter.computeMagneticDeclination(fix, time);
  if (!maybe_declination.has_value()) {
    RCLCPP_ERROR(node.get_logger(), "%s", maybe_declination.error().c_str());
  }
  ASSERT_TRUE(maybe_declination.has_value());
  EXPECT_NEAR(4.507, angles::to_degrees(*maybe_declination), 1e-3);

  // No magnetic model for 2031
  time = cras::parseTime("2031-11-18T13:00:00Z");
  maybe_declination = converter.computeMagneticDeclination(fix, time);
  EXPECT_FALSE(maybe_declination.has_value());

  // Magnetic model for wall time is used.
  converter.setUseWallTimeForDeclination(true);
  time = cras::parseTime("2000-11-18T13:00:00Z");
  maybe_declination = converter.computeMagneticDeclination(fix, time);
  EXPECT_TRUE(maybe_declination.has_value());
}

TEST(CompassConverter, GetMagneticDeclination) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  auto time = cras::parseTime("2024-11-18T13:00:00Z");

  auto maybe_declination = converter.getMagneticDeclination(time);
  EXPECT_FALSE(maybe_declination.has_value());

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  fix.altitude = 200.0;
  converter.setNavSatPos(fix);

  maybe_declination = converter.getMagneticDeclination(time);
  if (!maybe_declination.has_value()) {
    RCLCPP_ERROR(node.get_logger(), "%s", maybe_declination.error().c_str());
  }
  ASSERT_TRUE(maybe_declination.has_value());
  EXPECT_NEAR(5.333, angles::to_degrees(*maybe_declination), 1e-3);

  time = cras::parseTime("2019-11-18T13:00:00Z");
  maybe_declination = converter.getMagneticDeclination(time);
  if (!maybe_declination.has_value()) {
    RCLCPP_ERROR(node.get_logger(), "%s", maybe_declination.error().c_str());
  }
  ASSERT_TRUE(maybe_declination.has_value());
  EXPECT_NEAR(4.507, angles::to_degrees(*maybe_declination), 1e-3);

  // No magnetic model for 2031
  time = cras::parseTime("2031-11-18T13:00:00Z");
  maybe_declination = converter.getMagneticDeclination(time);
  EXPECT_FALSE(maybe_declination.has_value());

  // Magnetic model for wall time is used.
  converter.setUseWallTimeForDeclination(true);
  time = cras::parseTime("2000-11-18T13:00:00Z");
  maybe_declination = converter.getMagneticDeclination(time);
  EXPECT_TRUE(maybe_declination.has_value());
}

TEST(CompassConverter, ComputeUTMGridConvergence) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  auto maybe_convergence_and_zone = converter.computeUTMGridConvergenceAndZone(fix, std::nullopt);
  ASSERT_TRUE(maybe_convergence_and_zone.has_value());
  EXPECT_NEAR(0, angles::to_degrees(maybe_convergence_and_zone->first), 1e-3);
  EXPECT_EQ(33, maybe_convergence_and_zone->second);

  fix.latitude = 51.0;
  fix.longitude = 10.0;
  maybe_convergence_and_zone = converter.computeUTMGridConvergenceAndZone(fix, std::nullopt);
  ASSERT_TRUE(maybe_convergence_and_zone.has_value());
  EXPECT_NEAR(0.777177, angles::to_degrees(maybe_convergence_and_zone->first), 1e-5);
  EXPECT_EQ(32, maybe_convergence_and_zone->second);

  fix.latitude = -51.0;
  fix.longitude = 10.0;
  maybe_convergence_and_zone = converter.computeUTMGridConvergenceAndZone(fix, std::nullopt);
  ASSERT_TRUE(maybe_convergence_and_zone.has_value());
  EXPECT_NEAR(-0.777177, angles::to_degrees(maybe_convergence_and_zone->first), 1e-5);
  EXPECT_EQ(32, maybe_convergence_and_zone->second);

  // Force the neighbor zone (this should be zone 32).
  fix.latitude = 51.0;
  fix.longitude = 10.0;
  maybe_convergence_and_zone = converter.computeUTMGridConvergenceAndZone(fix, 33);
  ASSERT_TRUE(maybe_convergence_and_zone.has_value());
  EXPECT_NEAR(-3.8896687, angles::to_degrees(maybe_convergence_and_zone->first), 1e-5);
  EXPECT_EQ(33, maybe_convergence_and_zone->second);
}

TEST(CompassConverter, GetUTMGridConvergence) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);
  converter.setKeepUTMZone(false);

  auto maybe_convergence = converter.getUTMGridConvergence();
  auto maybe_zone = converter.getUTMZone();
  EXPECT_FALSE(maybe_convergence.has_value());
  EXPECT_FALSE(maybe_zone.has_value());

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  converter.setNavSatPos(fix);
  maybe_convergence = converter.getUTMGridConvergence();
  maybe_zone = converter.getUTMZone();
  ASSERT_TRUE(maybe_convergence.has_value());
  EXPECT_NEAR(0, angles::to_degrees(*maybe_convergence), 1e-3);
  EXPECT_EQ(33, *maybe_zone);

  fix.latitude = 51.0;
  fix.longitude = 10.0;
  converter.setNavSatPos(fix);
  maybe_convergence = converter.getUTMGridConvergence();
  maybe_zone = converter.getUTMZone();
  ASSERT_TRUE(maybe_convergence.has_value());
  EXPECT_NEAR(0.777177, angles::to_degrees(*maybe_convergence), 1e-5);
  EXPECT_EQ(32, *maybe_zone);

  fix.latitude = -51.0;
  fix.longitude = 10.0;
  converter.setNavSatPos(fix);
  maybe_convergence = converter.getUTMGridConvergence();
  maybe_zone = converter.getUTMZone();
  ASSERT_TRUE(maybe_convergence.has_value());
  EXPECT_NEAR(-0.777177, angles::to_degrees(*maybe_convergence), 1e-5);
  EXPECT_EQ(32, *maybe_zone);

  // Force the neighbor zone (this should be zone 32).
  fix.latitude = 51.0;
  fix.longitude = 10.0;
  converter.forceUTMZone(33);
  converter.setNavSatPos(fix);
  maybe_convergence = converter.getUTMGridConvergence();
  maybe_zone = converter.getUTMZone();
  ASSERT_TRUE(maybe_convergence.has_value());
  EXPECT_NEAR(-3.8896687, angles::to_degrees(*maybe_convergence), 1e-5);
  EXPECT_EQ(33, *maybe_zone);
}

TEST(CompassConverter, ConvertNotRequiresNavSat) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  for (const auto reference : std::list {Az::REFERENCE_MAGNETIC, Az::REFERENCE_GEOGRAPHIC, Az::REFERENCE_UTM}) {
    SCOPED_TRACE(reference);
    azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = reference;
    azimuth.azimuth = M_PI_2;
    auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, reference);
    ASSERT_TRUE(maybe_azimuth.has_value());
    EXPECT_EQ(azimuth.header, maybe_azimuth->header);
    EXPECT_EQ(M_PI_2, maybe_azimuth->azimuth);
    EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
    EXPECT_EQ(reference, maybe_azimuth->reference);

    azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = reference;
    azimuth.azimuth = M_PI_2;
    maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, reference);
    ASSERT_TRUE(maybe_azimuth.has_value());
    EXPECT_EQ(azimuth.header, maybe_azimuth->header);
    EXPECT_NEAR(90, maybe_azimuth->azimuth, 1e-9);
    EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
    EXPECT_EQ(reference, maybe_azimuth->reference);

    azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = reference;
    azimuth.azimuth = 90;
    maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, reference);
    ASSERT_TRUE(maybe_azimuth.has_value());
    EXPECT_EQ(azimuth.header, maybe_azimuth->header);
    EXPECT_NEAR(M_PI_2, maybe_azimuth->azimuth, 1e-9);
    EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
    EXPECT_EQ(reference, maybe_azimuth->reference);

    azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = reference;
    azimuth.azimuth = M_PI_2;
    maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, reference);
    ASSERT_TRUE(maybe_azimuth.has_value());
    EXPECT_EQ(azimuth.header, maybe_azimuth->header);
    EXPECT_NEAR(0, maybe_azimuth->azimuth, 1e-9);
    EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
    EXPECT_EQ(reference, maybe_azimuth->reference);

    azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = reference;
    azimuth.azimuth = 90;
    maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, reference);
    ASSERT_TRUE(maybe_azimuth.has_value());
    EXPECT_EQ(azimuth.header, maybe_azimuth->header);
    EXPECT_NEAR(0, maybe_azimuth->azimuth, 1e-9);
    EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
    EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
    EXPECT_EQ(reference, maybe_azimuth->reference);
  }
}

TEST(CompassConverter, ConvertNavSatMissing) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;
  azimuth.azimuth = M_PI_2;

  for (const auto fromUnit : std::list {Az::UNIT_DEG, Az::UNIT_RAD}) {
    SCOPED_TRACE(fromUnit);
    for (const auto toUnit : std::list {Az::UNIT_DEG, Az::UNIT_RAD}) {
      SCOPED_TRACE(toUnit);
      for (const auto fromOrientation : std::list {Az::ORIENTATION_ENU, Az::ORIENTATION_NED}) {
        SCOPED_TRACE(fromOrientation);
        for (const auto toOrientation : std::list {Az::ORIENTATION_ENU, Az::ORIENTATION_NED}) {
          SCOPED_TRACE(toOrientation);

          azimuth.unit = fromUnit; azimuth.orientation = fromOrientation;

          azimuth.reference = Az::REFERENCE_MAGNETIC;
          auto maybe_azimuth = converter.convertAzimuth(azimuth, toUnit, toOrientation, Az::REFERENCE_GEOGRAPHIC);
          EXPECT_FALSE(maybe_azimuth.has_value());

          azimuth.reference = Az::REFERENCE_UTM;
          maybe_azimuth = converter.convertAzimuth(azimuth, toUnit, toOrientation, Az::REFERENCE_GEOGRAPHIC);
          EXPECT_FALSE(maybe_azimuth.has_value());

          azimuth.reference = Az::REFERENCE_MAGNETIC;
          maybe_azimuth = converter.convertAzimuth(azimuth, toUnit, toOrientation, Az::REFERENCE_UTM);
          EXPECT_FALSE(maybe_azimuth.has_value());

          azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
          maybe_azimuth = converter.convertAzimuth(azimuth, toUnit, toOrientation, Az::REFERENCE_UTM);
          EXPECT_FALSE(maybe_azimuth.has_value());
        }
      }
    }
  }
}

TEST(CompassConverter, ConvertRequiresNavSatFromMag) {  // NOLINT
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

  double declination_deg = 4.04;
  double declination_rad = angles::from_degrees(declination_deg);
  double convergence_deg = 0.777177;
  double convergence_rad = angles::from_degrees(convergence_deg);

  converter.setNavSatPos(fix);

  //
  // From: MAG, RAD, ENU
  //

  // To: MAG->GEO, RAD, ENU
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_MAGNETIC;
  azimuth.azimuth = M_PI_2;
  auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - declination_rad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, RAD->DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - declination_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declination_rad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, RAD->DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - declination_deg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->UTM, RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - declination_rad + convergence_rad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, RAD->DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - declination_deg + convergence_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declination_rad + convergence_rad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, RAD->DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - declination_deg + convergence_deg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  //
  // From: MAG, RAD, NED
  //

  // To: MAG->GEO, RAD, NED
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_MAGNETIC;
  azimuth.azimuth = M_PI_2;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declination_rad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, RAD->DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + declination_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declination_rad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, RAD->DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + declination_deg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->UTM, RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declination_rad - convergence_rad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, RAD->DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + declination_deg - convergence_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declination_rad - convergence_rad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, RAD->DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + declination_deg - convergence_deg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  //
  // From: MAG, DEG, ENU
  //

  // To: MAG->GEO, DEG, ENU
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_MAGNETIC;
  azimuth.azimuth = 90;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - declination_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, DEG->RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - declination_rad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - declination_deg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, DEG->RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declination_rad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->UTM, DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - declination_deg + convergence_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, DEG->RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - declination_rad + convergence_rad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - declination_deg + convergence_deg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, DEG->RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declination_rad + convergence_rad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  //
  // From: MAG, DEG, NED
  //

  // To: MAG->GEO, DEG, NED
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_MAGNETIC;
  azimuth.azimuth = 90;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + declination_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, DEG->RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declination_rad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + declination_deg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->GEO, DEG->RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declination_rad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: MAG->UTM, DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + declination_deg - convergence_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, DEG->RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declination_rad - convergence_rad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + declination_deg - convergence_deg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: MAG->UTM, DEG->RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declination_rad - convergence_rad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);
}

TEST(CompassConverter, ConvertRequiresNavSatFromGeo) {  // NOLINT
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

  double declination_deg = 4.04;
  double declinationRad = angles::from_degrees(declination_deg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  converter.setNavSatPos(fix);

  //
  // From: GEO, RAD, ENU
  //

  // To: GEO->MAG, RAD, ENU
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
  azimuth.azimuth = M_PI_2;
  auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, RAD->DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + declination_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, RAD->DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + declination_deg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->UTM, RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, RAD->DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + convergenceRad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, RAD->DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + convergenceDeg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  //
  // From: GEO, RAD, NED
  //

  // To: GEO->MAG, RAD, NED
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
  azimuth.azimuth = M_PI_2;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, RAD->DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - declination_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, RAD->DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - declination_deg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->UTM, RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, RAD->DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - convergenceRad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, RAD->DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - convergenceDeg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  //
  // From: GEO, DEG, ENU
  //

  // To: GEO->MAG, DEG, ENU
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
  azimuth.azimuth = 90;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + declination_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, DEG->RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + declination_deg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, DEG->RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->UTM, DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, DEG->RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + convergenceDeg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, DEG->RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + convergenceRad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  //
  // From: GEO, DEG, NED
  //

  // To: GEO->MAG, DEG, NED
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_GEOGRAPHIC;
  azimuth.azimuth = 90;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - declination_deg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, DEG->RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - declination_deg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->MAG, DEG->RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: GEO->UTM, DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, DEG->RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - convergenceDeg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);

  // To: GEO->UTM, DEG->RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - convergenceRad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_UTM, maybe_azimuth->reference);
}

TEST(CompassConverter, ConvertRequiresNavSatFromUTM) {  // NOLINT
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

  double declination_deg = 4.04;
  double declinationRad = angles::from_degrees(declination_deg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  converter.setNavSatPos(fix);

  //
  // From: UTM, RAD, ENU
  //

  // To: UTM->MAG, RAD, ENU
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, RAD->DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + declination_deg - convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad - convergenceRad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, RAD->DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + declination_deg - convergenceDeg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->GEO, RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, RAD->DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - convergenceRad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, RAD->DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - convergenceDeg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  //
  // From: UTM, RAD, NED
  //

  // To: UTM->MAG, RAD, NED
  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad + convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, RAD->DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - declination_deg + convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad + convergenceRad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, RAD->DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - declination_deg + convergenceDeg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->GEO, RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, RAD->DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + convergenceRad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, RAD->DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + convergenceDeg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  //
  // From: UTM, DEG, ENU
  //

  // To: UTM->MAG, DEG, ENU
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = 90;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + declination_deg - convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, DEG->RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + declination_deg - convergenceDeg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, DEG->RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + declinationRad - convergenceRad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->GEO, DEG, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, DEG->RAD, ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, DEG, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - convergenceDeg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, DEG->RAD, ENU->NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - convergenceRad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  //
  // From: UTM, DEG, NED
  //

  // To: UTM->MAG, DEG, NED
  azimuth.unit = Az::UNIT_DEG; azimuth.orientation = Az::ORIENTATION_NED; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = 90;
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - declination_deg + convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, DEG->RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - declinationRad + convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 - declination_deg + convergenceDeg), maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->MAG, DEG->RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 - declinationRad + convergenceRad), maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);

  // To: UTM->GEO, DEG, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 + convergenceDeg, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, DEG->RAD, NED
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_NED, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + convergenceRad, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, DEG, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_DEG, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(90 - (90 + convergenceDeg) + 360, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_DEG, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);

  // To: UTM->GEO, DEG->RAD, NED->ENU
  maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 - (M_PI_2 + convergenceRad) + 2 * M_PI, maybe_azimuth->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, maybe_azimuth->reference);
}

TEST(CompassConverter, ConvertWithInitialVals) {  // NOLINT
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

  double declination_deg = 4.04;
  double declinationRad = angles::from_degrees(declination_deg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);
}

TEST(CompassConverter, ConvertWithInitialValsZeroTime) {  // NOLINT
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
  auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);
}

TEST(CompassConverter, ConvertForcedDeclination) {  // NOLINT
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

  double declination_deg = 5.0;
  double declinationRad = angles::from_degrees(declination_deg);
  double convergenceDeg = 0.777177;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);
}

TEST(CompassConverter, ConvertForcedConvergence) {  // NOLINT
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

  double declination_deg = 4.04;
  double declinationRad = angles::from_degrees(declination_deg);
  double convergenceDeg = 5.0;
  double convergenceRad = angles::from_degrees(convergenceDeg);

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declinationRad - convergenceRad, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);
}

TEST(CompassConverter, ConvertForcedBoth) {  // NOLINT
  rclcpp::Node node = rclcpp::Node("test_node");
  compass_conversions::CompassConverter converter(node, true);
  converter.forceMagneticDeclination(angles::from_degrees(5.0));
  converter.forceUTMGridConvergence(angles::from_degrees(1.0));

  const auto time = cras::parseTime("2024-11-18T13:00:00Z");
  Az azimuth;
  azimuth.header.frame_id = "test";
  azimuth.header.stamp = time;

  double declination_deg = 5.0;
  double declination_rad = angles::from_degrees(declination_deg);
  double convergence_deg = 1.0;
  double convergence_rad = angles::from_degrees(convergence_deg);

  azimuth.unit = Az::UNIT_RAD; azimuth.orientation = Az::ORIENTATION_ENU; azimuth.reference = Az::REFERENCE_UTM;
  azimuth.azimuth = M_PI_2;
  auto maybe_azimuth = converter.convertAzimuth(azimuth, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(azimuth.header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2 + declination_rad - convergence_rad, maybe_azimuth->azimuth, 1e-2);
  EXPECT_EQ(Az::UNIT_RAD, maybe_azimuth->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, maybe_azimuth->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, maybe_azimuth->reference);
}

TEST(CompassConverter, ConvertQuaternion) {  // NOLINT
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

  auto maybe_azimuth = converter.convertQuaternion(
    *maybeQuat, 0.0, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(maybeQuat->header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2, maybe_azimuth->azimuth, 1e-6);
  EXPECT_NEAR(0, maybe_azimuth->variance, 1e-6);
  EXPECT_EQ(maybe_azimuth->unit, Az::UNIT_RAD);
  EXPECT_EQ(maybe_azimuth->orientation, Az::ORIENTATION_ENU);
  EXPECT_EQ(maybe_azimuth->reference, Az::REFERENCE_MAGNETIC);

  maybe_azimuth = converter.convertQuaternion(
    maybeQuat->quaternion, maybeQuat->header, 0.0, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_MAGNETIC);
  ASSERT_TRUE(maybe_azimuth.has_value());
  EXPECT_EQ(maybeQuat->header, maybe_azimuth->header);
  EXPECT_NEAR(M_PI_2, maybe_azimuth->azimuth, 1e-6);
  EXPECT_NEAR(0, maybe_azimuth->variance, 1e-6);
  EXPECT_EQ(maybe_azimuth->unit, Az::UNIT_RAD);
  EXPECT_EQ(maybe_azimuth->orientation, Az::ORIENTATION_ENU);
  EXPECT_EQ(maybe_azimuth->reference, Az::REFERENCE_MAGNETIC);
}

TEST(CompassConverter, ConvertToPose) {  // NOLINT
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

  const auto maybe_pose = converter.convertToPose(azimuth);
  ASSERT_TRUE(maybe_pose.has_value());
  EXPECT_EQ(azimuth.header, maybe_pose->header);
  EXPECT_NEAR(0, maybe_pose->pose.pose.orientation.x, 1e-4);
  EXPECT_NEAR(0, maybe_pose->pose.pose.orientation.y, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybe_pose->pose.pose.orientation.z, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybe_pose->pose.pose.orientation.w, 1e-4);
  EXPECT_TRUE(std::isfinite(maybe_pose->pose.covariance[0 * 6 + 0]));
  EXPECT_TRUE(std::isfinite(maybe_pose->pose.covariance[1 * 6 + 1]));
  EXPECT_TRUE(std::isfinite(maybe_pose->pose.covariance[2 * 6 + 2]));
  EXPECT_NE(0.0, maybe_pose->pose.covariance[3 * 6 + 3]);
  EXPECT_NE(0.0, maybe_pose->pose.covariance[4 * 6 + 4]);
  EXPECT_NEAR(4.0, maybe_pose->pose.covariance[5 * 6 + 5], 1e-4);
}

TEST(CompassConverter, ConvertToImu) {  // NOLINT
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

  const auto maybe_imu = converter.convertToImu(azimuth);
  ASSERT_TRUE(maybe_imu.has_value());
  EXPECT_EQ(azimuth.header, maybe_imu->header);
  EXPECT_NEAR(0, maybe_imu->orientation.x, 1e-4);
  EXPECT_NEAR(0, maybe_imu->orientation.y, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybe_imu->orientation.z, 1e-4);
  EXPECT_NEAR(M_SQRT1_2, maybe_imu->orientation.w, 1e-4);
  EXPECT_NE(0.0, maybe_imu->orientation_covariance[0 * 3 + 0]);
  EXPECT_NE(0.0, maybe_imu->orientation_covariance[1 * 3 + 1]);
  EXPECT_NEAR(4.0, maybe_imu->orientation_covariance[2 * 3 + 2], 1e-4);
  EXPECT_EQ(-1.0, maybe_imu->angular_velocity_covariance[0]);
  EXPECT_EQ(-1.0, maybe_imu->linear_acceleration_covariance[0]);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}
