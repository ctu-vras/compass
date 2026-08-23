// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Logic for naming topics according to the type of Azimuth message they carry.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <map>
#include <optional>
#include <string>
#include <tuple>

#include <compass_conversions/topic_names.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

using Az = compass_interfaces::msg::Azimuth;

namespace compass_conversions {

namespace {

std::string getAzimuthTopicSuffix(const Orientation orientation, const Reference reference) {
  const std::string ref_str =
    reference == Az::REFERENCE_MAGNETIC ? "mag" : (reference == Az::REFERENCE_GEOGRAPHIC ? "true" : "utm");
  const std::string orStr = orientation == Az::ORIENTATION_ENU ? "enu" : "ned";
  return ref_str + "/" + orStr;
}

}  // namespace

template<>
std::string getAzimuthTopicSuffix<Az>(
    const Unit unit, const Orientation orientation, const Reference reference) {
  const auto unit_str = unit == Az::UNIT_RAD ? "rad" : "deg";
  return getAzimuthTopicSuffix(orientation, reference) + "/" + unit_str;
}

template<>
std::string getAzimuthTopicSuffix<geometry_msgs::msg::QuaternionStamped>(
    const Unit unit, const Orientation orientation, const Reference reference) {
  return getAzimuthTopicSuffix(orientation, reference) + "/quat";
}

template<>
std::string getAzimuthTopicSuffix<geometry_msgs::msg::PoseWithCovarianceStamped>(
    const Unit unit, const Orientation orientation, const Reference reference) {
  return getAzimuthTopicSuffix(orientation, reference) + "/pose";
}

template<>
std::string getAzimuthTopicSuffix<sensor_msgs::msg::Imu>(
    const Unit unit, const Orientation orientation, const Reference reference) {
  return getAzimuthTopicSuffix(orientation, reference) + "/imu";
}

std::optional<std::tuple<Unit, Orientation, Reference>>
parseAzimuthTopicName(const std::string& topic) {
  const auto parts = cras::split(topic, "/");

  if (parts.size() < 3) {
    return {};
  }

  auto it = parts.rbegin();
  const auto unit_part = *it;
  ++it;
  const auto or_part = *it;
  ++it;
  const auto ref_part = *it;

  Unit unit;
  if (unit_part == "deg") {
    unit = Az::UNIT_DEG;
  } else if (unit_part == "rad" || unit_part == "imu" || unit_part == "pose" || unit_part == "quat") {
    unit = Az::UNIT_RAD;
  } else {
    return {};
  }

  Orientation orientation;
  if (or_part == "ned") {
    orientation = Az::ORIENTATION_NED;
  } else if (or_part == "enu") {
    orientation = Az::ORIENTATION_ENU;
  } else {
    return {};
  }

  Reference reference;
  if (ref_part == "mag") {
    reference = Az::REFERENCE_MAGNETIC;
  } else if (ref_part == "true") {
    reference = Az::REFERENCE_GEOGRAPHIC;
  } else if (ref_part == "utm") {
    reference = Az::REFERENCE_UTM;
  } else {
    return {};
  }

  return {{unit, orientation, reference}};
}

std::optional<std::tuple<Unit, Orientation, Reference>>
parseAzimuthTopicName(const std::shared_ptr<std::map<std::string, std::string>>& connection_header_ptr) {
  if (connection_header_ptr != nullptr && connection_header_ptr->contains("topic")) {
    return parseAzimuthTopicName(connection_header_ptr->at("topic"));
  }
  return {};
}

}  // namespace compass_conversions
