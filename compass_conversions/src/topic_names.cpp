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

namespace compass_conversions
{

namespace
{

std::string getAzimuthTopicSuffix(const Orientation orientation, const Reference reference)
{
  const std::string refStr =
    reference == Az::REFERENCE_MAGNETIC ? "mag" : (reference == Az::REFERENCE_GEOGRAPHIC ? "true" : "utm");
  const std::string orStr = orientation == Az::ORIENTATION_ENU ? "enu" : "ned";
  return refStr + "/" + orStr;
}

}

template<> std::string getAzimuthTopicSuffix<Az>(
  const Unit unit, const Orientation orientation, const Reference reference)
{
  const auto unitStr = unit == Az::UNIT_RAD ? "rad" : "deg";
  return getAzimuthTopicSuffix(orientation, reference) + "/" + unitStr;
}

template<> std::string getAzimuthTopicSuffix<geometry_msgs::msg::QuaternionStamped>(
  const Unit unit, const Orientation orientation, const Reference reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/quat";
}

template<> std::string getAzimuthTopicSuffix<geometry_msgs::msg::PoseWithCovarianceStamped>(
  const Unit unit, const Orientation orientation, const Reference reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/pose";
}

template<> std::string getAzimuthTopicSuffix<sensor_msgs::msg::Imu>(
  const Unit unit, const Orientation orientation, const Reference reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/imu";
}

std::optional<std::tuple<Unit, Orientation, Reference>>
parseAzimuthTopicName(const std::string& topic)
{
  const auto parts = cras::split(topic, "/");

  // *INDENT-OFF*
  if (parts.size() < 3)
    return {};

  auto it = parts.rbegin();
  const auto unitPart = *it;
  ++it;
  const auto orPart = *it;
  ++it;
  const auto refPart = *it;

  Unit unit;
  if (unitPart == "deg")
    unit = Az::UNIT_DEG;
  else if (unitPart == "rad" || unitPart == "imu" || unitPart == "pose" || unitPart == "quat")
    unit = Az::UNIT_RAD;
  else
    return {};

  Orientation orientation;
  if (orPart == "ned")
    orientation = Az::ORIENTATION_NED;
  else if (orPart == "enu")
    orientation = Az::ORIENTATION_ENU;
  else
    return {};

  Reference reference;
  if (refPart == "mag")
    reference = Az::REFERENCE_MAGNETIC;
  else if (refPart == "true")
    reference = Az::REFERENCE_GEOGRAPHIC;
  else if (refPart == "utm")
    reference = Az::REFERENCE_UTM;
  else
    return {};

  // *INDENT-ON*
  return {{unit, orientation, reference}};
}

std::optional<std::tuple<Unit, Orientation, Reference>>
parseAzimuthTopicName(const std::shared_ptr<std::map<std::string, std::string>>& connectionHeaderPtr)
{
  if (connectionHeaderPtr != nullptr && connectionHeaderPtr->contains("topic"))
    return parseAzimuthTopicName(connectionHeaderPtr->at("topic"));
  // *INDENT-OFF*
  return {};
  // *INDENT-ON*
}

}
