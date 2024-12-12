// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Logic for naming topics according to the type of Azimuth message they carry.
 * \author Martin Pecka
 */

#include <string>
#include <tuple>

#include <boost/shared_ptr.hpp>

#include <compass_conversions/topic_names.h>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <ros/datatypes.h>
#include <sensor_msgs/Imu.h>

using Az = compass_msgs::Azimuth;

namespace compass_conversions
{

namespace
{

std::string getAzimuthTopicSuffix(const decltype(Az::orientation) orientation, const decltype(Az::reference) reference)
{
  const std::string refStr =
    reference == Az::REFERENCE_MAGNETIC ? "mag" : (reference == Az::REFERENCE_GEOGRAPHIC ? "true" : "utm");
  const std::string orStr = orientation == Az::ORIENTATION_ENU ? "enu" : "ned";
  return refStr + "/" + orStr;
}

}

template<> std::string getAzimuthTopicSuffix<Az>(
  const decltype(Az::unit) unit,
  const decltype(Az::orientation) orientation,
  const decltype(Az::reference) reference)
{
  const auto unitStr = unit == Az::UNIT_RAD ? "rad" : "deg";
  return getAzimuthTopicSuffix(orientation, reference) + "/" + unitStr;
}

template<> std::string getAzimuthTopicSuffix<geometry_msgs::QuaternionStamped>(
  const decltype(Az::unit) unit,
  const decltype(Az::orientation) orientation,
  const decltype(Az::reference) reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/quat";
}

template<> std::string getAzimuthTopicSuffix<geometry_msgs::PoseWithCovarianceStamped>(
  const decltype(Az::unit) unit,
  const decltype(Az::orientation) orientation,
  const decltype(Az::reference) reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/pose";
}

template<> std::string getAzimuthTopicSuffix<sensor_msgs::Imu>(
  const decltype(Az::unit) unit,
  const decltype(Az::orientation) orientation,
  const decltype(Az::reference) reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/imu";
}

cras::optional<std::tuple<decltype(Az::unit), decltype(Az::orientation), decltype(Az::reference)>>
parseAzimuthTopicName(const std::string& topic)
{
  const auto parts = cras::split(topic, "/");
  if (parts.size() < 3)
    return cras::nullopt;

  auto it = parts.rbegin();
  const auto unitPart = *it;
  ++it;
  const auto orPart = *it;
  ++it;
  const auto refPart = *it;

  decltype(Az::unit) unit;
  if (unitPart == "deg")
    unit = Az::UNIT_DEG;
  else if (unitPart == "rad" || unitPart == "imu" || unitPart == "pose" || unitPart == "quat")
    unit = Az::UNIT_RAD;
  else
    return cras::nullopt;

  decltype(Az::orientation) orientation;
  if (orPart == "ned")
    orientation = Az::ORIENTATION_NED;
  else if (orPart == "enu")
    orientation = Az::ORIENTATION_ENU;
  else
    return cras::nullopt;

  decltype(Az::reference) reference;
  if (refPart == "mag")
    reference = Az::REFERENCE_MAGNETIC;
  else if (refPart == "true")
    reference = Az::REFERENCE_GEOGRAPHIC;
  else if (refPart == "utm")
    reference = Az::REFERENCE_UTM;
  else
    return cras::nullopt;

  return std::make_tuple(unit, orientation, reference);
}

cras::optional<std::tuple<decltype(Az::unit), decltype(Az::orientation), decltype(Az::reference)>>
parseAzimuthTopicName(const boost::shared_ptr<ros::M_string>& connectionHeaderPtr)
{
  if (connectionHeaderPtr != nullptr && connectionHeaderPtr->find("topic") != connectionHeaderPtr->end())
    return parseAzimuthTopicName(connectionHeaderPtr->at("topic"));
  return cras::nullopt;
}

}
