#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Logic for naming topics according to the type of Azimuth message they carry.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include <compass_interfaces/msg/azimuth.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace compass_conversions
{

using Unit = compass_interfaces::msg::Azimuth::_unit_type;
using Orientation = compass_interfaces::msg::Azimuth::_orientation_type;
using Reference = compass_interfaces::msg::Azimuth::_reference_type;

/**
 * \brief Get the suffix of topic name that identifies the particular representation of azimuth.
 *
 * \tparam T Type of the message carrying the azimuth information.
 * \param[in] unit Angular units (only make sense for Azimuth messages).
 * \param[in] orientation ENU or NED orientation of the world.
 * \param[in] reference What North reference does the azimuth use.
 * \return The suffix.
 */
template<typename T, typename ::std::enable_if_t<
    std::is_same_v<T, compass_interfaces::msg::Azimuth> ||
    std::is_same_v<T, geometry_msgs::msg::PoseWithCovarianceStamped> ||
    std::is_same_v<T, geometry_msgs::msg::QuaternionStamped> ||
    std::is_same_v<T, sensor_msgs::msg::Imu>
  >* = nullptr>
std::string getAzimuthTopicSuffix(Unit unit, Orientation orientation, Reference reference);

/**
 * \brief Autodetect azimuth representation from the name of the topic on which the message came.
 *
 * \param[in] topic The topic to parse.
 * \return The autodetected parameters, or nullopt if autodetection failed.
 */
std::optional<std::tuple<Unit, Orientation, Reference>> parseAzimuthTopicName(const std::string& topic);

/**
 * \brief Autodetect azimuth representation from connection header of a topic it came on.
 *
 * \param[in] connectionHeaderPtr Pointer to the connection header, should contain key "topic".
 * \return The autodetected parameters, or nullopt if autodetection failed.
 */
std::optional<std::tuple<Unit, Orientation, Reference>> parseAzimuthTopicName(
  const std::shared_ptr<std::map<std::string, std::string>>& connectionHeaderPtr);

}
