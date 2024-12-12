#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Logic for naming topics according to the type of Azimuth message they carry.
 * \author Martin Pecka
 */

#include <string>
#include <type_traits>
#include <utility>

#include <boost/shared_ptr.hpp>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/optional.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <ros/datatypes.h>
#include <sensor_msgs/Imu.h>

namespace compass_conversions
{

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
    std::is_same<T, compass_msgs::Azimuth>::value ||
    std::is_same<T, geometry_msgs::PoseWithCovarianceStamped>::value ||
    std::is_same<T, geometry_msgs::QuaternionStamped>::value ||
    std::is_same<T, sensor_msgs::Imu>::value
  >* = nullptr>
std::string getAzimuthTopicSuffix(
  decltype(compass_msgs::Azimuth::unit) unit,
  decltype(compass_msgs::Azimuth::orientation) orientation,
  decltype(compass_msgs::Azimuth::reference) reference);

/**
 * \brief Autodetect azimuth representation from the name of the topic on which the message came.
 *
 * \param[in] topic The topic to parse.
 * \return The autodetected parameters, or nullopt if autodetection failed.
 */
cras::optional<std::tuple<
  decltype(compass_msgs::Azimuth::unit),
  decltype(compass_msgs::Azimuth::orientation),
  decltype(compass_msgs::Azimuth::reference)
>> parseAzimuthTopicName(const std::string& topic);

/**
 * \brief Autodetect azimuth representation from connection header of a topic it came on.
 *
 * \param[in] connectionHeaderPtr Pointer to the connection header, should contain key "topic".
 * \return The autodetected parameters, or nullopt if autodetection failed.
 */
cras::optional<std::tuple<
  decltype(compass_msgs::Azimuth::unit),
  decltype(compass_msgs::Azimuth::orientation),
  decltype(compass_msgs::Azimuth::reference)
>> parseAzimuthTopicName(const boost::shared_ptr<ros::M_string>& connectionHeaderPtr);

}
