#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_msgs::Azimuth messages.
 * \author Martin Pecka
 */

#include <string>

#include <compass_msgs/Azimuth.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <tf2/convert.h>

namespace tf2
{
template<> const ros::Time& getTimestamp(const compass_msgs::Azimuth& t);
template<> const std::string& getFrameId(const compass_msgs::Azimuth& t);

compass_msgs::Azimuth toMsg(const compass_msgs::Azimuth& in);
void fromMsg(const compass_msgs::Azimuth& msg, compass_msgs::Azimuth& out);

template<>
void doTransform(
  const compass_msgs::Azimuth& t_in, compass_msgs::Azimuth& t_out, const geometry_msgs::TransformStamped& transform);
}
