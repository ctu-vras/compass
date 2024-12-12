// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_msgs::Azimuth messages.
 * \author Martin Pecka
 */

#include <string>

#include <angles/angles.h>
#include <compass_conversions/tf2_compass_msgs.h>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/tf2_utils.hpp>
#include <geometry_msgs/TransformStamped.h>

using Az = compass_msgs::Azimuth;

namespace tf2
{

template<>
const ros::Time& getTimestamp(const compass_msgs::Azimuth& t)
{
  return t.header.stamp;
}

template<>
const std::string& getFrameId(const compass_msgs::Azimuth& t)
{
  return t.header.frame_id;
}

compass_msgs::Azimuth toMsg(const compass_msgs::Azimuth& in)
{
  return in;
}

void fromMsg(const compass_msgs::Azimuth& msg, compass_msgs::Azimuth& out)
{
  out = msg;
}

template<>
void doTransform(
  const compass_msgs::Azimuth& t_in, compass_msgs::Azimuth& t_out, const geometry_msgs::TransformStamped& transform)
{
  t_out = t_in;
  t_out.header.frame_id = transform.header.frame_id;
  t_out.header.stamp = transform.header.stamp;

  auto yaw = cras::getYaw(transform.transform.rotation);
  if (t_in.unit == Az::UNIT_DEG)
    yaw = angles::to_degrees(yaw);

  t_out.azimuth += yaw * (t_in.orientation == Az::ORIENTATION_NED ? -1 : 1);
}

}
