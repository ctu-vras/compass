// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_interfaces::msg::Azimuth messages.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <string>

#include <angles/angles.h>
#include <compass_conversions/tf2_compass_msgs.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
#include <tf2/time.hpp>

using Az = compass_interfaces::msg::Azimuth;

namespace tf2
{

template<>
tf2::TimePoint getTimestamp(const compass_interfaces::msg::Azimuth& t)
{
  return tf2::TimePoint(tf2::Duration(rclcpp::Time(t.header.stamp).nanoseconds()));
}

template<>
std::string getFrameId(const compass_interfaces::msg::Azimuth& t)
{
  return t.header.frame_id;
}

compass_interfaces::msg::Azimuth toMsg(const compass_interfaces::msg::Azimuth& in)
{
  return in;
}

void fromMsg(const compass_interfaces::msg::Azimuth& msg, compass_interfaces::msg::Azimuth& out)
{
  out = msg;
}

template<>
void doTransform(
  const compass_interfaces::msg::Azimuth& t_in, compass_interfaces::msg::Azimuth& t_out,
  const geometry_msgs::msg::TransformStamped& transform)
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
