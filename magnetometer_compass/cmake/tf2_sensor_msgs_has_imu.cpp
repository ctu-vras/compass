// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Test if tf2_sensor_msgs can handle IMU messages (https://github.com/ros2/geometry2/issues/800)
 * \author Martin Pecka
 */

#include <sensor_msgs/msg/imu.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

int main(int, char**)
{
  sensor_msgs::msg::Imu imu;
  const auto stamp = tf2::getTimestamp(imu);
  return 0;
}
