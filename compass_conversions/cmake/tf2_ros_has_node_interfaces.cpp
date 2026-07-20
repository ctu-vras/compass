// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Test availability of node interfaces constructors of tf2_ros classes.
 * \author Martin Pecka
 */

#include <tf2_ros/create_timer_ros.hpp>

struct Test : public tf2_ros::CreateTimerROS::RequiredInterfaces {};
