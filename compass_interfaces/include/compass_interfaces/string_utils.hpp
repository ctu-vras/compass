#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief String utilities for compass packages. Adapted from cras_cpp_common.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <string>

#include <compass_interfaces/msg/azimuth.hpp>

namespace compass_interfaces
{

/**
 * \brief Convert the given azimuth unit to string.
 * \param[in] unit The unit.
 * \return The string.
 * \throw std::runtime_error If the unit is invalid.
 */
std::string unitToString(msg::Azimuth::_unit_type unit);

/**
 * \brief Convert the given string to azimuth unit.
 * \param[in] unitStr A string describing the unit.
 * \return The unit.
 * \throw std::runtime_error If the unit is invalid.
 */
msg::Azimuth::_unit_type parseUnit(const std::string& unitStr);

/**
 * \brief Convert the given azimuth orientation to string.
 * \param[in] orientation The orientation.
 * \return The string.
 * \throw std::runtime_error If the orientation is invalid.
 */
std::string orientationToString(msg::Azimuth::_orientation_type orientation);

/**
 * \brief Convert the given string to azimuth orientation.
 * \param[in] orientationStr A string describing the orientation.
 * \return The orientation.
 * \throw std::runtime_error If the orientation is invalid.
 */
msg::Azimuth::_orientation_type parseOrientation(const std::string& orientationStr);

/**
 * \brief Convert the given azimuth reference to string.
 * \param[in] reference The reference.
 * \return The string.
 * \throw std::runtime_error If the reference is invalid.
 */
std::string referenceToString(msg::Azimuth::_reference_type reference);

/**
 * \brief Convert the given string to azimuth reference.
 * \param[in] referenceStr A string describing the reference.
 * \return The reference.
 * \throw std::runtime_error If the reference is invalid.
 */
msg::Azimuth::_reference_type parseReference(const std::string& referenceStr);

}
