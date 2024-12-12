#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief String utilities for compass_msgs.
 * \author Martin Pecka
 */

#include <string>

#include <compass_msgs/Azimuth.h>

namespace compass_msgs
{

/**
 * \brief Convert the given azimuth unit to string.
 * \param[in] unit The unit.
 * \return The string.
 * \throw std::runtime_error If the unit is invalid.
 */
std::string unitToString(decltype(Azimuth::unit) unit);

/**
 * \brief Convert the given string to azimuth unit.
 * \param[in] unitStr A string describing the unit.
 * \return The unit.
 * \throw std::runtime_error If the unit is invalid.
 */
decltype(Azimuth::unit) parseUnit(const std::string& unitStr);

/**
 * \brief Convert the given azimuth orientation to string.
 * \param[in] orientation The orientation.
 * \return The string.
 * \throw std::runtime_error If the orientation is invalid.
 */
std::string orientationToString(decltype(Azimuth::orientation) orientation);

/**
 * \brief Convert the given string to azimuth orientation.
 * \param[in] orientationStr A string describing the orientation.
 * \return The orientation.
 * \throw std::runtime_error If the orientation is invalid.
 */
decltype(Azimuth::orientation) parseOrientation(const std::string& orientationStr);

/**
 * \brief Convert the given azimuth reference to string.
 * \param[in] reference The reference.
 * \return The string.
 * \throw std::runtime_error If the reference is invalid.
 */
std::string referenceToString(decltype(Azimuth::reference) reference);

/**
 * \brief Convert the given string to azimuth reference.
 * \param[in] referenceStr A string describing the reference.
 * \return The reference.
 * \throw std::runtime_error If the reference is invalid.
 */
decltype(Azimuth::reference) parseReference(const std::string& referenceStr);

}
