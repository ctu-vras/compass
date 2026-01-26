// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief String utilities for compass packages.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <string>

#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_interfaces/string_utils.hpp>
#include <cras_cpp_common/format.hpp>
#include <cras_cpp_common/string_utils.hpp>

namespace compass_interfaces
{

std::string unitToString(const msg::Azimuth::_unit_type unit)
{
  switch (unit)
  {
    case compass_interfaces::msg::Azimuth::UNIT_RAD:
      return "rad";
    case compass_interfaces::msg::Azimuth::UNIT_DEG:
      return "deg";
    default:
      throw std::runtime_error(cras::format("Invalid compass_interfaces::msg::Azimuth::unit {}", std::to_string(unit)));
  }
}

msg::Azimuth::_unit_type parseUnit(const std::string& unitStr)
{
  const auto unit = cras::toLower(unitStr);
  if (unit == "rad")
    return compass_interfaces::msg::Azimuth::UNIT_RAD;
  else if (unit == "deg")
    return compass_interfaces::msg::Azimuth::UNIT_DEG;
  else
    throw std::runtime_error(cras::format("Invalid compass_interfaces::msg::Azimuth::unit '{}'", unitStr.c_str()));
}

std::string orientationToString(const msg::Azimuth::_orientation_type orientation)
{
  switch (orientation)
  {
    case compass_interfaces::msg::Azimuth::ORIENTATION_ENU:
      return "ENU";
    case compass_interfaces::msg::Azimuth::ORIENTATION_NED:
      return "NED";
    default:
      throw std::runtime_error(
        cras::format("Invalid compass_interfaces::msg::Azimuth::orientation {}", std::to_string(orientation)));
  }
}

msg::Azimuth::_orientation_type parseOrientation(const std::string& orientationStr)
{
  const auto orientation = cras::toLower(orientationStr);
  if (orientation == "enu")
    return compass_interfaces::msg::Azimuth::ORIENTATION_ENU;
  else if (orientation == "ned")
    return compass_interfaces::msg::Azimuth::ORIENTATION_NED;
  else
    throw std::runtime_error(
      cras::format("Invalid compass_interfaces::msg::Azimuth::orientation '{}'", orientationStr.c_str()));
}

std::string referenceToString(const msg::Azimuth::_reference_type reference)
{
  switch (reference)
  {
    case compass_interfaces::msg::Azimuth::REFERENCE_MAGNETIC:
      return "magnetic";
    case compass_interfaces::msg::Azimuth::REFERENCE_GEOGRAPHIC:
      return "geographic";
    case compass_interfaces::msg::Azimuth::REFERENCE_UTM:
      return "UTM";
    default:
      throw std::runtime_error(
        cras::format("Invalid compass_interfaces::msg::Azimuth::reference {}", std::to_string(reference)));
  }
}

msg::Azimuth::_reference_type parseReference(const std::string& referenceStr)
{
  const auto reference = cras::toLower(referenceStr);
  if (reference == "magnetic")
    return compass_interfaces::msg::Azimuth::REFERENCE_MAGNETIC;
  else if (reference == "geographic" || reference == "true")
    return compass_interfaces::msg::Azimuth::REFERENCE_GEOGRAPHIC;
  else if (reference == "utm")
    return compass_interfaces::msg::Azimuth::REFERENCE_UTM;
  else
    throw std::runtime_error(
      cras::format("Invalid compass_interfaces::msg::Azimuth::reference '{}'", referenceStr.c_str()));
}

}
