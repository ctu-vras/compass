// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief String utilities for compass_msgs.
 * \author Martin Pecka
 */

#include <string>

#include <compass_msgs/Azimuth.h>
#include <compass_msgs/string_utils.h>
#include <cras_cpp_common/string_utils.hpp>

namespace compass_msgs
{

std::string unitToString(const decltype(Azimuth::unit) unit)
{
  switch (unit)
  {
    case Azimuth::UNIT_RAD:
      return "rad";
    case Azimuth::UNIT_DEG:
      return "deg";
    default:
      throw std::runtime_error(cras::format("Invalid Azimuth::unit %u", unit));
  }
}

decltype(Azimuth::unit) parseUnit(const std::string& unitStr)
{
  const auto unit = cras::toLower(unitStr);
  if (unit == "rad")
    return Azimuth::UNIT_RAD;
  else if (unit == "deg")
    return Azimuth::UNIT_DEG;
  else
    throw std::runtime_error(cras::format("Invalid Azimuth::unit '%s'", unitStr.c_str()));
}

std::string orientationToString(const decltype(Azimuth::orientation) orientation)
{
  switch (orientation)
  {
    case Azimuth::ORIENTATION_ENU:
      return "ENU";
    case Azimuth::ORIENTATION_NED:
      return "NED";
    default:
      throw std::runtime_error(cras::format("Invalid Azimuth::orientation %u", orientation));
  }
}

decltype(Azimuth::orientation) parseOrientation(const std::string& orientationStr)
{
  const auto orientation = cras::toLower(orientationStr);
  if (orientation == "enu")
    return Azimuth::ORIENTATION_ENU;
  else if (orientation == "ned")
    return Azimuth::ORIENTATION_NED;
  else
    throw std::runtime_error(cras::format("Invalid Azimuth::orientation '%s'", orientationStr.c_str()));
}

std::string referenceToString(const decltype(Azimuth::reference) reference)
{
  switch (reference)
  {
    case Azimuth::REFERENCE_MAGNETIC:
      return "magnetic";
    case Azimuth::REFERENCE_GEOGRAPHIC:
      return "geographic";
    case Azimuth::REFERENCE_UTM:
      return "UTM";
    default:
      throw std::runtime_error(cras::format("Invalid Azimuth::reference %u", reference));
  }
}

decltype(Azimuth::reference) parseReference(const std::string& referenceStr)
{
  const auto reference = cras::toLower(referenceStr);
  if (reference == "magnetic")
    return Azimuth::REFERENCE_MAGNETIC;
  else if (reference == "geographic" || reference == "true")
    return Azimuth::REFERENCE_GEOGRAPHIC;
  else if (reference == "utm")
    return Azimuth::REFERENCE_UTM;
  else
    throw std::runtime_error(cras::format("Invalid Azimuth::reference '%s'", referenceStr.c_str()));
}

}
