// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Earth magnetic field model.
 * \author Martin Pecka
 */

#include <ctime>
#include <memory>
#include <string>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/MagneticModel.hpp>

#include <angles/angles.h>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <magnetic_model/magnetic_model.h>
#include <ros/time.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

namespace magnetic_model
{

const char* MagneticModel::WMM2010 = "wmm2010";
const char* MagneticModel::WMM2015 = "wmm2015v2";
const char* MagneticModel::WMM2020 = "wmm2020";

using Az = compass_msgs::Azimuth;

struct ModelErrors
{
  double X;  // nT
  double Y;  // nT
  double Z;  // nT
  double H;  // nT
  double F;  // nT
  double D_ofs;
  double D_lin;
  double I;  // degrees
};

struct MagneticModelPrivate
{
  //! \brief The GeographicLib magnetic field model.
  std::unique_ptr<GeographicLib::MagneticModel> magneticModel;

  ModelErrors errors{};
};

MagneticModel::MagneticModel(
  const cras::LogHelperPtr& log, const std::string& name, const std::string& modelPath, const bool strict) :
  cras::HasLogger(log), strict(strict), data(new MagneticModelPrivate{})
{
  try
  {
    this->data->magneticModel = std::make_unique<GeographicLib::MagneticModel>(name, modelPath);
  }
  catch (const GeographicLib::GeographicErr& e)
  {
    throw std::invalid_argument(cras::format(
      "Could not create magnetic field model %s because of the following error: %s", name.c_str(), e.what()));
  }

  if (name == WMM2010)
  {
    // https://geomag.bgs.ac.uk/documents/WMM2010_Report.pdf, Section 3.3
    this->data->errors.X = 405;
    this->data->errors.Y = 244;
    this->data->errors.Z = 559;
    this->data->errors.H = 371;
    this->data->errors.F = 614;
    this->data->errors.I = 0.38;
    this->data->errors.D_ofs = 1.71;
    this->data->errors.D_lin = 0;
  }
  else if (name == WMM2015)
  {
    // https://repository.library.noaa.gov/view/noaa/48055/noaa_48055_DS1.pdf?download-document-submit=Download, Sec 3.4
    this->data->errors.X = 138;
    this->data->errors.Y = 89;
    this->data->errors.Z = 165;
    this->data->errors.H = 133;
    this->data->errors.F = 152;
    this->data->errors.I = 0.22;
    this->data->errors.D_ofs = 0.23;
    this->data->errors.D_lin = 5430;
  }
  else if (name == WMM2020)
  {
    // https://repository.library.noaa.gov/view/noaa/24390/noaa_24390_DS1.pdf?download-document-submit=Download, Sec 3.4
    this->data->errors.X = 131;
    this->data->errors.Y = 94;
    this->data->errors.Z = 157;
    this->data->errors.H = 128;
    this->data->errors.F = 145;
    this->data->errors.I = 0.21;
    this->data->errors.D_ofs = 0.26;
    this->data->errors.D_lin = 5625;
  }

  CRAS_INFO("Initialized magnetic model %s.", name.c_str());
}

MagneticModel::~MagneticModel() = default;
bool MagneticModel::isValid(const ros::Time& time) const
{
  return this->isValid(cras::getYear(time));
}

bool MagneticModel::isValid(const int year) const
{
  return year >= this->data->magneticModel->MinTime() && year < this->data->magneticModel->MaxTime();
}

cras::expected<MagneticField, std::string> MagneticModel::getMagneticField(
  const sensor_msgs::NavSatFix& fix, const ros::Time& stamp) const
{
  double errorCoef = 1.0;

  const auto year = cras::getYear(stamp);
  const auto minYear = this->data->magneticModel->MinTime();
  const auto maxYear = this->data->magneticModel->MaxTime();
  if (year < minYear || year > maxYear)
  {
    const auto err = cras::format("Using magnetic field model %s for an invalid year %u!",
      this->data->magneticModel->MagneticModelName().c_str(), year);
    if (this->strict)
    {
      return cras::make_unexpected(err);
    }
    else
    {
      CRAS_ERROR_THROTTLE(10.0, "%s", err.c_str());
      errorCoef *= std::max(1.0, std::max(std::abs(year - minYear), std::abs(year - maxYear)));
    }
  }

  const auto minAlt = this->data->magneticModel->MinHeight();
  const auto maxAlt = this->data->magneticModel->MaxHeight();
  if (fix.altitude < minAlt || fix.altitude > maxAlt)
  {
    const auto err = cras::format(
      "Using magnetic field model %s in altitude %.01f m which is outside the model range.",
      this->data->magneticModel->MagneticModelName().c_str(), fix.altitude);
    if (this->strict)
    {
      return cras::make_unexpected(err);
    }
    else
    {
      CRAS_ERROR_THROTTLE(10.0, "%s", err.c_str());
      errorCoef *= std::max(1.0, std::max(std::abs(fix.altitude - minAlt), std::abs(fix.altitude - maxAlt)) / 1000.0);
    }
  }

  MagneticField result;
  (*this->data->magneticModel)(year, fix.latitude, fix.longitude, fix.altitude,
    result.field.magnetic_field.x, result.field.magnetic_field.y, result.field.magnetic_field.z,
    result.dt.x, result.dt.y, result.dt.z);

  result.field.header.frame_id = fix.header.frame_id;
  result.field.header.stamp = stamp;
  result.field.magnetic_field.x *= 1e-9;
  result.field.magnetic_field.y *= 1e-9;
  result.field.magnetic_field.z *= 1e-9;
  result.dt.x *= 1e-9;
  result.dt.y *= 1e-9;
  result.dt.z *= 1e-9;
  result.error.x = errorCoef * this->data->errors.X * 1e-9;
  result.error.y = errorCoef * this->data->errors.Y * 1e-9;
  result.error.z = errorCoef * this->data->errors.Z * 1e-9;
  result.field.magnetic_field_covariance[0 * 3 + 0] = std::pow(result.error.x, 2);
  result.field.magnetic_field_covariance[1 * 3 + 1] = std::pow(result.error.y, 2);
  result.field.magnetic_field_covariance[2 * 3 + 2] = std::pow(result.error.z, 2);

  return result;
}

cras::expected<MagneticFieldComponentProperties, std::string> MagneticModel::getMagneticFieldComponents(
  const sensor_msgs::NavSatFix& fix, const ros::Time& stamp) const
{
  double errorCoef = 1.0;
  const auto minAlt = this->data->magneticModel->MinHeight();
  const auto maxAlt = this->data->magneticModel->MaxHeight();
  if (fix.altitude < minAlt || fix.altitude > maxAlt)
  {
    const auto err = cras::format(
      "Using magnetic field model %s in altitude %.01f m which is outside the model range.",
      this->data->magneticModel->MagneticModelName().c_str(), fix.altitude);
    if (this->strict)
    {
      return cras::make_unexpected(err);
    }
    else
    {
      CRAS_ERROR_THROTTLE(10.0, "%s", err.c_str());
      errorCoef *= std::max(1.0, std::max(std::abs(fix.altitude - minAlt), std::abs(fix.altitude - maxAlt)) / 1000.0);
    }
  }

  const auto field = this->getMagneticField(fix, stamp);
  if (!field.has_value())
    return cras::make_unexpected(field.error());

  auto result = this->getMagneticFieldComponents(*field, stamp);
  if (result.has_value())
  {
    result->errors.horizontalMagnitude *= errorCoef;
    result->errors.totalMagnitude *= errorCoef;
    result->errors.declination *= errorCoef;
    result->errors.inclination *= errorCoef;
  }

  return result;
}

cras::expected<MagneticFieldComponentProperties, std::string> MagneticModel::getMagneticFieldComponents(
  const MagneticField& field, const ros::Time& stamp) const
{
  double errorCoef = 1.0;
  const auto year = cras::getYear(stamp);
  const auto minYear = this->data->magneticModel->MinTime();
  const auto maxYear = this->data->magneticModel->MaxTime();
  if (year < minYear || year > maxYear)
  {
    const auto err = cras::format("Using magnetic field model %s for an invalid year %u!",
      this->data->magneticModel->MagneticModelName().c_str(), year);
    if (this->strict)
    {
      return cras::make_unexpected(err);
    }
    else
    {
      CRAS_ERROR_THROTTLE(10.0, "%s", err.c_str());
      errorCoef *= std::max(1.0, std::max(std::abs(year - minYear), std::abs(year - maxYear)));
    }
  }

  MagneticFieldComponentProperties res;
  GeographicLib::MagneticModel::FieldComponents(
    field.field.magnetic_field.x * 1e9, field.field.magnetic_field.y * 1e9, field.field.magnetic_field.z * 1e9,
    field.dt.x * 1e9, field.dt.y * 1e9, field.dt.z * 1e9,
    res.values.horizontalMagnitude, res.values.totalMagnitude, res.values.declination, res.values.inclination,
    res.dt.horizontalMagnitude, res.dt.totalMagnitude, res.dt.declination, res.dt.inclination);

  // Compute declination before scaling down the results from nT to T
  res.errors.declination = errorCoef * angles::from_degrees(std::sqrt(
    std::pow(this->data->errors.D_ofs, 2) +
    std::pow(this->data->errors.D_lin / res.values.horizontalMagnitude, 2)));
  res.values.horizontalMagnitude *= 1e-9;
  res.values.totalMagnitude *= 1e-9;
  res.values.declination = angles::from_degrees(res.values.declination);
  res.values.inclination = angles::from_degrees(res.values.inclination);
  res.dt.horizontalMagnitude *= 1e-9;
  res.dt.totalMagnitude *= 1e-9;
  res.dt.declination = angles::from_degrees(res.dt.declination);
  res.dt.inclination = angles::from_degrees(res.dt.inclination);
  res.errors.horizontalMagnitude = errorCoef * this->data->errors.H * 1e-9;
  res.errors.totalMagnitude = errorCoef * this->data->errors.F * 1e-9;
  res.errors.inclination = errorCoef * angles::from_degrees(this->data->errors.I);

  tm t{};
  t.tm_year = year - 1900;
  t.tm_mon = 0;
  t.tm_mday = 1;
  t.tm_hour = 0;
  t.tm_min = 0;
  t.tm_sec = 0;
  const auto yearStart = cras::fromStructTm(t);

  t.tm_year = year + 1 - 1900;
  const auto nextYearStart = cras::fromStructTm(t);

  double yearFrac {0.0};
  if (yearStart.has_value() && nextYearStart.has_value())
  {
    const auto yearDuration = *nextYearStart - *yearStart;
    const auto nowSinceYearStart = stamp - *yearStart;
    yearFrac = (nowSinceYearStart / yearDuration).toSec();
  }

  res.values.horizontalMagnitude += yearFrac * res.dt.horizontalMagnitude;
  res.values.totalMagnitude += yearFrac * res.dt.totalMagnitude;
  res.values.declination += yearFrac * res.dt.declination;
  res.values.inclination += yearFrac * res.dt.inclination;

  return res;
}

}
