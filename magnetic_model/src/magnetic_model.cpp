// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Earth magnetic field model.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <ctime>
#include <memory>
#include <string>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/MagneticModel.hpp>

#include <angles/angles.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/format.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <magnetic_model/magnetic_model.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace magnetic_model {

const char* MagneticModel::GAZEBO = "gazebo";
const char* MagneticModel::IGRF14 = "igrf14";
const char* MagneticModel::WMM2010 = "wmm2010";
const char* MagneticModel::WMM2015 = "wmm2015v2";
const char* MagneticModel::WMM2020 = "wmm2020";
const char* MagneticModel::WMM2025 = "wmm2025";

struct ModelErrors {
  double X;  // nT
  double Y;  // nT
  double Z;  // nT
  double H;  // nT
  double F;  // nT
  double D_ofs;
  double D_lin;
  double I;  // degrees
};

struct MagneticModelPrivate {
  //! \brief The GeographicLib magnetic field model.
  std::unique_ptr<GeographicLib::MagneticModel> magnetic_model_;

  ModelErrors errors_ {};

  bool is_gazebo_ {false};  //!< True for the Gazebo model which needs to fix date and zero-out secular variation

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr log_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_;
};

MagneticModel::MagneticModel(
    RequiredInterfaces node, const std::string& name, const std::string& model_path, const bool strict)
    : strict_(strict), data_(new MagneticModelPrivate{}), node_(node) {
  data_->log_ = node.get_node_logging_interface();
  data_->clock_ = node.get_node_clock_interface();

  data_->is_gazebo_ = name == GAZEBO;
  const auto model_name = data_->is_gazebo_ ? IGRF14 : name;
  try {
    data_->magnetic_model_ = std::make_unique<GeographicLib::MagneticModel>(model_name, model_path);
  } catch (const GeographicLib::GeographicErr& e) {
    throw std::invalid_argument(cras::format(
      "Could not create magnetic field model {} because of the following error: {}", name, e.what()));
  }

  if (name == WMM2010) {
    // https://geomag.bgs.ac.uk/documents/WMM2010_Report.pdf, Section 3.3
    data_->errors_.X = 405;
    data_->errors_.Y = 244;
    data_->errors_.Z = 559;
    data_->errors_.H = 371;
    data_->errors_.F = 614;
    data_->errors_.I = 0.38;
    data_->errors_.D_ofs = 1.71;
    data_->errors_.D_lin = 0;
  } else if (name == WMM2015) {
    // https://repository.library.noaa.gov/view/noaa/48055/noaa_48055_DS1.pdf?download-document-submit=Download, Sec 3.4
    data_->errors_.X = 138;
    data_->errors_.Y = 89;
    data_->errors_.Z = 165;
    data_->errors_.H = 133;
    data_->errors_.F = 152;
    data_->errors_.I = 0.22;
    data_->errors_.D_ofs = 0.23;
    data_->errors_.D_lin = 5430;
  } else if (name == WMM2020) {
    // https://repository.library.noaa.gov/view/noaa/24390/noaa_24390_DS1.pdf?download-document-submit=Download, Sec 3.4
    data_->errors_.X = 131;
    data_->errors_.Y = 94;
    data_->errors_.Z = 157;
    data_->errors_.H = 128;
    data_->errors_.F = 145;
    data_->errors_.I = 0.21;
    data_->errors_.D_ofs = 0.26;
    data_->errors_.D_lin = 5625;
  } else if (name == WMM2025) {
    // https://www.ncei.noaa.gov/products/world-magnetic-model/accuracy-limitations-error-model
    data_->errors_.X = 137;
    data_->errors_.Y = 89;
    data_->errors_.Z = 141;
    data_->errors_.H = 133;
    data_->errors_.F = 138;
    data_->errors_.I = 0.20;
    data_->errors_.D_ofs = 0.26;
    data_->errors_.D_lin = 5417;
  } else if (name == IGRF14) {
    // https://earth-planets-space.springeropen.com/articles/10.1186/s40623-022-01572-y/tables/1
    data_->errors_.X = 144;
    data_->errors_.Y = 136;
    data_->errors_.Z = 293;
    data_->errors_.H = 135;
    data_->errors_.F = 178;
    data_->errors_.I = 0.29;
    data_->errors_.D_ofs = 0.39;
    data_->errors_.D_lin = 0;
  } else if (name == GAZEBO) {
    data_->errors_.X = 0;
    data_->errors_.Y = 0;
    data_->errors_.Z = 0;
    data_->errors_.H = 0;
    data_->errors_.F = 0;
    data_->errors_.I = 0;
    data_->errors_.D_ofs = 0;
    data_->errors_.D_lin = 0;
  }

  RCLCPP_INFO(data_->log_->get_logger(), "Initialized magnetic model %s.", name.c_str());
}

MagneticModel::~MagneticModel() = default;

bool MagneticModel::isValid(const rclcpp::Time& time) const {
  return isValid(cras::getYear(time));
}

bool MagneticModel::isValid(const int year) const {
  if (data_->is_gazebo_) {
    return true;
  }
  return year >= data_->magnetic_model_->MinTime() && year < data_->magnetic_model_->MaxTime();
}

cras::expected<MagneticField, std::string> MagneticModel::getMagneticField(
    const sensor_msgs::msg::NavSatFix& fix_msg, const rclcpp::Time& stamp_in) const {
  auto fix = fix_msg;
  auto stamp = stamp_in;
  if (data_->is_gazebo_) {
    fix.altitude = 0;
    stamp = {1516579200, 0};
  }

  double error_coef = 1.0;

  const auto year = cras::getYear(stamp);
  const auto min_year = data_->magnetic_model_->MinTime();
  const auto max_year = data_->magnetic_model_->MaxTime();
  if (year < min_year || year > max_year) {
    const auto err = cras::format("Using magnetic field model {} for an invalid year {}!",
      data_->magnetic_model_->MagneticModelName(), std::to_string(year));
    if (strict_) {
      return cras::make_unexpected(err);
    } else {
      RCLCPP_ERROR_THROTTLE(data_->log_->get_logger(), *data_->clock_->get_clock(), 10000., "%s", err.c_str());
      error_coef *= std::max(1.0, std::max(std::abs(year - min_year), std::abs(year - max_year)));
    }
  }

  const auto min_alt = data_->magnetic_model_->MinHeight();
  const auto max_alt = data_->magnetic_model_->MaxHeight();
  if (fix.altitude < min_alt || fix.altitude > max_alt) {
    const auto err = cras::format(
      "Using magnetic field model {} in altitude {} m which is outside the model range.",
      data_->magnetic_model_->MagneticModelName(), std::to_string(fix.altitude));
    if (strict_) {
      return cras::make_unexpected(err);
    } else {
      RCLCPP_ERROR_THROTTLE(data_->log_->get_logger(), *data_->clock_->get_clock(), 10000., "%s", err.c_str());
      error_coef *= std::max(
        1.0,
        std::max(
          std::abs(fix.altitude - min_alt),
          std::abs(fix.altitude - max_alt)
          ) / 1000.0
      );
    }
  }

  MagneticField result;
  (*data_->magnetic_model_)(
    year, fix.latitude, fix.longitude, fix.altitude,
    result.field.magnetic_field.x, result.field.magnetic_field.y, result.field.magnetic_field.z,
    result.dt.x, result.dt.y, result.dt.z);

  result.field.header.frame_id = fix.header.frame_id;
  result.field.header.stamp = stamp_in;
  result.field.magnetic_field.x *= 1e-9;
  result.field.magnetic_field.y *= 1e-9;
  result.field.magnetic_field.z *= 1e-9;
  result.dt.x *= 1e-9;
  result.dt.y *= 1e-9;
  result.dt.z *= 1e-9;
  result.error.x = error_coef * data_->errors_.X * 1e-9;
  result.error.y = error_coef * data_->errors_.Y * 1e-9;
  result.error.z = error_coef * data_->errors_.Z * 1e-9;
  result.field.magnetic_field_covariance[0 * 3 + 0] = std::pow(result.error.x, 2);
  result.field.magnetic_field_covariance[1 * 3 + 1] = std::pow(result.error.y, 2);
  result.field.magnetic_field_covariance[2 * 3 + 2] = std::pow(result.error.z, 2);

  return result;
}

cras::expected<MagneticFieldComponentProperties, std::string> MagneticModel::getMagneticFieldComponents(
    const sensor_msgs::msg::NavSatFix& fix_msg, const rclcpp::Time& stamp_in) const {
  auto fix = fix_msg;
  auto stamp = stamp_in;
  if (data_->is_gazebo_) {
    fix.altitude = 0;
    stamp = {1516579200, 0};
  }

  double error_coef = 1.0;
  const auto min_alt = data_->magnetic_model_->MinHeight();
  const auto max_alt = data_->magnetic_model_->MaxHeight();
  if (fix.altitude < min_alt || fix.altitude > max_alt) {
    const auto err = cras::format(
      "Using magnetic field model {} in altitude {} m which is outside the model range.",
      data_->magnetic_model_->MagneticModelName(), std::to_string(fix.altitude));
    if (strict_) {
      return cras::make_unexpected(err);
    } else {
      RCLCPP_ERROR_THROTTLE(data_->log_->get_logger(), *data_->clock_->get_clock(), 10000., "%s", err.c_str());
      error_coef *= std::max(
        1.0,
        std::max(
          std::abs(fix.altitude - min_alt),
          std::abs(fix.altitude - max_alt)
          ) / 1000.0
      );
    }
  }

  const auto field = getMagneticField(fix, stamp);
  if (!field.has_value()) {
    return cras::make_unexpected(field.error());
  }

  auto result = getMagneticFieldComponents(*field, stamp);
  if (result.has_value()) {
    result->errors.horizontal_magnitude *= error_coef;
    result->errors.total_magnitude *= error_coef;
    result->errors.declination *= error_coef;
    result->errors.inclination *= error_coef;
  }

  return result;
}

cras::expected<MagneticFieldComponentProperties, std::string> MagneticModel::getMagneticFieldComponents(
    const MagneticField& field, const rclcpp::Time& stamp_in) const {
  auto stamp = stamp_in;
  if (data_->is_gazebo_) {
    stamp = {1516579200, 0};
  }

  double error_coef = 1.0;
  const auto year = cras::getYear(stamp);
  const auto min_year = data_->magnetic_model_->MinTime();
  const auto max_year = data_->magnetic_model_->MaxTime();
  if (year < min_year || year > max_year) {
    const auto err = cras::format("Using magnetic field model {} for an invalid year {}!",
      data_->magnetic_model_->MagneticModelName(), std::to_string(year));
    if (strict_) {
      return cras::make_unexpected(err);
    } else {
      RCLCPP_ERROR_THROTTLE(data_->log_->get_logger(), *data_->clock_->get_clock(), 10000., "%s", err.c_str());
      error_coef *= std::max(1.0, std::max(std::abs(year - min_year), std::abs(year - max_year)));
    }
  }

  MagneticFieldComponentProperties res;
  GeographicLib::MagneticModel::FieldComponents(
    field.field.magnetic_field.x * 1e9, field.field.magnetic_field.y * 1e9, field.field.magnetic_field.z * 1e9,
    field.dt.x * 1e9, field.dt.y * 1e9, field.dt.z * 1e9,
    res.values.horizontal_magnitude, res.values.total_magnitude, res.values.declination, res.values.inclination,
    res.dt.horizontal_magnitude, res.dt.total_magnitude, res.dt.declination, res.dt.inclination);

  // Compute declination before scaling down the results from nT to T
  res.errors.declination = error_coef * angles::from_degrees(std::sqrt(
    std::pow(data_->errors_.D_ofs, 2) +
    std::pow(data_->errors_.D_lin / res.values.horizontal_magnitude, 2)));
  res.values.horizontal_magnitude *= 1e-9;
  res.values.total_magnitude *= 1e-9;
  res.values.declination = angles::from_degrees(res.values.declination);
  res.values.inclination = angles::from_degrees(res.values.inclination);
  res.dt.horizontal_magnitude *= 1e-9;
  res.dt.total_magnitude *= 1e-9;
  res.dt.declination = angles::from_degrees(res.dt.declination);
  res.dt.inclination = angles::from_degrees(res.dt.inclination);
  res.errors.horizontal_magnitude = error_coef * data_->errors_.H * 1e-9;
  res.errors.total_magnitude = error_coef * data_->errors_.F * 1e-9;
  res.errors.inclination = error_coef * angles::from_degrees(data_->errors_.I);

  tm t{};
  t.tm_year = year - 1900;
  t.tm_mon = 0;
  t.tm_mday = 1;
  t.tm_hour = 0;
  t.tm_min = 0;
  t.tm_sec = 0;
  const auto year_start = cras::fromStructTm(t);

  t.tm_year = year + 1 - 1900;
  const auto next_year_start = cras::fromStructTm(t);

  double year_frac {0.0};
  if (year_start.has_value() && next_year_start.has_value()) {
    const double yearDuration = next_year_start->seconds() - year_start->seconds();
    const double nowSinceYearStart = stamp.seconds() - year_start->seconds();
    year_frac = (nowSinceYearStart / yearDuration);
  }

  res.values.horizontal_magnitude += year_frac * res.dt.horizontal_magnitude;
  res.values.total_magnitude += year_frac * res.dt.total_magnitude;
  res.values.declination += year_frac * res.dt.declination;
  res.values.inclination += year_frac * res.dt.inclination;

  if (data_->is_gazebo_) {
    res.dt.horizontal_magnitude = 0;
    res.dt.total_magnitude = 0;
    res.dt.declination = 0;
    res.dt.inclination = 0;
  }

  return res;
}

}  // namespace magnetic_model
