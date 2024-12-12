// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert between various compass representations.
 * \author Martin Pecka
 */

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <angles/angles.h>
#include <compass_conversions/compass_converter.h>
#include <compass_conversions/topic_names.h>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <magnetic_model/magnetic_model.h>
#include <magnetic_model/magnetic_model_manager.h>
#include <ros/message_event.h>
#include <ros/message_traits.h>
#include <ros/package.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <topic_tools/shape_shifter.h>

namespace compass_conversions
{

using Az = compass_msgs::Azimuth;
using Imu = sensor_msgs::Imu;
using Pose = geometry_msgs::PoseWithCovarianceStamped;
using Quat = geometry_msgs::QuaternionStamped;

/**
 * \brief Private data of CompassConverter.
 */
struct CompassConverterPrivate
{
  //! \brief Magnetic model manager.
  std::unique_ptr<magnetic_model::MagneticModelManager> magneticModelManager;

  //! \brief Cache of already initialized magnetic field models. Keys are years.
  std::map<uint32_t, std::shared_ptr<magnetic_model::MagneticModel>> magneticModels;
};

CompassConverter::CompassConverter(const cras::LogHelperPtr& log, const bool strict) :
  cras::HasLogger(log), strict(strict), data(new CompassConverterPrivate{})
{
  this->data->magneticModelManager = std::make_unique<magnetic_model::MagneticModelManager>(this->log);
}

CompassConverter::~CompassConverter() = default;

void CompassConverter::configFromParams(const cras::BoundParamHelper& params)
{
  cras::TempLocale l(LC_ALL, "en_US.UTF-8");  // Support printing ° signs

  if (params.hasParam("magnetic_declination"))
    this->forceMagneticDeclination(params.getParam<double>("magnetic_declination", cras::nullopt, "rad"));
  else
    this->forcedMagneticModelName = params.getParam("magnetic_model", std::string());

  if (params.hasParam("magnetic_models_path"))
    this->setMagneticModelPath(params.getParam<std::string>("magnetic_models_path", cras::nullopt));

  if (params.hasParam("utm_grid_convergence"))
    this->forceUTMGridConvergence(params.getParam<double>("utm_grid_convergence", cras::nullopt, "rad"));

  if (params.hasParam("utm_zone"))
    this->forceUTMZone(params.getParam<int>("utm_zone", cras::nullopt));

  this->setKeepUTMZone(params.getParam("keep_utm_zone", this->keepUTMZone));

  if (!this->forcedMagneticDeclination.has_value() || !this->forcedUTMGridConvergence.has_value())
  {
    if (params.hasParam("initial_lat") && params.hasParam("initial_lon"))
    {
      sensor_msgs::NavSatFix msg;

      msg.latitude = params.getParam<double>("initial_lat", cras::nullopt, "°");
      msg.longitude = params.getParam<double>("initial_lon", cras::nullopt, "°");
      msg.altitude = params.getParam("initial_alt", 0.0, "m");

      std::list<std::string> computedValues;
      if (!this->forcedMagneticDeclination.has_value())
        computedValues.emplace_back("magnetic declination");
      if (!this->forcedUTMGridConvergence.has_value())
        computedValues.emplace_back("UTM grid convergence");

      CRAS_INFO(
        "Initial GPS coords for computation of %s are %.6f°, %.6f°, altitude %.0f m.",
        cras::join(computedValues, "and").c_str(), msg.latitude, msg.longitude, msg.altitude);

      this->setNavSatPos(msg);
    }
  }
}

void CompassConverter::forceMagneticDeclination(const cras::optional<double>& declination)
{
  this->forcedMagneticDeclination = declination;
}

void CompassConverter::forceUTMGridConvergence(const cras::optional<double>& convergence)
{
  this->forcedUTMGridConvergence = this->lastUTMGridConvergence = convergence;
}

void CompassConverter::setMagneticModelPath(const cras::optional<std::string>& modelPath)
{
  this->data->magneticModelManager->setModelPath(modelPath);
}

void CompassConverter::forceMagneticModelName(const std::string& model)
{
  this->forcedMagneticModelName = model;
}

void CompassConverter::setKeepUTMZone(const bool keep)
{
  this->keepUTMZone = keep;
}

cras::expected<double, std::string> CompassConverter::getMagneticDeclination(const ros::Time& stamp) const
{
  if (this->forcedMagneticDeclination.has_value())
    return *this->forcedMagneticDeclination;

  if (!this->lastFix.has_value())
    return cras::make_unexpected("Cannot determine magnetic declination without GNSS pose.");

  return this->computeMagneticDeclination(*this->lastFix, stamp);
}

cras::expected<double, std::string> CompassConverter::computeMagneticDeclination(
  const sensor_msgs::NavSatFix& fix, const ros::Time& stamp) const
{
  const auto year = cras::getYear(stamp);
  if (this->data->magneticModels[year] == nullptr)
  {
    const auto modelName = !this->forcedMagneticModelName.empty() ?
      this->forcedMagneticModelName : this->data->magneticModelManager->getBestMagneticModelName(stamp);

    const auto model = this->data->magneticModelManager->getMagneticModel(modelName, this->strict);
    if (!model.has_value())
      return cras::make_unexpected(cras::format(
        "Could not create magnetic field model %s for year %u because of the following error: %s",
        modelName.c_str(), year, model.error().c_str()));
    this->data->magneticModels[year] = *model;
  }

  const auto& magModel = *this->data->magneticModels[year];

  const auto fieldComponents = magModel.getMagneticFieldComponents(fix, stamp);
  if (!fieldComponents.has_value())
    return cras::make_unexpected(fieldComponents.error());

  return fieldComponents->values.declination;
}

cras::expected<double, std::string> CompassConverter::getUTMGridConvergence() const
{
  if (this->forcedUTMGridConvergence.has_value())
    return *this->forcedUTMGridConvergence;

  if (!this->lastUTMGridConvergence.has_value())
    return cras::make_unexpected("UTM grid convergence has not yet been determined from GNSS pose.");

  return *this->lastUTMGridConvergence;
}

cras::expected<int, std::string> CompassConverter::getUTMZone() const
{
  if (this->forcedUTMZone.has_value())
    return *this->forcedUTMZone;

  if (!this->lastUTMZone.has_value())
    return cras::make_unexpected("UTM zone has not yet been determined from GNSS pose.");

  return *this->lastUTMZone;
}

void CompassConverter::forceUTMZone(const cras::optional<int>& zone)
{
  if (zone.has_value() && (zone < GeographicLib::UTMUPS::MINZONE || zone > GeographicLib::UTMUPS::MAXZONE))
    CRAS_WARN("Invalid UTM zone: %d", *zone);
  else
    this->forcedUTMZone = this->lastUTMZone = zone;
}

cras::expected<std::pair<double, int>, std::string> CompassConverter::computeUTMGridConvergenceAndZone(
  const sensor_msgs::NavSatFix& fix, const cras::optional<int>& utmZone) const
{
  if (utmZone.has_value() && (*utmZone < GeographicLib::UTMUPS::MINZONE || *utmZone > GeographicLib::UTMUPS::MAXZONE))
    return cras::make_unexpected(cras::format("Invalid UTM zone: %d", *utmZone));

  try
  {
    int zone;
    bool isNorthHemisphere;
    double northing, easting, utmGridConvergence, projectionScale;
    const int setzone = utmZone.value_or(GeographicLib::UTMUPS::STANDARD);

    GeographicLib::UTMUPS::Forward(fix.latitude, fix.longitude,
      zone, isNorthHemisphere, easting, northing, utmGridConvergence, projectionScale, setzone);

    return std::make_pair(angles::from_degrees(utmGridConvergence), zone);
  }
  catch (const GeographicLib::GeographicErr& e)
  {
    return cras::make_unexpected(cras::format("Could not get UTM grid convergence: %s", e.what()));
  }
}

cras::expected<compass_msgs::Azimuth, std::string> CompassConverter::convertAzimuth(
  const compass_msgs::Azimuth& azimuth,
  const decltype(compass_msgs::Azimuth::unit) unit,
  const decltype(compass_msgs::Azimuth::orientation) orientation,
  const decltype(compass_msgs::Azimuth::reference) reference) const
{
  // Fast track for no conversion
  if (azimuth.unit == unit && azimuth.orientation == orientation && azimuth.reference == reference)
    return azimuth;

  compass_msgs::Azimuth result = azimuth;
  result.unit = unit;
  result.orientation = orientation;
  result.reference = reference;

  using Az = compass_msgs::Azimuth;

  // Convert the input to NED radians
  if (azimuth.unit == Az::UNIT_DEG)
    result.azimuth = angles::from_degrees(result.azimuth);
  if (azimuth.orientation == Az::ORIENTATION_ENU)
    result.azimuth = M_PI_2 - result.azimuth;

  // When going magnetic->true, we need to add declination in NED.
  // When going true->UTM, we need to subtract grid convergence in NED.

  // Now convert between references in NED radians
  if (azimuth.reference != result.reference)
  {
    if (azimuth.reference == Az::REFERENCE_MAGNETIC)
    {
      const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);
      if (!magneticDeclination.has_value())
        return cras::make_unexpected(cras::format(
          "Cannot convert magnetic azimuth to true without knowing magnetic declination. Error: %s",
          magneticDeclination.error().c_str()));

      result.azimuth += *magneticDeclination;

      if (result.reference == Az::REFERENCE_UTM)
      {
        const auto convergence = this->getUTMGridConvergence();
        if (!convergence.has_value())
          return cras::make_unexpected(cras::format(
            "Cannot convert true azimuth to UTM without knowing UTM grid convergence. Error: %s",
            convergence.error().c_str()));

        result.azimuth -= *convergence;
      }
    }
    else if (azimuth.reference == Az::REFERENCE_GEOGRAPHIC)
    {
      if (result.reference == Az::REFERENCE_MAGNETIC)
      {
        const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);
        if (!magneticDeclination.has_value())
          return cras::make_unexpected(cras::format(
            "Cannot convert true azimuth to magnetic without knowing magnetic declination. Error: %s",
            magneticDeclination.error().c_str()));

        result.azimuth -= *magneticDeclination;
      }
      else if (result.reference == Az::REFERENCE_UTM)
      {
        const auto convergence = this->getUTMGridConvergence();
        if (!convergence.has_value())
          return cras::make_unexpected(cras::format(
            "Cannot convert true azimuth to UTM without knowing UTM grid convergence. Error: %s",
            convergence.error().c_str()));

        result.azimuth -= *convergence;
      }
    }
    else if (azimuth.reference == Az::REFERENCE_UTM)
    {
      const auto convergence = this->getUTMGridConvergence();
      if (!convergence.has_value())
        return cras::make_unexpected(cras::format(
          "Cannot convert UTM azimuth to true without knowing UTM grid convergence. Error: %s",
          convergence.error().c_str()));

      result.azimuth += *convergence;

      if (result.reference == Az::REFERENCE_MAGNETIC)
      {
        const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);
        if (!magneticDeclination.has_value())
          return cras::make_unexpected(cras::format(
            "Cannot convert true azimuth to magnetic without knowing magnetic declination. Error: %s",
            magneticDeclination.error().c_str()));

        result.azimuth -= *magneticDeclination;
      }
    }
  }

  // Reference is correct now; convert to the output unit and orientation
  if (result.orientation == Az::ORIENTATION_ENU)
    result.azimuth = M_PI_2 - result.azimuth;
  result.azimuth = angles::normalize_angle_positive(result.azimuth);
  if (result.unit == Az::UNIT_DEG)
    result.azimuth = angles::to_degrees(result.azimuth);

  if (azimuth.unit == Az::UNIT_RAD && result.unit == Az::UNIT_DEG)
    result.variance = std::pow(angles::to_degrees(std::sqrt(azimuth.variance)), 2);
  else if (azimuth.unit == Az::UNIT_DEG && result.unit == Az::UNIT_RAD)
    result.variance = std::pow(angles::from_degrees(std::sqrt(azimuth.variance)), 2);

  return result;
}

cras::expected<compass_msgs::Azimuth, std::string> CompassConverter::convertQuaternion(
  const geometry_msgs::QuaternionStamped& quat,
  const decltype(compass_msgs::Azimuth::variance) variance,
  const decltype(compass_msgs::Azimuth::unit) unit,
  const decltype(compass_msgs::Azimuth::orientation) orientation,
  const decltype(compass_msgs::Azimuth::reference) reference) const
{
  return this->convertQuaternion(quat.quaternion, quat.header, variance, unit, orientation, reference);
}

cras::expected<compass_msgs::Azimuth, std::string> CompassConverter::convertQuaternion(
  const geometry_msgs::Quaternion& quat,
  const std_msgs::Header& header,
  const decltype(compass_msgs::Azimuth::variance) variance,
  const decltype(compass_msgs::Azimuth::unit) unit,
  const decltype(compass_msgs::Azimuth::orientation) orientation,
  const decltype(compass_msgs::Azimuth::reference) reference) const
{
  tf2::Quaternion q;
  tf2::fromMsg(quat, q);
  if (q.length2() < 1e-6)
    return cras::make_unexpected("Invalid quaternion (all zeros).");

  compass_msgs::Azimuth result;
  result.header = header;
  result.azimuth = cras::getYaw(quat);
  result.variance = variance;
  if (unit == Az::UNIT_DEG)
  {
    result.azimuth = angles::to_degrees(result.azimuth);
    result.variance = std::pow(angles::to_degrees(std::sqrt(variance)), 2);
  }
  result.orientation = orientation;
  result.unit = unit;
  result.reference = reference;
  return result;
}

cras::expected<geometry_msgs::QuaternionStamped, std::string> CompassConverter::convertToQuaternion(
  const compass_msgs::Azimuth& azimuth) const
{
  tf2::Stamped<tf2::Quaternion> quat;
  quat.frame_id_ = azimuth.header.frame_id;
  quat.stamp_ = azimuth.header.stamp;
  quat.setRPY(0, 0, azimuth.azimuth * (azimuth.unit == Az::UNIT_RAD ? 1 : M_PI / 180.0));
  return tf2::toMsg(quat);
}

cras::expected<geometry_msgs::PoseWithCovarianceStamped, std::string> CompassConverter::convertToPose(
  const compass_msgs::Azimuth& azimuth) const
{
  const auto maybeQuat = this->convertToQuaternion(azimuth);
  if (!maybeQuat.has_value())
    return cras::make_unexpected(cras::format("Could not convert azimuth to pose: %s", maybeQuat.error().c_str()));

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = azimuth.header;
  pose.pose.pose.orientation = maybeQuat->quaternion;
  pose.pose.covariance[0 * 6 + 0] = std::numeric_limits<double>::infinity();
  pose.pose.covariance[1 * 6 + 1] = std::numeric_limits<double>::infinity();
  pose.pose.covariance[2 * 6 + 2] = std::numeric_limits<double>::infinity();
  pose.pose.covariance[3 * 6 + 3] = 4 * M_PI * M_PI;
  pose.pose.covariance[4 * 6 + 4] = 4 * M_PI * M_PI;
  pose.pose.covariance[5 * 6 + 5] = azimuth.variance;

  return pose;
}

cras::expected<sensor_msgs::Imu, std::string> CompassConverter::convertToImu(const compass_msgs::Azimuth& azimuth) const
{
  const auto maybeQuat = this->convertToQuaternion(azimuth);
  if (!maybeQuat.has_value())
    return cras::make_unexpected(cras::format("Could not convert azimuth to pose: %s", maybeQuat.error().c_str()));

  sensor_msgs::Imu imu;
  imu.header = azimuth.header;
  imu.linear_acceleration_covariance[0] = -1;
  imu.angular_velocity_covariance[0] = -1;
  imu.orientation = maybeQuat->quaternion;
  imu.orientation_covariance[0 * 3 + 0] = 4 * M_PI * M_PI;
  imu.orientation_covariance[1 * 3 + 1] = 4 * M_PI * M_PI;
  imu.orientation_covariance[2 * 3 + 2] = azimuth.variance;

  return imu;
}

cras::expected<compass_msgs::Azimuth, std::string> CompassConverter::convertQuaternionMsgEvent(
  const ros::MessageEvent<geometry_msgs::QuaternionStamped const>& quatEvent,
  const decltype(compass_msgs::Azimuth::variance) variance,
  const decltype(compass_msgs::Azimuth::unit) unit,
  const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation,
  const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference) const
{
  auto msgOrientation = orientation;
  auto msgReference = reference;
  if (!msgOrientation.has_value() || !msgReference.has_value())
  {
    const auto maybeAzimuthParams = parseAzimuthTopicName(quatEvent.getConnectionHeaderPtr());
    if (maybeAzimuthParams.has_value())
    {
      msgOrientation = std::get<1>(*maybeAzimuthParams);
      msgReference = std::get<2>(*maybeAzimuthParams);
    }
  }

  if (!msgOrientation.has_value() || !msgReference.has_value())
    return cras::make_unexpected("Orientation and reference are not specified and cannot be autodetected.");

  const auto msg = quatEvent.getConstMessage();
  return this->convertQuaternion(*msg, variance, unit, *msgOrientation, *msgReference);
}

cras::expected<compass_msgs::Azimuth, std::string> CompassConverter::convertPoseMsgEvent(
  const ros::MessageEvent<geometry_msgs::PoseWithCovarianceStamped const>& poseEvent,
  const decltype(compass_msgs::Azimuth::unit) unit,
  const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation,
  const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference) const
{
  auto msgOrientation = orientation;
  auto msgReference = reference;
  if (!msgOrientation.has_value() || !msgReference.has_value())
  {
    const auto maybeAzimuthParams = parseAzimuthTopicName(poseEvent.getConnectionHeaderPtr());
    if (maybeAzimuthParams.has_value())
    {
      msgOrientation = std::get<1>(*maybeAzimuthParams);
      msgReference = std::get<2>(*maybeAzimuthParams);
    }
  }

  if (!msgOrientation.has_value() || !msgReference.has_value())
    return cras::make_unexpected("Orientation and reference are not specified and cannot be autodetected.");

  const auto msg = poseEvent.getConstMessage();
  return this->convertQuaternion(
    msg->pose.pose.orientation, msg->header, msg->pose.covariance[5 * 6 + 5], unit, *msgOrientation, *msgReference);
}

cras::expected<compass_msgs::Azimuth, std::string> CompassConverter::convertImuMsgEvent(
  const ros::MessageEvent<sensor_msgs::Imu const>& imuEvent,
  const decltype(compass_msgs::Azimuth::unit) unit,
  const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation,
  const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference) const
{
  auto msgOrientation = orientation;
  auto msgReference = reference;
  if (!msgOrientation.has_value() || !msgReference.has_value())
  {
    const auto maybeAzimuthParams = parseAzimuthTopicName(imuEvent.getConnectionHeaderPtr());
    if (maybeAzimuthParams.has_value())
    {
      msgOrientation = std::get<1>(*maybeAzimuthParams);
      msgReference = std::get<2>(*maybeAzimuthParams);
    }
  }

  // IMUs should output orientation in ENU frame
  if (!msgOrientation.has_value())
    msgOrientation = Az::ORIENTATION_ENU;

  if (!msgReference.has_value())
    return cras::make_unexpected("Reference is not specified and cannot be autodetected.");

  const auto msg = imuEvent.getConstMessage();
  return this->convertQuaternion(
    msg->orientation, msg->header, msg->orientation_covariance[2 * 3 + 2], unit, *msgOrientation, *msgReference);
}

template<class M> using ME = ros::MessageEvent<M>;
template<class M> using Creator = ros::DefaultMessageCreator<M>;

cras::expected<compass_msgs::Azimuth, std::string> CompassConverter::convertUniversalMsgEvent(
  const ros::MessageEvent<topic_tools::ShapeShifter const>& event,
  const decltype(compass_msgs::Azimuth::variance) variance,
  const decltype(compass_msgs::Azimuth::unit) unit,
  const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation,
  const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference) const
{
  const auto msg = event.getConstMessage();
  const auto header = event.getConnectionHeaderPtr();
  const auto stamp = event.getReceiptTime();
  const auto& type = msg->getDataType();
  namespace mt = ros::message_traits;

  if (type == mt::datatype<Az>())
  {
    const auto az = *msg->instantiate<Az>();
    return this->convertAzimuth(az, unit, az.orientation, az.reference);
  }
  else if (type == mt::datatype<Pose>())
  {
    const auto poseEvent = ME<Pose const>(msg->instantiate<Pose>(), header, stamp, false, Creator<Pose>());
    return this->convertPoseMsgEvent(poseEvent, unit, orientation, reference);
  }
  else if (type == mt::datatype<Quat>())
  {
    const auto quatEvent = ME<Quat const>(msg->instantiate<Quat>(), header, stamp, false, Creator<Quat>());
    return this->convertQuaternionMsgEvent(quatEvent, variance, unit, orientation, reference);
  }
  else if (type == mt::datatype<Imu>())
  {
    const auto imuEvent = ME<Imu const>(msg->instantiate<Imu>(), header, stamp, false, Creator<Imu>());
    return this->convertImuMsgEvent(imuEvent, unit, orientation, reference);
  }
  else
    return cras::make_unexpected(cras::format("Invalid message type: %s.", type.c_str()));
}

void CompassConverter::setNavSatPos(const sensor_msgs::NavSatFix& fix)
{
  this->lastFix = fix;

  if (!this->forcedUTMGridConvergence.has_value())
  {
    const auto maybeConvergenceAndZone = this->computeUTMGridConvergenceAndZone(fix, this->forcedUTMZone);
    if (!maybeConvergenceAndZone.has_value())
    {
      CRAS_WARN_THROTTLE(10.0, "Error computing UTM grid convergence: %s", maybeConvergenceAndZone.error().c_str());
    }
    else
    {
      const auto [convergence, zone] = *maybeConvergenceAndZone;

      this->lastUTMZone = zone;
      if (this->keepUTMZone && !this->forcedUTMZone.has_value())
        this->forcedUTMZone = zone;

      this->lastUTMGridConvergence = convergence;
    }
  }
}

}
