#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert between various compass representations.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <utility>

#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/message_event.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Header.h>
#include <topic_tools/shape_shifter.h>

namespace compass_conversions
{

struct CompassConverterPrivate;

/**
 * \brief Convert between various compass representations.
 *
 * When converting only units or orientations, no NavSatFix is needed.
 * However, if you need to convert between different references, you need to provide a NavSatFix with valid GNSS
 * position and time and set it using setNavSatPos().
 */
class CompassConverter : public cras::HasLogger
{
public:
  /**
   * \brief Create the compass converter.
   *
   * \param[in] log The logger.
   * \param[in] strict Whether to fail if the magnetic model is used outside its natural validity bounds.
   */
  CompassConverter(const cras::LogHelperPtr& log, bool strict);
  virtual ~CompassConverter();

  /**
   * \brief Configure the compass converter from the given ROS parameters.
   *
   * \param[in] params ROS parameters that configure the converter.
   *
   * The parameters read from params struct are:
   * - `magnetic_declination` (double, radians, optional): If set, forces this value of magnetic declination.
   * - `utm_grid_convergence` (double, radians, optional): If set, forces this value of UTM grid convergence.
   * - `magnetic_models_path` (string, default "$PACKAGE/data/magnetic"): Path where WMM magnetic models can be found.
   *     If set to empty string, the models will be searched in a default folder of GeographicLib. Environment variables
   *     `GEOGRAPHICLIB_MAGNETIC_PATH` or `GEOGRAPHICLIB_DATA` influence the location of this folder.
   * - `magnetic_model` (string, optional): If set, forces using the given WMM model instead of determining the proper
   *                                        one by year. Example value is "wmm2020".
   * - `utm_zone` (int, optional): If set, forces using this UTM zone instead of determining the proper one.
   * - `keep_utm_zone` (bool, default true): If true, the first determined UTM zone will be used for all future
   *                                         conversions.
   * - `initial_lat` (double, degrees, optional): If set, use this latitude before the first navsat pose is received.
   * - `initial_lon` (double, degrees, optional): If set, use this longitude before the first navsat pose is received.
   * - `initial_alt` (double, meters, optional): If set, use this altitude before the first navsat pose is received.
   */
  void configFromParams(const cras::BoundParamHelper& params);

  /**
   * \brief Force magnetic declination instead of computing it.
   * \param[in] declination The forced declination in rad. If nullopt, declination is set to be computed.
   */
  virtual void forceMagneticDeclination(const cras::optional<double>& declination);

  /**
   * \brief Force UTM grid convergence instead of computing it.
   * \param[in] convergence The forced convergence in rad. If nullopt, convergence is set to be computed.
   */
  virtual void forceUTMGridConvergence(const cras::optional<double>& convergence);

  /**
   * \brief Set the path where magnetic models are stored.
   * \param[in] modelPath Path to the folder with stored models. If nullopt, the default data distributed with this
   *                      package will be used. If empty string, a default system location will be used. The default
   *                      system location is determined by GeographicLib and can be influenced by setting environment
   *                      variables `GEOGRAPHICLIB_MAGNETIC_PATH` or `GEOGRAPHICLIB_DATA`.
   */
  virtual void setMagneticModelPath(const cras::optional<std::string>& modelPath);

  /**
   * \brief Force using the given magnetic model instead of automatically selecting the best one.
   * \param[in] model The name of the model to force (e.g. "wmm2020"). If empty, the best model will be autoselected.
   */
  virtual void forceMagneticModelName(const std::string& model);

  /**
   * \brief Set whether UTM zone should be kept after it is first computed.
   * \param[in] keep If true, the next call to setNavSatPos() will store the computed UTM zone to forcedUTMZone.
   */
  virtual void setKeepUTMZone(bool keep);

  /**
   * \brief Callback for GNSS fix (so that the converter can compute UTM grid convergence and store the pose).
   * \param[in] fix The fix message. Only `latitude`, `longitude`, `altitude` and `header.stamp` are relevant.
   * \note This function computes lastUTMGridConvergence if forcedUTMGridConvergence is not set.
   * \note This function computes lastUTMZone if forcedUTMGridConvergence is not set.
   * \note If keepUTMZone is set and forcedUTMZone is not set, the first determined zone will be remembered
   *       in forcedUTMZone.
   */
  virtual void setNavSatPos(const sensor_msgs::NavSatFix& fix);

  /**
   * \brief Get the value of magnetic declination for the last location received by setNatSatPos().
   * \param[in] stamp The time for which declination is queried.
   * \return The magnetic declination in radians or an error message.
   * \note If forcedMagneticDeclination is set, it is returned without any adjustments.
   */
  virtual cras::expected<double, std::string> getMagneticDeclination(const ros::Time& stamp) const;

  /**
   * \brief Compute magnetic declination for the given position and time.
   * \param[in] fix The position to compute declination for.
   * \param[in] stamp The time to compute declination for.
   * \return The magnetic declination in radians or an error message.
   * \note This function does not take forcedMagneticDeclination into account.
   */
  virtual cras::expected<double, std::string> computeMagneticDeclination(
    const sensor_msgs::NavSatFix& fix, const ros::Time& stamp) const;

  /**
   * \brief Get the value of UTM grid convergence for the last location received by setNatSatPos().
   * \return The UTM grid convergence in radians, or an error message.
   * \note If forcedUTMGridConvergence is set, it is returned without any adjustments. Otherwise, lastUTMGridConvergence
   *       is returned.
   * \note If forcedUTMZone is set, the returned grid convergence will be in this zone. Otherwise, it will be in
   *       lastUTMZone.
   */
  virtual cras::expected<double, std::string> getUTMGridConvergence() const;

  /**
   * \brief Get the UTM zone of the last location received by setNatSatPos().
   * \return The UTM zone, or an error message.
   * \note If forcedUTMZone is set, it will be directly returned.
   */
  virtual cras::expected<int, std::string> getUTMZone() const;

  /**
   * \brief Set the UTM zone that will be used for all future UTM operations.
   * \param[in] zone The UTM zone. If nullopt, UTM operations will use the default zone.
   * \note This function sets forcedUTMZone.
   */
  virtual void forceUTMZone(const cras::optional<int>& zone);

  /**
   * \brief Get the value of UTM grid convergence and UTM zone for the provided place.
   * \param[in] fix The place for which grid convergence is queried.
   * \param[in] utmZone Optional forced UTM zone. If not specified, the default UTM zone will be used.
   * \return The UTM grid convergence in radians and corresponding UTM zone, or an error message.
   * \note This function does not take forcedUTMGridConvergence into account.
   */
  virtual cras::expected<std::pair<double, int>, std::string> computeUTMGridConvergenceAndZone(
    const sensor_msgs::NavSatFix& fix, const cras::optional<int>& utmZone) const;

  /**
   * \brief Convert the given compass_msgs::Azimuth message parametrized by the given unit, orientation and reference.
   *
   * \param[in] azimuth The input azimuth message.
   * \param[in] unit The output azimuth units.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   * \return The converted Azimuth message or an error message.
   *
   * \note To convert between different references, setNavSatPos() has to be called prior to calling this function.
   *       The declination and grid convergence of the last set navsat pose will be used.
   */
  virtual cras::expected<compass_msgs::Azimuth, std::string> convertAzimuth(
    const compass_msgs::Azimuth& azimuth,
    decltype(compass_msgs::Azimuth::unit) unit,
    decltype(compass_msgs::Azimuth::orientation) orientation,
    decltype(compass_msgs::Azimuth::reference) reference) const;

  /**
   * \brief Convert the given geometry_msgs::QuaternionStamped message to Azimuth parametrized by the given unit,
   * orientation and reference.
   *
   * \param[in] quat The input quaternion message.
   * \param[in] variance Variance of the measurement (in rad^2).
   * \param[in] unit The output azimuth units.
   * \param[in] orientation The output azimuth orientation (this is just a declaration, no conversion is done).
   * \param[in] reference The output azimuth reference (this is just a declaration, no conversion is done).
   * \return The converted Azimuth message or an error message.
   */
  virtual cras::expected<compass_msgs::Azimuth, std::string> convertQuaternion(
    const geometry_msgs::QuaternionStamped& quat,
    decltype(compass_msgs::Azimuth::variance) variance,
    decltype(compass_msgs::Azimuth::unit) unit,
    decltype(compass_msgs::Azimuth::orientation) orientation,
    decltype(compass_msgs::Azimuth::reference) reference) const;

  /**
   * \brief Convert the given geometry_msgs::Quaternion message to Azimuth parametrized by the given unit,
   * orientation and reference.
   *
   * \param[in] quat The input quaternion message.
   * \param[in] header Header of the output message.
   * \param[in] variance Variance of the measurement (in rad^2).
   * \param[in] unit The output azimuth units.
   * \param[in] orientation The output azimuth orientation (this is just a declaration, no conversion is done).
   * \param[in] reference The output azimuth reference (this is just a declaration, no conversion is done).
   * \return The converted Azimuth message or an error message.
   */
  virtual cras::expected<compass_msgs::Azimuth, std::string> convertQuaternion(
    const geometry_msgs::Quaternion& quat,
    const std_msgs::Header& header,
    decltype(compass_msgs::Azimuth::variance) variance,
    decltype(compass_msgs::Azimuth::unit) unit,
    decltype(compass_msgs::Azimuth::orientation) orientation,
    decltype(compass_msgs::Azimuth::reference) reference) const;

  /**
   * \brief Convert a received geometry_msgs::QuaternionStamped message to Azimuth. If needed, extract the orientation
   *        and reference from the topic name.
   *
   * \param[in] quatEvent The input quaternion message.
   * \param[in] variance Variance of the measurement (in rad^2).
   * \param[in] unit The output azimuth units.
   * \param[in] orientation The declared input orientation (autodetected from topic if not specified).
   * \param[in] reference The declared input reference (autodetected from topic if not specified).
   * \return The converted Azimuth message or an error message.
   */
  cras::expected<compass_msgs::Azimuth, std::string> convertQuaternionMsgEvent(
    const ros::MessageEvent<geometry_msgs::QuaternionStamped const>& quatEvent,
    decltype(compass_msgs::Azimuth::variance) variance = 0,
    decltype(compass_msgs::Azimuth::unit) unit = compass_msgs::Azimuth::UNIT_RAD,
    const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation = {},
    const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference = {}) const;

  /**
   * \brief Convert a received geometry_msgs::PoseWithCovarianceStamped message to Azimuth. If needed, extract the
   *        orientation and reference from the topic name.
   *
   * \param[in] poseEvent The input pose message.
   * \param[in] unit The output azimuth units.
   * \param[in] orientation The declared input orientation (autodetected from topic if not specified).
   * \param[in] reference The declared input reference (autodetected from topic if not specified).
   * \return The converted Azimuth message or an error message.
   */
  cras::expected<compass_msgs::Azimuth, std::string> convertPoseMsgEvent(
    const ros::MessageEvent<geometry_msgs::PoseWithCovarianceStamped const>& poseEvent,
    decltype(compass_msgs::Azimuth::unit) unit = compass_msgs::Azimuth::UNIT_RAD,
    const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation = {},
    const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference = {}) const;

  /**
   * \brief Convert a received sensor_msgs::Imu message to Azimuth. If needed, extract the orientation and reference
   *        from the topic name.
   *
   * \param[in] imuEvent The input IMU message.
   * \param[in] unit The output azimuth units.
   * \param[in] orientation The declared input orientation (autodetected from topic if not specified).
   * \param[in] reference The declared input reference (autodetected from topic if not specified).
   * \return The converted Azimuth message or an error message.
   */
  cras::expected<compass_msgs::Azimuth, std::string> convertImuMsgEvent(
    const ros::MessageEvent<sensor_msgs::Imu const>& imuEvent,
    decltype(compass_msgs::Azimuth::unit) unit = compass_msgs::Azimuth::UNIT_RAD,
    const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation = {},
    const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference = {}) const;

  /**
   * \brief Convert a received message to Azimuth. If needed, extract the orientation and reference from the topic name.
   *
   * Supported message types are compass_msgs::Azimuth, geometry_msgs::QuaternionStamped,
   * geometry_msgs::PoseWithCovarianceStamped and sensor_msgs::Imu.
   *
   * \param[in] event The input message.
   * \param[in] variance Variance of the measurement (in rad^2) (if it isn't a part of the message).
   * \param[in] unit The output azimuth units.
   * \param[in] orientation The declared input orientation (autodetected from topic if not specified).
   * \param[in] reference The declared input reference (autodetected from topic if not specified).
   * \return The converted Azimuth message or an error message.
   */
  cras::expected<compass_msgs::Azimuth, std::string> convertUniversalMsgEvent(
    const ros::MessageEvent<topic_tools::ShapeShifter const>& event,
    decltype(compass_msgs::Azimuth::variance) variance = 0,
    decltype(compass_msgs::Azimuth::unit) unit = compass_msgs::Azimuth::UNIT_RAD,
    const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation = {},
    const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference = {}) const;

  /**
   * \brief Convert the given Azimuth message to geometry_msgs::QuaternionStamped in the same parametrization.
   *
   * \param[in] azimuth The input azimuth message.
   * \return The converted geometry_msgs::QuaternionStamped message or an error message.
   */
  virtual cras::expected<geometry_msgs::QuaternionStamped, std::string> convertToQuaternion(
    const compass_msgs::Azimuth& azimuth) const;

  /**
   * \brief Convert the given Azimuth message to geometry_msgs::PoseWithCovarianceStamped in the same parametrization.
   *
   * \param[in] azimuth The input azimuth message.
   * \return The converted geometry_msgs::PoseWithCovarianceStamped message or an error message.
   */
  virtual cras::expected<geometry_msgs::PoseWithCovarianceStamped, std::string> convertToPose(
    const compass_msgs::Azimuth& azimuth) const;

  /**
   * \brief Convert the given Azimuth message to sensor_msgs::Imu in the same parametrization.
   *
   * \param[in] azimuth The input azimuth message.
   * \return The converted sensor_msgs::Imu message or an error message.
   */
  virtual cras::expected<sensor_msgs::Imu, std::string> convertToImu(const compass_msgs::Azimuth& azimuth) const;

protected:
  //! \brief UTM convergence of the last received navsat position (or the forced one).
  cras::optional<double> lastUTMGridConvergence;

  //! \brief Last determined UTM zone. If empty, no zone has been determined yet.
  cras::optional<int> lastUTMZone;

  //! \brief The user-forced magnetic declination (if set, do not compute it).
  cras::optional<double> forcedMagneticDeclination;

  //! \brief The user-forced UTM grid convergence (if set, do not compute it).
  cras::optional<double> forcedUTMGridConvergence;

  //! \brief The user-forced UTM zone (if set, do not compute it).
  cras::optional<int> forcedUTMZone;

  //! \brief If true, the first determined UTM zone will be kept for all future queries.
  bool keepUTMZone {true};

  //! \brief If the user forces a magnetic model, this is its name.
  std::string forcedMagneticModelName{};

  //! \brief If true, convertAzimuth() will fail when the magnetic model is used outside its bounds.
  bool strict {true};

  //! \brief Last received GNSS fix. Used for determining magnetic declination and UTM grid convergence.
  cras::optional<sensor_msgs::NavSatFix> lastFix;

  //! \brief PIMPL data
  std::unique_ptr<CompassConverterPrivate> data;
};

}
