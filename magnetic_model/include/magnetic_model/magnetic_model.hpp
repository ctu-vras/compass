#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Earth magnetic field model.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace magnetic_model
{

/**
 * \brief Properties of a local magnetic field vector.
 */
struct MagneticField
{
  sensor_msgs::msg::MagneticField field;  //!< The computed magnetic field vector in ENU convention [T].
  geometry_msgs::msg::Vector3 dt;  //!< Time derivative of the field [T/year].
  geometry_msgs::msg::Vector3 error;  //!< Standard deviations of the magnetic field vector components.
};

/**
 * \brief Components into which a 3D magnetic vector can be decomposed.
 */
struct MagneticFieldComponents
{
  double horizontalMagnitude;  //!< Magnitude of the horizontal component [T].
  double totalMagnitude;  //!< Total magnitude of the vector [T].
  double declination;  //!< Declination of the vector (angle from North towards East) [rad].
  double inclination;  //!< Inclination of the vector (angle from horizontal direction downwards) [rad].
};

/**
 * \brief Magnetic field components together with their time derivatives and error terms.
 */
struct MagneticFieldComponentProperties
{
  MagneticFieldComponents values;  //!< Values of the components [T or rad].
  MagneticFieldComponents dt;  //!< Time derivatives of the components [T/year or rad/year].
  MagneticFieldComponents errors;  //!< Standard deviations of the components [T or rad].
};

struct MagneticModelPrivate;

/**
 * \brief Earth magnetic model.
 *
 * The GAZEBO model is a special instance of IGRF-14 on 22/01/2018, which corresponds to the
 * baked-in data table used by the Gazebo Magnetometer system.
 */
class MagneticModel
{
public:
  using NodeClockInterface = rclcpp::node_interfaces::NodeClockInterface;
  using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;

  using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    NodeClockInterface,
    NodeLoggingInterface
  >;

  static const char* GAZEBO;  //!< Gazebo model name
  static const char* IGRF14;  //!< IGRF-14 model name
  static const char* WMM2010;  //!< WMM 2010 model name
  static const char* WMM2015;  //!< WMM 2015v2 model name
  static const char* WMM2020;  //!< WMM 2020 model name
  static const char* WMM2025;  //!< WMM 2025 model name

  /**
   * \brief Create the magnetic model.
   *
   * \param[in] node The node to use.
   * \param[in] name Name of the model (e.g. "wmm2020").
   * \param[in] modelPath Path to the folder with stored models. If empty, a default system location will be used.
   * \param[in] strict Whether to fail if the magnetic model is used outside its natural validity bounds.
   */
  MagneticModel(RequiredInterfaces node, const std::string& name, const std::string& modelPath, bool strict);
  virtual ~MagneticModel();

  /**
   * \brief Tell whether this model is valid at the given time instant.
   * \param[in] time The time of validity.
   * \return Whether the model is valid (regardless of strictness).
   */
  virtual bool isValid(const rclcpp::Time& time) const;

  /**
   * \brief Tell whether this model is valid at the given year.
   * \param[in] year The year of validity.
   * \return Whether the model is valid (regardless of strictness).
   */
  virtual bool isValid(int year) const;

  /**
   * \brief Get the magnetic field vector on the provided place on Earth.
   * \param[in] fixMsg The place for which magnetic field is queried. Timestamp from the message is ignored.
   * \param[in] stampIn The time for which magnetic field is queried.
   * \return The magnetic field.
   */
  virtual cras::expected<MagneticField, std::string> getMagneticField(
    const sensor_msgs::msg::NavSatFix& fixMsg, const rclcpp::Time& stampIn) const;

  /**
   * \brief Get the magnetic field components on the provided place on Earth.
   * \param[in] fixMsg The place for which magnetic field components are queried. Timestamp from the message is ignored.
   * \param[in] stampIn The time for which magnetic field components are queried.
   * \return The magnetic field components.
   */
  virtual cras::expected<MagneticFieldComponentProperties, std::string> getMagneticFieldComponents(
    const sensor_msgs::msg::NavSatFix& fixMsg, const rclcpp::Time& stampIn) const;

  /**
   * \brief Get the components of the measured magnetic field.
   * \param[in] field The measured magnetic field.
   * \param[in] stampIn The time for which magnetic field components are queried.
   * \return The magnetic field components.
   */
  virtual cras::expected<MagneticFieldComponentProperties, std::string> getMagneticFieldComponents(
    const MagneticField& field, const rclcpp::Time& stampIn) const;

protected:
  //! \brief If true, the magnetic model will fail if it is used outside its bounds.
  bool strict {true};

  //! \brief PIMPL data
  std::unique_ptr<MagneticModelPrivate> data;
  RequiredInterfaces node;
};

}
