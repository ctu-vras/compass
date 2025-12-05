#pragma once
// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert magnetometer and IMU measurements to azimuth.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <string>

#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/expected.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/buffer.hpp>

namespace magnetometer_compass
{

struct MagnetometerCompassPrivate;

/**
 * \brief Convert magnetometer and IMU measurements to azimuth.
 */
class MagnetometerCompass
{
public:
  using NodeClockInterface = rclcpp::node_interfaces::NodeClockInterface;
  using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;
  using NodeParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;

  using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    NodeClockInterface,
    NodeLoggingInterface,
    NodeParametersInterface
  >;

  /**
   * \brief Create the compass.
   * \param[in] node The node to use.
   * \param[in] frame The target frame in which the azimuth is expressed. Its Z axis should approx. point upwards.
   *                  Azimuth is the angle between magnetic North and this frame's X axis.
   * \param[in] tf TF buffer for transforming incoming data to `frame`. If you are sure data are already in the target
   *               frame, you can pass an empty buffer.
   */
  MagnetometerCompass(RequiredInterfaces node, const std::string& frame, const std::shared_ptr<tf2_ros::Buffer>& tf);

  virtual ~MagnetometerCompass();

  /**
   * \brief Configure the bias remover from ROS parameters.
   *
   * The following parameters are read:
   * - `~initial_variance` (double, default 0): Variance of the measurement used at startup (in rad^2).
   * - `~low_pass_ratio` (double, default 0.95): The azimuth is filtered with a low-pass filter. This sets its
   *                                             aggressivity (0 means raw measurements, 1 means no updates).
   */
  virtual void configFromParams();

  geometry_msgs::msg::Quaternion getRotationBetweenFrames(const sensor_msgs::msg::Imu& imu_msg);

  /**
   * \brief The azimuth is filtered with a low-pass filter. This sets its aggressivity.
   * \param[in] ratio The ratio (0 means raw measurements, 1 means no updates).
   */
  virtual void setLowPassRatio(double ratio);

  /**
   * \brief Compute azimuth from the provided IMU and magnetometer measurements.
   * \param[in] imu IMU tied to the magnetometer. It has to contain valid orientation.
   * \param[in] magUnbiased Magnetometer measurement with removed bias.
   * \return The computed azimuth, or an error message. The azimuth will be in radians and NED frame.
   * \note The function does not check time synchronization of the two inputs.
   * \note Both inputs have to be transformable to the configured target frame.
   */
  virtual cras::expected<compass_interfaces::msg::Azimuth, std::string> computeAzimuth(
    const sensor_msgs::msg::Imu& imu, const sensor_msgs::msg::MagneticField& magUnbiased);

  /**
   * \brief Reset the computation (i.e. the low-pass filter and estimated variance).
   */
  virtual void reset();

protected:
  /**
   * \brief Re-estimate variance after adding a new measurement.
   * \note This is not yet implemented.
   */
  virtual void updateVariance();

private:
  std::unique_ptr<MagnetometerCompassPrivate> data;  //!< PIMPL
  RequiredInterfaces node;
  geometry_msgs::msg::Quaternion last_imu_orientation {};
};

}
