#pragma once
// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to remove bias from 3-axis magnetometer measurements.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>

#include <magnetometer_pipeline/bias_remover.hpp>
#include <message_filters/connection.hpp>
#include <message_filters/message_event.hpp>
#include <message_filters/simple_filter.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace magnetometer_pipeline
{

/**
 * \brief Message filter to remove bias from 3-axis magnetometer measurements.
 *
 * \sa https://wiki.ros.org/message_filters
 *
 * Example usage:
 * \code{.cpp}
 * std::shared_ptr<rclcpp::Node> node = createNodelet();
 * message_filters::Subscriber<sensor_msgs::msg::MagneticField> magInput =
 *   node->create_subscription<sensor_msgs::msg::MagneticField>("imu/mag", 100);
 * message_filters::Subscriber<sensor_msgs::msg::MagneticField> biasInput =
 *   node->create_subscription<sensor_msgs::msg::MagneticField>("imu/mag_bias", 10);
 * magnetometer_pipeline::BiasRemoverFilter filter(*node, magInput, biasInput);
 * filter->configFromParams();
 * filter.registerCallback([](const sensor_msgs::msg::MagneticField::ConstSharedPtr& unbiasedMsg) {
 *   ...  // Handle the unbiased data
 * });
 * \endcode
 */
class BiasRemoverFilter : public message_filters::SimpleFilter<sensor_msgs::msg::MagneticField>
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
   * \brief Construct azimuth filter that can convert all parameters.
   *
   * \tparam MagInput The type of the input filter.
   * \tparam BiasInput The type of the bias filter.
   * \param[in] node The node to use.
   * \param[in] magInput The message filter producing raw magnetometer measurements messages.
   * \param[in] biasInput The message filter producing magnetometer bias messages.
   */
  template<class MagInput, class BiasInput>
  BiasRemoverFilter(RequiredInterfaces node, MagInput& magInput, BiasInput& biasInput) : node(node)
  {
    this->remover = std::make_unique<MagnetometerBiasRemover>(node);
    this->connectMagnetometerInput(magInput);
    this->connectBiasInput(biasInput);
  }

  virtual ~BiasRemoverFilter();

  template<class MagInput>
  void connectMagnetometerInput(MagInput& f)
  {
    this->magConnection.disconnect();
    this->magConnection = f.registerCallback(&BiasRemoverFilter::cbMag, this);
  }

  template<class BiasInput>
  void connectBiasInput(BiasInput& f)
  {
    this->biasConnection.disconnect();
    this->biasConnection = f.registerCallback(&BiasRemoverFilter::cbBias, this);
    // Bias can be a latched message, so we could miss the only message sent there. Resubscribe to be sure we get it.
    f.subscribe();
  }

  /**
   * \brief Configure the bias removal process from ROS parameters.
   *
   * The following parameters are read:
   * - `~initial_mag_bias_x` (double, no default, optional): Magnetometer bias in the X axis.
   * - `~initial_mag_bias_y` (double, no default, optional): Magnetometer bias in the Y axis.
   * - `~initial_mag_bias_z` (double, no default, optional): Magnetometer bias in the Z axis.
   * - `~initial_scaling_matrix` (double[9], optional): Magnetometer scaling matrix (row-major).
   */
  virtual void configFromParams();

protected:
  virtual void cbMag(const message_filters::MessageEvent<sensor_msgs::msg::MagneticField const>& event);
  virtual void cbBias(const message_filters::MessageEvent<sensor_msgs::msg::MagneticField const>& event);

  message_filters::Connection magConnection;  //!< Connection to the magnetometer measurements input.
  message_filters::Connection biasConnection;  //!< Connection to the bias input.

  std::unique_ptr<MagnetometerBiasRemover> remover;  //!< The bias remover that does the actual computations.
  RequiredInterfaces node;
};

}
