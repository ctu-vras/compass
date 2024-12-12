#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to remove bias from 3-axis magnetometer measurements.
 * \author Martin Pecka
 */

#include <memory>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/log_utils.h>
#include <magnetometer_pipeline/bias_remover.h>
#include <message_filters/connection.h>
#include <message_filters/simple_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/MagneticField.h>

namespace magnetometer_pipeline
{

/**
 * \brief Message filter to remove bias from 3-axis magnetometer measurements.
 *
 * \sa https://wiki.ros.org/message_filters
 *
 * Example usage:
 * \code{.cpp}
 * ros::NodeHandle nh;
 * message_filters::Subscriber<sensor_msgs::MagneticField> magInput(nh, "imu/mag", 100);
 * message_filters::Subscriber<sensor_msgs::MagneticField> biasInput(nh, "imu/mag_bias", 10);
 * magnetometer_pipeline::BiasRemoverFilter filter(log, magInput, biasInput);
 * // filter->configFromParams(params);
 * filter.registerCallback([](const sensor_msgs::MagneticField::ConstPtr& unbiasedMsg) {
 *   ...  // Handle the unbiased data
 * });
 * \endcode
 */
class BiasRemoverFilter : public message_filters::SimpleFilter<sensor_msgs::MagneticField>, public cras::HasLogger
{
public:
  /**
   * \brief Construct azimuth filter that can convert all parameters.
   *
   * \tparam MagInput The type of the input filter.
   * \tparam BiasInput The type of the bias filter.
   * \param[in] log Logger.
   * \param[in] magInput The message filter producing raw magnetometer measurements messages.
   * \param[in] biasInput The message filter producing magnetometer bias messages.
   */
  template<class MagInput, class BiasInput>
  BiasRemoverFilter(const cras::LogHelperPtr& log, MagInput& magInput, BiasInput& biasInput) : HasLogger(log)
  {
    this->remover = std::make_unique<MagnetometerBiasRemover>(log);
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
   * \param[in] params The parameters.
   *
   * The following parameters are read:
   * - `~initial_mag_bias_x` (double, no default, optional): Magnetometer bias in the X axis.
   * - `~initial_mag_bias_y` (double, no default, optional): Magnetometer bias in the Y axis.
   * - `~initial_mag_bias_z` (double, no default, optional): Magnetometer bias in the Z axis.
   * - `~initial_scaling_matrix` (double[9], optional): Magnetometer scaling matrix (row-major).
   */
  virtual void configFromParams(const cras::BoundParamHelper& params);

protected:
  virtual void cbMag(const ros::MessageEvent<sensor_msgs::MagneticField const>& event);
  virtual void cbBias(const ros::MessageEvent<sensor_msgs::MagneticField const>& event);

  message_filters::Connection magConnection;  //!< Connection to the magnetometer measurements input.
  message_filters::Connection biasConnection;  //!< Connection to the bias input.

  std::unique_ptr<MagnetometerBiasRemover> remover;  //!< The bias remover that does the actual computations.
};

}
