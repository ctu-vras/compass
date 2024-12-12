// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Remove known bias from 3-axis magnetometer.
 * \author Martin Pecka
 */

#include <memory>

#include <cras_cpp_common/nodelet_utils.hpp>
#include <magnetometer_pipeline/message_filter.h>
#include <message_filters/subscriber.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

namespace magnetometer_pipeline
{
using Field = sensor_msgs::MagneticField;

/**
 * \brief Remove known bias from 3-axis magnetometer.
 *
 * For the magnetometer to work correctly, it is required to measure its bias. This node listens on the `imu/mag_bias`
 * topic for this measurement, and until at least one message arrives, the node will not publish anything. If you do not
 * have a node publishing the bias, you can alternatively provide it via parameters. Depending on the application, it
 * may be required to re-estimate the bias from time to time even during runtime.
 * 
 * Subscribed topics:
 * - `imu/mag` (`sensor_msgs/MagneticField`): 3-axis magnetometer measurements (bias not removed).
 * - `imu/mag_bias` (`sensor_msgs/MagneticField`): Bias of the magnetometer. This value will be subtracted from the
 *                                                 incoming magnetometer measurements. Messages on this topic do not
 *                                                 need to come repeatedly if the bias does not change. The
 *                                                 `magnetic_field_covariance` field can be "misused" to carry a 3x3
 *                                                 bias scaling matrix.
 *
 * Published topics (see above for explanation):
 * - `imu/mag_unbiased` (`sensor_msgs/MagneticField`): The magnetic field measurement with bias removed.
 *
 * Parameters:
 * - `~initial_mag_bias_x` (double, no default, optional): Magnetometer bias in the X axis.
 * - `~initial_mag_bias_y` (double, no default, optional): Magnetometer bias in the Y axis.
 * - `~initial_mag_bias_z` (double, no default, optional): Magnetometer bias in the Z axis.
 * - `~initial_scaling_matrix` (double[9], optional): Magnetometer scaling matrix (row-major).
 *   - If you specify any of the `~initial_mag_bias_*` params, the node does not need to receive the bias messages.
 */
class MagnetometerBiasRemoverNodelet : public cras::Nodelet
{
public:
  MagnetometerBiasRemoverNodelet();
  ~MagnetometerBiasRemoverNodelet() override;

protected:
  void onInit() override;

  std::unique_ptr<BiasRemoverFilter> remover;  //!< \brief The bias remover doing the actual work.

  std::unique_ptr<message_filters::Subscriber<Field>> magSub;  //!< \brief Subscriber for magnetic field measurements.
  std::unique_ptr<message_filters::Subscriber<Field>> magBiasSub;  //!< \brief Subscriber for bias.

  ros::Publisher magUnbiasedPub;  //!< \brief Publisher of unbiased measurements.
};

MagnetometerBiasRemoverNodelet::MagnetometerBiasRemoverNodelet() = default;

MagnetometerBiasRemoverNodelet::~MagnetometerBiasRemoverNodelet() = default;

void MagnetometerBiasRemoverNodelet::onInit()
{
  cras::Nodelet::onInit();

  auto nh = this->getNodeHandle();
  auto topicNh = ros::NodeHandle(nh, "imu");
  auto params = this->privateParams();

  this->magUnbiasedPub = topicNh.advertise<Field>("mag_unbiased", 10);

  this->magSub = std::make_unique<message_filters::Subscriber<Field>>(topicNh, "mag", 100);
  this->magBiasSub = std::make_unique<message_filters::Subscriber<Field>>(topicNh, "mag_bias", 10);

  this->remover = std::make_unique<BiasRemoverFilter>(this->log, *this->magSub, *this->magBiasSub);
  this->remover->configFromParams(*params);
  this->remover->registerCallback([this](const Field::ConstPtr& msg) {this->magUnbiasedPub.publish(msg);});
}

}

PLUGINLIB_EXPORT_CLASS(magnetometer_pipeline::MagnetometerBiasRemoverNodelet, nodelet::Nodelet)
