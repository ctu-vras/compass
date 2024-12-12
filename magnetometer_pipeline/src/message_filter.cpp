// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to remove bias from 3-axis magnetometer measurements.
 * \author Martin Pecka
 */

#include <boost/make_shared.hpp>

#include <cras_cpp_common/log_utils.h>
#include <magnetometer_pipeline/message_filter.h>
#include <ros/message_event.h>
#include <sensor_msgs/MagneticField.h>

namespace magnetometer_pipeline
{

using Field = sensor_msgs::MagneticField;

BiasRemoverFilter::~BiasRemoverFilter() = default;

void BiasRemoverFilter::configFromParams(const cras::BoundParamHelper& params)
{
  this->remover->configFromParams(params);
}

void BiasRemoverFilter::cbMag(const ros::MessageEvent<Field const>& event)
{
  const auto maybeMagUnbiased = this->remover->removeBias(*event.getConstMessage());
  if (!maybeMagUnbiased.has_value())
  {
    CRAS_ERROR_DELAYED_THROTTLE(10.0, "Bias remover cannot work: %s. Waiting...", maybeMagUnbiased.error().c_str());
    return;
  }

  const auto header = event.getConnectionHeaderPtr();
  const auto stamp = event.getReceiptTime();
  this->signalMessage(ros::MessageEvent<Field const>(
    boost::make_shared<Field>(*maybeMagUnbiased), header, stamp, false, ros::DefaultMessageCreator<Field>()));
}

void BiasRemoverFilter::cbBias(const ros::MessageEvent<Field const>& event)
{
  this->remover->setBias(*event.getConstMessage());
}

}
