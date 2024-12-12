// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to convert between various compass representations.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <compass_conversions/message_filter.h>
#include <compass_msgs/string_utils.h>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <ros/callback_queue_interface.h>
#include <ros/message_event.h>
#include <ros/node_handle.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>
#include <ros/transport_hints.h>
#include <sensor_msgs/NavSatFix.h>

using Az = compass_msgs::Azimuth;

namespace compass_conversions
{

UniversalAzimuthSubscriber::UniversalAzimuthSubscriber(const cras::LogHelperPtr& log, ros::NodeHandle& nh,
  const std::string& topic, const uint32_t queueSize, const ros::TransportHints& transportHints,
  ros::CallbackQueueInterface* callbackQueue) : cras::HasLogger(log), converter(log, true)
{
  this->subscribe(nh, topic, queueSize, transportHints, callbackQueue);
}

UniversalAzimuthSubscriber::~UniversalAzimuthSubscriber()
{
  this->unsubscribe();
}

void UniversalAzimuthSubscriber::subscribe(ros::NodeHandle& nh, const std::string& topic, const uint32_t queueSize,
  const ros::TransportHints& transportHints, ros::CallbackQueueInterface* callbackQueue)
{
  this->unsubscribe();

  if (!topic.empty())
  {
    this->subscribeOps.template initByFullCallbackType<const EventType&>(
      topic, queueSize, boost::bind(&UniversalAzimuthSubscriber::cb, this, boost::placeholders::_1));
    this->subscribeOps.callback_queue = callbackQueue;
    this->subscribeOps.transport_hints = transportHints;
    this->sub = nh.subscribe(this->subscribeOps);
    this->nh = nh;
  }
}

void UniversalAzimuthSubscriber::subscribe()
{
  this->unsubscribe();

  if (!this->subscribeOps.topic.empty())
    this->sub = this->nh.subscribe(this->subscribeOps);
}

void UniversalAzimuthSubscriber::unsubscribe()
{
  sub.shutdown();
}

void UniversalAzimuthSubscriber::setInputDefaults(
  const cras::optional<decltype(compass_msgs::Azimuth::orientation)>& orientation,
  const cras::optional<decltype(compass_msgs::Azimuth::reference)>& reference,
  const cras::optional<decltype(compass_msgs::Azimuth::variance)>& variance)
{
  this->inputOrientation = orientation;
  this->inputReference = reference;
  this->inputVariance = variance;
}

void UniversalAzimuthSubscriber::configFromParams(const cras::BoundParamHelper& params)
{
  cras::optional<decltype(Az::orientation)> inputOrientation;
  if (params.hasParam("input_orientation"))
    inputOrientation = params.getParam<decltype(Az::orientation), std::string>(
      "input_orientation", cras::nullopt, "",
      {.resultToStr = &compass_msgs::orientationToString, .toResult = &compass_msgs::parseOrientation});

  cras::optional<decltype(Az::reference)> inputReference;
  if (params.hasParam("input_reference"))
    inputReference = params.getParam<decltype(Az::reference), std::string>(
      "input_reference", cras::nullopt, "",
      {.resultToStr = &compass_msgs::referenceToString, .toResult = &compass_msgs::parseReference});

  cras::optional<decltype(Az::variance)> inputVariance;
  if (params.hasParam("input_variance"))
    inputVariance = params.getParam<double>("input_variance", cras::nullopt, "rad^2");

  this->setInputDefaults(inputOrientation, inputReference, inputVariance);
}

std::string UniversalAzimuthSubscriber::getTopic() const
{
  return this->subscribeOps.topic;
}

const ros::Subscriber& UniversalAzimuthSubscriber::getSubscriber() const
{
  return this->sub;
}

void UniversalAzimuthSubscriber::add(const EventType&)
{
}

void UniversalAzimuthSubscriber::cb(const EventType& event)
{
  const auto maybeAzimuth = this->converter.convertUniversalMsgEvent(
    event, this->inputVariance.value_or(0.0), Az::UNIT_RAD, this->inputOrientation, this->inputReference);
  if (!maybeAzimuth.has_value())
  {
    CRAS_ERROR_THROTTLE(10.0, "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }

  const auto header = event.getConnectionHeaderPtr();
  const auto stamp = event.getReceiptTime();
  this->signalMessage(ros::MessageEvent<Az const>(
    boost::make_shared<Az>(*maybeAzimuth), header, stamp, false, ros::DefaultMessageCreator<Az>()));
}


CompassFilter::~CompassFilter() = default;

void CompassFilter::cbAzimuth(const AzimuthEventType& azimuthEvent)
{
  const auto& msg = azimuthEvent.getConstMessage();
  const auto output = this->converter->convertAzimuth(
    *msg, this->unit, this->orientation, this->reference.value_or(msg->reference));
  if (!output.has_value())
  {
    CRAS_ERROR_THROTTLE(10.0,
      "Azimuth conversion failed%s: %s", fixReceived ? "" : "(no fix message received yet)", output.error().c_str());
    return;
  }
  this->signalMessage(AzimuthEventType(
    boost::make_shared<Az>(*output), azimuthEvent.getConnectionHeaderPtr(),
    azimuthEvent.getReceiptTime(), false, ros::DefaultMessageCreator<Az>()));
}

void CompassFilter::cbFix(const FixEventType& fixEvent)
{
  this->fixReceived = true;
  this->converter->setNavSatPos(*fixEvent.getConstMessage());
}

void CompassFilter::cbUTMZone(const UTMZoneEventType& utmZoneEvent)
{
  this->converter->forceUTMZone(utmZoneEvent.getConstMessage()->data);
}

}
