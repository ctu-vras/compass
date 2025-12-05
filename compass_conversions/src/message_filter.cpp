// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to convert between various compass representations.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <compass_conversions/message_filter.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_interfaces/string_utils.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <message_filters/message_event.hpp>
#include <message_filters/subscriber.hpp>
#include <rclcpp/node_interfaces/get_node_clock_interface.hpp>
#include <rclcpp/node_interfaces/get_node_logging_interface.hpp>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace compass_conversions
{
using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Quat = geometry_msgs::msg::QuaternionStamped;
using Ser = rclcpp::SerializedMessage;

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
UniversalAzimuthSubscriber::UniversalAzimuthSubscriber(RequiredInterfaces node,
  const std::string& topic, const rclcpp::QoS& qos, rclcpp::SubscriptionOptions subscribeOptions) :
#else
UniversalAzimuthSubscriber::UniversalAzimuthSubscriber(rclcpp::Node* node, const std::string& topic,
  const rmw_qos_profile_t qos, rclcpp::SubscriptionOptions subscribeOptions) :
#endif
  azSub(message_filters::Subscriber<Az>()),
  poseSub(message_filters::Subscriber<Pose>()),
  quatSub(message_filters::Subscriber<Quat>()),
  imuSub(message_filters::Subscriber<Imu>()),
  serSub(message_filters::Subscriber<Ser>()),
  node(node), converter(node, true), topic(topic), qos(qos), options(subscribeOptions)
{
  UniversalAzimuthSubscriber::subscribe(node, topic, qos, subscribeOptions);
}

UniversalAzimuthSubscriber::~UniversalAzimuthSubscriber()
{
  UniversalAzimuthSubscriber::unsubscribe();
}

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
void UniversalAzimuthSubscriber::subscribe(RequiredInterfaces node, const std::string& topic,
  const rclcpp::QoS& qos, rclcpp::SubscriptionOptions subscribeOptions)
#else
void UniversalAzimuthSubscriber::subscribe(rclcpp::Node* node, const std::string& topic, const rmw_qos_profile_t qos,
    const rclcpp::SubscriptionOptions subscribeOptions)
#endif
{
  this->unsubscribe();

  if (!topic.empty())
  {
    this->node = node;
    this->topic = topic;
    this->qos = qos;
    this->options = subscribeOptions;

    this->azSub.subscribe(node, topic, qos, subscribeOptions);
    this->poseSub.subscribe(node, topic + "/pose", qos, subscribeOptions);
    this->quatSub.subscribe(node, topic + "/quat", qos, subscribeOptions);
    this->imuSub.subscribe(node, topic + "/imu", qos, subscribeOptions);

    this->azSub.registerCallback(
      std::function<void(const AzimuthEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::azCb, this)));
    this->poseSub.registerCallback(
      std::function<void(const PoseEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::poseCb, this)));
    this->quatSub.registerCallback(
      std::function<void(const QuatEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::quatCb, this)));
    this->imuSub.registerCallback(
      std::function<void(const ImuEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::imuCb, this)));
    this->serSub.registerCallback(
      std::function<void(const SerializedEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::serCb, this)));

    const auto log = rclcpp::node_interfaces::get_node_logging_interface(this->node);
    RCLCPP_INFO(log->get_logger(), "Listening for azimuth at topics %s, %s, %s, %s.\n",
      topic.c_str(), (topic + "/pose").c_str(), (topic + "/quat").c_str(), (topic + "/imu").c_str());
  }
}

void UniversalAzimuthSubscriber::subscribe()
{
  this->subscribe(this->node, this->topic, this->qos, this->options);
}

void UniversalAzimuthSubscriber::unsubscribe()
{
  this->azSub.unsubscribe();
  this->poseSub.unsubscribe();
  this->quatSub.unsubscribe();
  this->imuSub.unsubscribe();
  this->serSub.unsubscribe();
}

void UniversalAzimuthSubscriber::setInputDefaults(const std::optional<Orientation>& orientation,
  const std::optional<Reference>& reference, const std::optional<Variance>& variance)
{
  this->inputOrientation = orientation;
  this->inputReference = reference;
  this->inputVariance = variance;
}

void UniversalAzimuthSubscriber::configFromParams()
{
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  const auto params = this->node.get_node_parameters_interface();
#else
  const auto params = this->node;
#endif

  std::optional<Orientation> inputOrientation;
  if (params->has_parameter("input_orientation") && !params->get_parameter("input_orientation").as_string().empty())
    inputOrientation = compass_interfaces::parseOrientation(params->get_parameter("input_orientation").as_string());

  std::optional<Reference> inputReference;
  if (params->has_parameter("input_reference") && !params->get_parameter("input_reference").as_string().empty())
    inputReference = compass_interfaces::parseReference(params->get_parameter("input_reference").as_string());

  std::optional<Variance> inputVariance;
  if (params->has_parameter("input_variance") && params->get_parameter("input_variance").as_double() != -1.)
    inputVariance = params->get_parameter("input_variance").as_double();

  this->setInputDefaults(inputOrientation, inputReference, inputVariance);
}

std::string UniversalAzimuthSubscriber::getTopic() const
{
  return this->topic;
}

void UniversalAzimuthSubscriber::azCb(const AzimuthEventType& event)
{
  const auto& msg = event.getConstMessage();
  const auto stamp = event.getReceiptTime();

  const auto maybeAzimuth = this->converter.convertAzimuth(*msg, Az::UNIT_RAD, msg->orientation, msg->reference);

  if (!maybeAzimuth.has_value())
  {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(this->node);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(this->node);
    RCLCPP_ERROR_THROTTLE(log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }

  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::poseCb(const PoseEventType& event)
{
  const auto stamp = event.getReceiptTime();
  const auto topics = rclcpp::node_interfaces::get_node_topics_interface(this->node);

  const auto poseTopic = topics->resolve_topic_name(this->poseSub.getTopic());
  const auto maybeAzimuth = this->converter.convertPoseMsgEvent(poseTopic, event,
    Az::UNIT_RAD, this->inputOrientation, this->inputReference);

  if (!maybeAzimuth.has_value())
  {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(this->node);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(this->node);
    RCLCPP_ERROR_THROTTLE(log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::quatCb(const QuatEventType& event)
{
  const auto stamp = event.getReceiptTime();
  const auto topics = rclcpp::node_interfaces::get_node_topics_interface(this->node);
  const auto quatTopic = topics->resolve_topic_name(this->quatSub.getTopic());
  const auto maybeAzimuth = this->converter.convertQuaternionMsgEvent(
    quatTopic, event, this->inputVariance.value_or(0.0), Az::UNIT_RAD, this->inputOrientation, this->inputReference);

  if (!maybeAzimuth.has_value())
  {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(this->node);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(this->node);
    RCLCPP_ERROR_THROTTLE(log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::imuCb(const ImuEventType& event)
{
  const auto stamp = event.getReceiptTime();
  const auto topics = rclcpp::node_interfaces::get_node_topics_interface(this->node);
  const auto imuTopic = topics->resolve_topic_name(this->imuSub.getTopic());
  const auto maybeAzimuth = this->converter.convertImuMsgEvent(imuTopic, event,
    Az::UNIT_RAD, this->inputOrientation, this->inputReference);

  if (!maybeAzimuth.has_value())
  {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(this->node);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(this->node);
    RCLCPP_ERROR_THROTTLE(log->get_logger(), *clock->get_clock(), 10000.,
     "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::serCb(const SerializedEventType& event)
{
  const auto stamp = event.getReceiptTime();
  const auto maybeAzimuth = this->converter.convertSerializedMsgEvent(this->topic, event,
    Az::UNIT_RAD, this->inputVariance.value_or(0.0), this->inputOrientation, this->inputReference);

  if (!maybeAzimuth.has_value())
  {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(this->node);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(this->node);
    RCLCPP_ERROR_THROTTLE(log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

CompassFilter::~CompassFilter() = default;

void CompassFilter::cbAzimuth(const AzimuthEventType& azimuthEvent)
{
  const auto& msg = azimuthEvent.getConstMessage();

  const auto output = this->converter->convertAzimuth(
    *msg, this->unit, this->orientation, this->reference.value_or(msg->reference));
  if (!output.has_value())
  {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(this->node);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(this->node);
    RCLCPP_ERROR_THROTTLE(log->get_logger(), *clock->get_clock(), 10000.,
      "Azimuth conversion failed%s: %s", fixReceived ? "" : " (no fix message received yet)", output.error().c_str());
    return;
  }

  this->signalMessage(AzimuthEventType(
    std::make_shared<Az const>(*output),
    azimuthEvent.getReceiptTime(), false, message_filters::DefaultMessageCreator<Az>()));
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
