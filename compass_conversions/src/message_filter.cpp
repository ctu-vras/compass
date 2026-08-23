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

namespace compass_conversions {

using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Quat = geometry_msgs::msg::QuaternionStamped;
using Ser = rclcpp::SerializedMessage;

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
UniversalAzimuthSubscriber::UniversalAzimuthSubscriber(
    RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos,
    rclcpp::SubscriptionOptions subscribe_options)
    : az_sub_(message_filters::Subscriber<Az>()), pose_sub_(message_filters::Subscriber<Pose>()),
      quat_sub_(message_filters::Subscriber<Quat>()), imu_sub_(message_filters::Subscriber<Imu>()),
      ser_sub_(message_filters::Subscriber<Ser>()), node_(node), converter_(node, true), topic_(topic),
      options_(subscribe_options), qos_(qos) {
  UniversalAzimuthSubscriber::subscribe(node, topic, qos, subscribe_options);
}
#else
UniversalAzimuthSubscriber::UniversalAzimuthSubscriber(
    rclcpp::Node* node, const std::string& topic, const rmw_qos_profile_t qos,
    rclcpp::SubscriptionOptions subscribe_options)
    : az_sub_(message_filters::Subscriber<Az>()), pose_sub_(message_filters::Subscriber<Pose>()),
      quat_sub_(message_filters::Subscriber<Quat>()), imu_sub_(message_filters::Subscriber<Imu>()),
      ser_sub_(message_filters::Subscriber<Ser>()), node_(node), converter_(node, true), topic_(topic),
      options_(subscribe_options), qos_(qos) {
  UniversalAzimuthSubscriber::subscribe(node, topic, qos, subscribe_options);
}
#endif

UniversalAzimuthSubscriber::~UniversalAzimuthSubscriber() {
  UniversalAzimuthSubscriber::unsubscribe();
}

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
void UniversalAzimuthSubscriber::subscribe(
    RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos,
    rclcpp::SubscriptionOptions subscribe_options)
#else
void UniversalAzimuthSubscriber::subscribe(
    rclcpp::Node* node, const std::string& topic, const rmw_qos_profile_t qos,
    const rclcpp::SubscriptionOptions subscribe_options)
#endif
{
  unsubscribe();

  if (!topic.empty()) {
    node_ = node;
    topic_ = topic;
    qos_ = qos;
    options_ = subscribe_options;

    az_sub_.subscribe(node, topic, qos, subscribe_options);
    pose_sub_.subscribe(node, topic + "/pose", qos, subscribe_options);
    quat_sub_.subscribe(node, topic + "/quat", qos, subscribe_options);
    imu_sub_.subscribe(node, topic + "/imu", qos, subscribe_options);

    az_sub_.registerCallback(
      std::function<void(const AzimuthEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::azCb, this)));
    pose_sub_.registerCallback(
      std::function<void(const PoseEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::poseCb, this)));
    quat_sub_.registerCallback(
      std::function<void(const QuatEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::quatCb, this)));
    imu_sub_.registerCallback(
      std::function<void(const ImuEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::imuCb, this)));
    ser_sub_.registerCallback(
      std::function<void(const SerializedEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::serCb, this)));

    const auto log = rclcpp::node_interfaces::get_node_logging_interface(node_);
    RCLCPP_INFO(
      log->get_logger(), "Listening for azimuth at topics %s, %s, %s, %s.\n",
      topic.c_str(), (topic + "/pose").c_str(), (topic + "/quat").c_str(), (topic + "/imu").c_str());
  }
}

void UniversalAzimuthSubscriber::subscribe() {
  subscribe(node_, topic_, qos_, options_);
}

void UniversalAzimuthSubscriber::unsubscribe() {
  az_sub_.unsubscribe();
  pose_sub_.unsubscribe();
  quat_sub_.unsubscribe();
  imu_sub_.unsubscribe();
  ser_sub_.unsubscribe();
}

void UniversalAzimuthSubscriber::setInputDefaults(
    const std::optional<Orientation>& orientation, const std::optional<Reference>& reference,
    const std::optional<Variance>& variance) {
  input_orientation_ = orientation;
  input_reference_ = reference;
  input_variance_ = variance;
}

void UniversalAzimuthSubscriber::configFromParams() {
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  const auto params = node_.get_node_parameters_interface();
#else
  const auto params = node_;
#endif

  std::optional<Orientation> input_orientation;
  if (params->has_parameter("input_orientation") && !params->get_parameter("input_orientation").as_string().empty()) {
    input_orientation = compass_interfaces::parseOrientation(params->get_parameter("input_orientation").as_string());
  }

  std::optional<Reference> input_reference;
  if (params->has_parameter("input_reference") && !params->get_parameter("input_reference").as_string().empty()) {
    input_reference = compass_interfaces::parseReference(params->get_parameter("input_reference").as_string());
  }

  std::optional<Variance> input_variance;
  if (params->has_parameter("input_variance") && params->get_parameter("input_variance").as_double() != -1.) {
    input_variance = params->get_parameter("input_variance").as_double();
  }

  setInputDefaults(input_orientation, input_reference, input_variance);
}

std::string UniversalAzimuthSubscriber::getTopic() const {
  return topic_;
}

void UniversalAzimuthSubscriber::azCb(const AzimuthEventType& event) {
  const auto& msg = event.getConstMessage();
  const auto stamp = event.getReceiptTime();

  const auto maybe_azimuth = converter_.convertAzimuth(*msg, Az::UNIT_RAD, msg->orientation, msg->reference);

  if (!maybe_azimuth.has_value()) {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(node_);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(node_);
    RCLCPP_ERROR_THROTTLE(
      log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybe_azimuth.error().c_str());
    return;
  }

  signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybe_azimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::poseCb(const PoseEventType& event) {
  const auto stamp = event.getReceiptTime();
  const auto topics = rclcpp::node_interfaces::get_node_topics_interface(node_);

  const auto pose_topic = topics->resolve_topic_name(pose_sub_.getTopic());
  const auto maybe_azimuth = converter_.convertPoseMsgEvent(
    pose_topic, event, Az::UNIT_RAD, input_orientation_, input_reference_);

  if (!maybe_azimuth.has_value()) {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(node_);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(node_);
    RCLCPP_ERROR_THROTTLE(
      log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybe_azimuth.error().c_str());
    return;
  }
  signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybe_azimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::quatCb(const QuatEventType& event) {
  const auto stamp = event.getReceiptTime();
  const auto topics = rclcpp::node_interfaces::get_node_topics_interface(node_);
  const auto quat_topic = topics->resolve_topic_name(quat_sub_.getTopic());
  const auto maybe_azimuth = converter_.convertQuaternionMsgEvent(
    quat_topic, event, input_variance_.value_or(0.0), Az::UNIT_RAD, input_orientation_, input_reference_);

  if (!maybe_azimuth.has_value()) {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(node_);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(node_);
    RCLCPP_ERROR_THROTTLE(
      log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybe_azimuth.error().c_str());
    return;
  }
  signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybe_azimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::imuCb(const ImuEventType& event) {
  const auto stamp = event.getReceiptTime();
  const auto topics = rclcpp::node_interfaces::get_node_topics_interface(node_);
  const auto imu_topic = topics->resolve_topic_name(imu_sub_.getTopic());
  const auto maybe_azimuth = converter_.convertImuMsgEvent(
    imu_topic, event, Az::UNIT_RAD, input_orientation_, input_reference_);

  if (!maybe_azimuth.has_value()) {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(node_);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(node_);
    RCLCPP_ERROR_THROTTLE(
      log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybe_azimuth.error().c_str());
    return;
  }
  signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybe_azimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::serCb(const SerializedEventType& event) {
  const auto stamp = event.getReceiptTime();
  const auto maybe_azimuth = converter_.convertSerializedMsgEvent(
    topic_, event, Az::UNIT_RAD, input_variance_.value_or(0.0), input_orientation_, input_reference_);

  if (!maybe_azimuth.has_value()) {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(node_);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(node_);
    RCLCPP_ERROR_THROTTLE(
      log->get_logger(), *clock->get_clock(), 10000.,
      "Error converting message to Azimuth: %s", maybe_azimuth.error().c_str());
    return;
  }
  signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybe_azimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

CompassFilter::~CompassFilter() = default;

void CompassFilter::cbAzimuth(const AzimuthEventType& azimuth_event) {
  const auto& msg = azimuth_event.getConstMessage();

  const auto output = converter_->convertAzimuth(
    *msg, unit_, orientation_, reference_.value_or(msg->reference));
  if (!output.has_value()) {
    const auto clock = rclcpp::node_interfaces::get_node_clock_interface(node_);
    const auto log = rclcpp::node_interfaces::get_node_logging_interface(node_);
    RCLCPP_ERROR_THROTTLE(
      log->get_logger(), *clock->get_clock(), 10000.,
      "Azimuth conversion failed%s: %s", fix_received_ ? "" : " (no fix message received yet)", output.error().c_str());
    return;
  }

  signalMessage(AzimuthEventType(
    std::make_shared<Az const>(*output),
    azimuth_event.getReceiptTime(), false, message_filters::DefaultMessageCreator<Az>()));
}

void CompassFilter::cbFix(const FixEventType& fix_event) {
  fix_received_ = true;
  converter_->setNavSatPos(*fix_event.getConstMessage());
}

void CompassFilter::cbUTMZone(const UTMZoneEventType& utm_zone_event) {
  converter_->forceUTMZone(utm_zone_event.getConstMessage()->data);
}

}  // namespace compass_conversions
