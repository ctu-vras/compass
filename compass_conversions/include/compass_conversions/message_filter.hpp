#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to convert between various compass representations.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <optional>
#include <string>

#include <compass_conversions/compass_converter.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <message_filters/connection.hpp>
#include <message_filters/message_event.hpp>
#include <message_filters/simple_filter.hpp>
#include <message_filters/subscriber.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>

namespace compass_conversions {

/**
 * \brief message_filters subscriber that can subscribe to various topic types and convert them all to an Azimuth
 *        message.
 *
 * Currently supported types are: compass_interfaces::msg::Azimuth, geometry_msgs::msg::PoseWithCovarianceStamped,
 * geometry_msgs::msg::QuaternionStamped, sensor_msgs::msg::Imu.
 */
class UniversalAzimuthSubscriber
    : public message_filters::SimpleFilter<compass_interfaces::msg::Azimuth>,
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_BASE_IS_TEMPLATE
    public message_filters::SubscriberBase<>
#else
    public message_filters::SubscriberBase
#endif
{
public:
  using NodeClockInterface = rclcpp::node_interfaces::NodeClockInterface;
  using NodeGraphInterface = rclcpp::node_interfaces::NodeGraphInterface;
  using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;
  using NodeParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;
  using NodeTopicsInterface = rclcpp::node_interfaces::NodeTopicsInterface;

  using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      NodeClockInterface,
      NodeGraphInterface,
      NodeLoggingInterface,
      NodeParametersInterface,
      NodeTopicsInterface
  >;

  // typedef message_filters::MessageEvent<rclcpp::GenericSubscription const> EventType;
  typedef message_filters::MessageEvent<compass_interfaces::msg::Azimuth const> AzimuthEventType;
  typedef message_filters::MessageEvent<geometry_msgs::msg::PoseWithCovarianceStamped const> PoseEventType;
  typedef message_filters::MessageEvent<geometry_msgs::msg::QuaternionStamped const> QuatEventType;
  typedef message_filters::MessageEvent<sensor_msgs::msg::Imu const> ImuEventType;
  typedef message_filters::MessageEvent<rclcpp::SerializedMessage const> SerializedEventType;

  using Az = compass_interfaces::msg::Azimuth;

  using Unit = compass_interfaces::msg::Azimuth::_unit_type;
  using Orientation = compass_interfaces::msg::Azimuth::_orientation_type;
  using Reference = compass_interfaces::msg::Azimuth::_reference_type;
  using Variance = compass_interfaces::msg::Azimuth::_variance_type;

  /**
   * \brief Constructor
   *
   * \param[in] node The ROS node to use for subscribing.
   * \param[in] topic The topic to subscribe to.
   * \param[in] qos The QoS setting for the subscription.
   * \param[in] subscribe_options Subscriber options.
   */
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  UniversalAzimuthSubscriber(
      RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos = {10},
      rclcpp::SubscriptionOptions subscribe_options = {});
#else
  UniversalAzimuthSubscriber(
      rclcpp::Node* node, const std::string& topic, rmw_qos_profile_t qos = rmw_qos_profile_default,
      rclcpp::SubscriptionOptions subscribe_options = {});
#endif

  /**
   * \brief Constructor
   *
   * \param[in] node The ROS node to use for subscribing.
   * \param[in] topic The topic to subscribe to.
   * \param[in] qos The QoS setting for the subscription.
   * \param[in] subscribe_options Subscriber options.
   */
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  UniversalAzimuthSubscriber(
      rclcpp::Node* node, const std::string& topic, const rclcpp::QoS& qos = {10},
      rclcpp::SubscriptionOptions subscribe_options = {})
      : UniversalAzimuthSubscriber(*node, topic, qos, subscribe_options) {}
#else
  UniversalAzimuthSubscriber(
      rclcpp::Node* node, const std::string& topic, rclcpp::QoS qos = {10},
      rclcpp::SubscriptionOptions subscribe_options = {})
      : UniversalAzimuthSubscriber(node, topic, qos.get_rmw_qos_profile(), subscribe_options) {}
#endif

  ~UniversalAzimuthSubscriber() override;

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  virtual void subscribe(
      RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos,
      rclcpp::SubscriptionOptions subscribe_options);

  void subscribe(
      SubscriberBase::RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos,
      rclcpp::SubscriptionOptions subscribe_options) override {
    const RequiredInterfaces newNode {
      node_.get_node_clock_interface(), node_.get_node_graph_interface(),
      node_.get_node_logging_interface(),
      node.get_node_parameters_interface(), node.get_node_topics_interface()
    };
    subscribe(newNode, topic, qos, subscribe_options);
  }

  void subscribe(SubscriberBase::RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos) override {
    subscribe(node, topic, qos, {});
  }
#else
  void subscribe(
      rclcpp::Node* node, const std::string& topic, rmw_qos_profile_t qos,
      rclcpp::SubscriptionOptions subscribe_options) override;

  void subscribe(NodePtr node, const std::string& topic, const rmw_qos_profile_t qos) override {
    subscribe(node.get(), topic, qos, {});
  }

  void subscribe(rclcpp::Node* node, const std::string& topic, const rmw_qos_profile_t qos) override {
    subscribe(node, topic, qos, {});
  }
#endif

  /**
   * \brief Re-subscribe to a topic.
   */
  void subscribe() override;

  /**
   * \brief Unsubscribe from the topic.
   */
  void unsubscribe() override;

  /**
   * \brief Set defaults for inputs which do not support autodetection of various azimuth properties.
   *
   * \param[in] orientation The default orientation used if it cannot be detected.
   * \param[in] reference The reference used if it cannot be detected.
   * \param[in] variance Default variance used for topics which cannot automatically discover it.
   */
  void setInputDefaults(
      const std::optional<Orientation>& orientation, const std::optional<Reference>& reference,
      const std::optional<Variance>& variance);

  /**
   * \brief Configure the subscriber from ROS parameters.
   *
   * Supported parameters:
   * - `~input_orientation` (str, 'enu' or 'ned', default: unspecified): ENU or NED orientation to be used to interpret
   *                                                                     input messages (in case orientation cannot be
   *                                                                     derived either from message contents or topic
   *                                                                     name).
   * - `~input_reference` (str, 'magnetic', 'geographic' or 'UTM', default: no change): North reference to be used to
   *                                                                                    interpret input messages (in
   *                                                                                    case reference cannot be derived
   *                                                                                    either from message contents or
   *                                                                                    topic name).
   * - `~input_variance` (double, optional, rad^2): If specified, this variance will be used in the output messages
   *                                                if variance cannot be determined from the input messages (e.g. for
   *                                                `QuaternionStamped`).
   */
  void configFromParams();

  /**
   * \brief Get the name of the subscribed topic.
   * \return The topic name.
   */
  std::string getTopic() const;

  /**
   * \brief Returns the internal rclcpp::Subscription.
   */
  const message_filters::Subscriber<compass_interfaces::msg::Azimuth>& getAzSubscriber() const { return az_sub_; }

  const message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>& getPoseSubscriber() const {
    return pose_sub_;
  }

  const message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>& getQuatSubscriber() const {
    return quat_sub_;
  }

  const message_filters::Subscriber<sensor_msgs::msg::Imu>& getImuSubscriber() const { return imu_sub_; }

  // const message_filters::Subscriber<rclcpp::SerializedMessage>& getSubscriber() const;

  template<typename F>
  void connectInput(F& f) {}

  // void add(const EventType& event);

protected:
  void azCb(const AzimuthEventType& event);

  void poseCb(const PoseEventType& event);

  void quatCb(const QuatEventType& event);

  void imuCb(const ImuEventType& event);

  void serCb(const SerializedEventType& event);

  message_filters::Subscriber<compass_interfaces::msg::Azimuth> az_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> pose_sub_;
  message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> quat_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<rclcpp::SerializedMessage> ser_sub_;
  // rclcpp::GenericSubscription sub;

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  RequiredInterfaces node_;
#else
  rclcpp::Node* node_;
#endif

  CompassConverter converter_;  //!< The azimuth message converter.

  //! Orientation of the input azimuth (in case it is a data type which does not tell orientation explicitly).
  std::optional<Orientation> input_orientation_;

  //! Reference of the input azimuth (in case it is a data type which does not tell reference explicitly).
  std::optional<Reference> input_reference_;

  //! Variance of the input azimuth (in case it is a data type which does not tell reference explicitly).
  std::optional<Variance> input_variance_;

  std::string topic_;
  rclcpp::SubscriptionOptions options_;
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  rclcpp::QoS qos_ {10};
#else
  rmw_qos_profile_t qos_ {rmw_qos_profile_default};
#endif
};

/**
 * \brief Message filter to convert between various compass representations.
 *
 * \sa https://wiki.ros.org/message_filters
 *
 * Example usage:
 * \code{.cpp}
 * message_filters::UniversalAzimuthSubscriber azimuthInput(...);
 * message_filters::Subscriber<sensor_msgs::msg::NavSatFix> fixInput(...);
 * auto converter = std::make_shared<compass_conversions::CompassConverter>(log, true);
 * // converter->configFromParams(params);
 * compass_conversions::CompassFilter filter(log, converter, azimuthInput, fixInput,
 *   compass_interfaces::msg::Azimuth::UNIT_RAD, compass_interfaces::msg::Azimuth::ORIENTATION_ENU,
 *   compass_interfaces::msg::Azimuth::REFERENCE_GEOGRAPHIC);
 * filter.registerCallback([](const compass_interfaces::msg::AzimuthConstSharedPtr& msg) {
 *   ...  // Handle the data
 * });
 * \endcode
 */
class CompassFilter : public message_filters::SimpleFilter<compass_interfaces::msg::Azimuth> {
public:
  using NodeClockInterface = rclcpp::node_interfaces::NodeClockInterface;
  using NodeGraphInterface = rclcpp::node_interfaces::NodeGraphInterface;
  using NodeLoggingInterface = rclcpp::node_interfaces::NodeLoggingInterface;
  using NodeParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;
  using NodeTopicsInterface = rclcpp::node_interfaces::NodeTopicsInterface;

  using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
      NodeClockInterface,
      NodeGraphInterface,
      NodeLoggingInterface,
      NodeParametersInterface,
      NodeTopicsInterface
  >;

  typedef message_filters::MessageEvent<compass_interfaces::msg::Azimuth const> AzimuthEventType;
  typedef message_filters::MessageEvent<sensor_msgs::msg::NavSatFix const> FixEventType;
  typedef message_filters::MessageEvent<std_msgs::msg::Int32 const> UTMZoneEventType;

  using Unit = compass_interfaces::msg::Azimuth::_unit_type;
  using Orientation = compass_interfaces::msg::Azimuth::_orientation_type;
  using Reference = compass_interfaces::msg::Azimuth::_reference_type;
  using Variance = compass_interfaces::msg::Azimuth::_variance_type;

  /**
   * \brief Construct azimuth filter that can convert all parameters.
   *
   * \tparam AzimuthInput The type of the input filter.
   * \tparam FixInput The type of the navsat fix filter.
   * \tparam UTMZoneInput The type of the UTM Zone filter.
   * \param[in] node The node to use.
   * \param[in] converter The azimuth converter instance. If nullptr, a default converter is constructed.
   * \param[in] azimuth_input The message filter producing azimuth messages.
   * \param[in] fix_input The message filter producing fix messages.
   * \param[in] utm_zone_input The message filter producing UTM zone messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput, class FixInput, class UTMZoneInput>
  CompassFilter(
      RequiredInterfaces node, const std::shared_ptr<CompassConverter>& converter,
      AzimuthInput& azimuth_input, FixInput& fix_input, UTMZoneInput& utm_zone_input,
      const Unit unit, const Orientation orientation, const Reference reference)
      : converter_(converter), unit_(unit), orientation_(orientation), reference_(reference), node_(node) {
    if (converter_ == nullptr) {
      converter_ = std::make_shared<CompassConverter>(node, true);
    }
    connectAzimuthInput(azimuth_input);
    connectFixInput(fix_input);
    connectUTMZoneInput(utm_zone_input);
  }

  template<class AzimuthInput, class FixInput, class UTMZoneInput>
  CompassFilter(
      rclcpp::Node* node, const std::shared_ptr<CompassConverter>& converter,
      AzimuthInput& azimuth_input, FixInput& fix_input, UTMZoneInput& utm_zone_input,
      const Unit unit, const Orientation orientation, const Reference reference)
      : CompassFilter(*node, converter, azimuth_input, fix_input, utm_zone_input, unit, orientation, reference) {}

  /**
   * \brief Construct azimuth filter that can convert all parameters.
   *
   * \tparam AzimuthInput The type of the input filter.
   * \tparam FixInput The type of the navsat fix filter.
   * \param[in] node The node to use.
   * \param[in] converter The azimuth converter instance. If nullptr, a default converter is constructed.
   * \param[in] azimuth_input The message filter producing azimuth messages.
   * \param[in] fix_input The message filter producing fix messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput, class FixInput>
  CompassFilter(
      RequiredInterfaces node, const std::shared_ptr<CompassConverter>& converter,
      AzimuthInput& azimuth_input, FixInput& fix_input,
      const Unit unit, const Orientation orientation, const Reference reference)
      : converter_(converter), unit_(unit), orientation_(orientation), reference_(reference), node_(node) {
    if (converter_ == nullptr) {
      converter_ = std::make_shared<CompassConverter>(node, true);
    }
    connectAzimuthInput(azimuth_input);
    connectFixInput(fix_input);
  }

  template<class AzimuthInput, class FixInput>
  CompassFilter(
      rclcpp::Node* node, const std::shared_ptr<CompassConverter>& converter,
      AzimuthInput& azimuth_input, FixInput& fix_input,
      const Unit unit, const Orientation orientation, const Reference reference)
      : CompassFilter(*node, converter, azimuth_input, fix_input, unit, orientation, reference) {}

  /**
   * \brief Construct azimuth filter that can only convert units and orientation.
   *
   * \tparam AzimuthInput The type of the input filter.
   * \param[in] node The node to use.
   * \param[in] converter The azimuth converter instance. If nullptr, a default converter is constructed.
   * \param[in] azimuth_input The message filter producing azimuth messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput>
  CompassFilter(
      const RequiredInterfaces node, const std::shared_ptr<CompassConverter>& converter,
      AzimuthInput& azimuth_input, const Unit unit, const Orientation orientation, const Reference reference)
      : converter_(converter), unit_(unit), orientation_(orientation), reference_(reference), node_(node) {
    if (converter_ == nullptr) {
      converter_ = std::make_shared<CompassConverter>(node, true);
    }
    connectAzimuthInput(azimuth_input);
  }

  template<class AzimuthInput>
  CompassFilter(
      rclcpp::Node* node, const std::shared_ptr<CompassConverter>& converter,
      AzimuthInput& azimuth_input, const Unit unit, const Orientation orientation, const Reference reference)
      : CompassFilter(*node, converter, azimuth_input, unit, orientation, reference) {}

  virtual ~CompassFilter();

  template<class AzimuthInput>
  void connectAzimuthInput(AzimuthInput& f) {
    azimuth_connection_.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    azimuth_connection_ = f.registerCallback(
      std::function<void(const AzimuthEventType&)>(std::bind_front(&CompassFilter::cbAzimuth, this)));
  }

  template<class FixInput>
  void connectFixInput(FixInput& f) {
    fix_connection_.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    fix_connection_ = f.registerCallback(
      std::function<void(const FixEventType&)>(std::bind_front(&CompassFilter::cbFix, this)));
  }

  template<class UTMZoneInput>
  void connectUTMZoneInput(UTMZoneInput& f) {
    utm_zone_connection_.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    utm_zone_connection_ = f.registerCallback(
      std::function<void(const UTMZoneEventType&)>(std::bind_front(&CompassFilter::cbUTMZone, this)));
  }

protected:
  virtual void cbAzimuth(const AzimuthEventType& azimuth_event);

  virtual void cbFix(const FixEventType& fix_event);

  virtual void cbUTMZone(const UTMZoneEventType& utm_zone_event);

  message_filters::Connection azimuth_connection_;  //!< Connection to the azimuth input.
  message_filters::Connection fix_connection_;  //!< Connection to the navsat fix input.
  message_filters::Connection utm_zone_connection_;  //!< Connection to the UTM zone input.

  std::shared_ptr<CompassConverter> converter_;  //!< The compass converter instance.
  bool fix_received_ {false};  //!< Whether at least one navsat fix message has been received.

  Unit unit_;  //!< The target azimuth unit.
  Orientation orientation_;  //!< The target azimuth orientation.

  //! The target azimuth reference (unchanged if empty).
  std::optional<Reference> reference_;
  RequiredInterfaces node_;
};

}  // namespace compass_conversions
