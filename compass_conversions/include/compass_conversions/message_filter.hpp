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

namespace compass_conversions
{

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
   * \param log Logger.
   * \param nh The ros::NodeHandle to use for subscribing.
   * \param topic The topic to subscribe to.
   * \param queueSize Queue size of the subscription.
   */
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  UniversalAzimuthSubscriber(RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos = {10},
    rclcpp::SubscriptionOptions subscribeOptions = {});

  UniversalAzimuthSubscriber(rclcpp::Node* node, const std::string& topic, const rclcpp::QoS& qos = {10},
    rclcpp::SubscriptionOptions subscribeOptions = {}) : UniversalAzimuthSubscriber(*node, topic, qos, subscribeOptions)
  {
  }
#else
  UniversalAzimuthSubscriber(rclcpp::Node* node, const std::string& topic,
    rmw_qos_profile_t qos = rmw_qos_profile_default, rclcpp::SubscriptionOptions subscribeOptions = {});

  UniversalAzimuthSubscriber(rclcpp::Node* node, const std::string& topic,
    rclcpp::QoS qos = {10}, rclcpp::SubscriptionOptions subscribeOptions = {})
    : UniversalAzimuthSubscriber(node, topic, qos.get_rmw_qos_profile(), subscribeOptions)
  {
  }
#endif

  ~UniversalAzimuthSubscriber() override;

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  virtual void subscribe(RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos,
    rclcpp::SubscriptionOptions subscribeOptions);

  void subscribe(SubscriberBase::RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos,
    rclcpp::SubscriptionOptions subscribeOptions) override
  {
    const RequiredInterfaces newNode
    {
      this->node.get_node_clock_interface(), this->node.get_node_graph_interface(),
      this->node.get_node_logging_interface(),
      node.get_node_parameters_interface(), node.get_node_topics_interface()
    };
    this->subscribe(newNode, topic, qos, subscribeOptions);
  }

  void subscribe(SubscriberBase::RequiredInterfaces node, const std::string& topic, const rclcpp::QoS& qos) override
  {
    this->subscribe(node, topic, qos, {});
  }
#else
  void subscribe(rclcpp::Node* node, const std::string& topic, rmw_qos_profile_t qos,
    rclcpp::SubscriptionOptions subscribeOptions) override;

  void subscribe(NodePtr node, const std::string& topic, const rmw_qos_profile_t qos) override
  {
    this->subscribe(node.get(), topic, qos, {});
  }

  void subscribe(rclcpp::Node* node, const std::string& topic, const rmw_qos_profile_t qos) override
  {
    this->subscribe(node, topic, qos, {});
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
  void setInputDefaults(const std::optional<Orientation>& orientation, const std::optional<Reference>& reference,
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
  const message_filters::Subscriber<compass_interfaces::msg::Azimuth>& getAzSubscriber() const { return this->azSub; }
  const message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>& getPoseSubscriber() const
  {
    return this->poseSub;
  }
  const message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>& getQuatSubscriber() const
  {
    return this->quatSub;
  }
  const message_filters::Subscriber<sensor_msgs::msg::Imu>& getImuSubscriber() const { return this->imuSub; }

  // const message_filters::Subscriber<rclcpp::SerializedMessage>& getSubscriber() const;

  template<typename F>
  void connectInput(F& f)
  {
  }

  // void add(const EventType& event);

protected:
  void azCb(const AzimuthEventType& event);
  void poseCb(const PoseEventType& event);
  void quatCb(const QuatEventType& event);
  void imuCb(const ImuEventType& event);
  void serCb(const SerializedEventType& event);

  message_filters::Subscriber<compass_interfaces::msg::Azimuth> azSub;
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> poseSub;
  message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> quatSub;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imuSub;
  message_filters::Subscriber<rclcpp::SerializedMessage> serSub;
  // rclcpp::GenericSubscription sub;

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  RequiredInterfaces node;
#else
  rclcpp::Node* node;
#endif

  CompassConverter converter;  //!< The azimuth message converter.

  //! Orientation of the input azimuth (in case it is a data type which does not tell orientation explicitly).
  std::optional<Orientation> inputOrientation;

  //! Reference of the input azimuth (in case it is a data type which does not tell reference explicitly).
  std::optional<Reference> inputReference;

  //! Variance of the input azimuth (in case it is a data type which does not tell reference explicitly).
  std::optional<Variance> inputVariance;

  std::string topic {std::string()};
  rclcpp::SubscriptionOptions options {};
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  rclcpp::QoS qos {10};
#else
  rmw_qos_profile_t qos {rmw_qos_profile_default};
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
class CompassFilter : public message_filters::SimpleFilter<compass_interfaces::msg::Azimuth>
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
   * \param[in] azimuthInput The message filter producing azimuth messages.
   * \param[in] fixInput The message filter producing fix messages.
   * \param[in] utmZoneInput The message filter producing UTM zone messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput, class FixInput, class UTMZoneInput>
  CompassFilter(RequiredInterfaces node, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput, FixInput& fixInput, UTMZoneInput& utmZoneInput,
    Unit unit, Orientation orientation, Reference reference)
    : node(node), converter(converter), unit(unit), orientation(orientation), reference(reference)
  {
    if (this->converter == nullptr)
      this->converter = std::make_shared<CompassConverter>(node, true);
    this->connectAzimuthInput(azimuthInput);
    this->connectFixInput(fixInput);
    this->connectUTMZoneInput(utmZoneInput);
  }

  template<class AzimuthInput, class FixInput, class UTMZoneInput>
  CompassFilter(rclcpp::Node* node, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput, FixInput& fixInput, UTMZoneInput& utmZoneInput,
    Unit unit, Orientation orientation, Reference reference)
    : CompassFilter(*node, converter, azimuthInput, fixInput, utmZoneInput, unit, orientation, reference)
  {
  }

  /**
   * \brief Construct azimuth filter that can convert all parameters.
   *
   * \tparam AzimuthInput The type of the input filter.
   * \tparam FixInput The type of the navsat fix filter.
   * \param[in] node The node to use.
   * \param[in] converter The azimuth converter instance. If nullptr, a default converter is constructed.
   * \param[in] azimuthInput The message filter producing azimuth messages.
   * \param[in] fixInput The message filter producing fix messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput, class FixInput>
  CompassFilter(RequiredInterfaces node, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput, FixInput& fixInput, Unit unit, Orientation orientation, Reference reference)
    : node(node), converter(converter), unit(unit), orientation(orientation), reference(reference)
  {
    if (this->converter == nullptr)
      this->converter = std::make_shared<CompassConverter>(node, true);
    this->connectAzimuthInput(azimuthInput);
    this->connectFixInput(fixInput);
  }

  template<class AzimuthInput, class FixInput>
  CompassFilter(rclcpp::Node* node, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput, FixInput& fixInput, Unit unit, Orientation orientation, Reference reference)
    : CompassFilter(*node, converter, azimuthInput, fixInput, unit, orientation, reference)
  {
  }

  /**
   * \brief Construct azimuth filter that can only convert units and orientation.
   *
   * \tparam AzimuthInput The type of the input filter.
   * \param[in] node The node to use.
   * \param[in] converter The azimuth converter instance. If nullptr, a default converter is constructed.
   * \param[in] azimuthInput The message filter producing azimuth messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput>
  CompassFilter(const RequiredInterfaces node, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput, Unit unit, Orientation orientation, Reference reference)
    : node(node), converter(converter), unit(unit), orientation(orientation), reference(reference)
  {
    if (this->converter == nullptr)
      this->converter = std::make_shared<CompassConverter>(node, true);
    this->connectAzimuthInput(azimuthInput);
  }

  template<class AzimuthInput>
  CompassFilter(rclcpp::Node* node, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput, Unit unit, Orientation orientation, Reference reference)
    : CompassFilter(*node, converter, azimuthInput, unit, orientation, reference)
  {
  }

  virtual ~CompassFilter();

  template<class AzimuthInput>
  void connectAzimuthInput(AzimuthInput& f)
  {
    this->azimuthConnection.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    this->azimuthConnection = f.registerCallback(
      std::function<void(const AzimuthEventType&)>(std::bind_front(&CompassFilter::cbAzimuth, this)));
  }

  template<class FixInput>
  void connectFixInput(FixInput& f)
  {
    this->fixConnection.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    this->fixConnection = f.registerCallback(
      std::function<void(const FixEventType&)>(std::bind_front(&CompassFilter::cbFix, this)));
  }

  template<class UTMZoneInput>
  void connectUTMZoneInput(UTMZoneInput& f)
  {
    this->utmZoneConnection.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    this->utmZoneConnection = f.registerCallback(
      std::function<void(const UTMZoneEventType&)>(std::bind_front(&CompassFilter::cbUTMZone, this)));
  }

protected:
  virtual void cbAzimuth(const AzimuthEventType& azimuthEvent);
  virtual void cbFix(const FixEventType& fixEvent);
  virtual void cbUTMZone(const UTMZoneEventType& utmZoneEvent);

  message_filters::Connection azimuthConnection;  //!< Connection to the azimuth input.
  message_filters::Connection fixConnection;  //!< Connection to the navsat fix input.
  message_filters::Connection utmZoneConnection;  //!< Connection to the UTM zone input.

  std::shared_ptr<CompassConverter> converter;  //!< The compass converter instance.
  bool fixReceived {false};  //!< Whether at least one navsat fix message has been received.

  Unit unit;  //!< The target azimuth unit.
  Orientation orientation;  //!< The target azimuth orientation.

  //! The target azimuth reference (unchanged if empty).
  std::optional<Reference> reference;
  RequiredInterfaces node;
};

}
