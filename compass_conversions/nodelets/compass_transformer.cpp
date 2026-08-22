// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_interfaces::msg::Azimuth messages.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <stdexcept>
#include <string>

#include <compass_conversions/compass_converter.hpp>
#include <compass_conversions/compass_transformer.hpp>
#include <compass_conversions/message_filter.hpp>
#include <compass_conversions/tf2_compass_msgs.hpp>
#include <compass_conversions/topic_names.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_interfaces/string_utils.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.hpp>

using Az = compass_interfaces::msg::Azimuth;
using Fix = sensor_msgs::msg::NavSatFix;

namespace compass_conversions {

OutputType parseOutputType(const std::string& output_type) {
  const auto output = cras::toLower(output_type);
  if (output == "azimuth") {
    return OutputType::Azimuth;
  } else if (output == "imu") {
    return OutputType::Imu;
  } else if (output == "pose") {
    return OutputType::Pose;
  } else if (output == "quaternion" || output == "quat") {
    return OutputType::Quaternion;
  } else {
    throw std::runtime_error("Unknown output type: " + output_type);
  }
}

std::string outputTypeToString(const OutputType type) {
  switch (type) {
    case OutputType::Azimuth:
      return "azimuth";
    case OutputType::Imu:
      return "imu";
    case OutputType::Pose:
      return "pose";
    case OutputType::Quaternion:
      return "quaternion";
    default:
      throw std::runtime_error(cras::format("Unknown output type: {}", std::to_string((static_cast<int>(type)))));
  }
}

/**
 * \brief Nodelet for transforming one type and parametrization of azimuth to another type, parametrization and
 *        TF frame.
 *
 * Subscribed topics:
 * - `~azimuth_in` (compass_interfaces/msg/Azimuth or geometry_msgs/msg/QuaternionStamped or
 *     geometry_msgs/msg/PoseWithCovarianceStamped or sensor_msgs/msg/Imu): The input azimuth. The name of the topic
 *     (if you remap it) can be used to autodetect some metadata for the conversion.
 *  - `fix` (sensor_msgs/msg/NavSatFix): GNSS fix messages that can be used to determine some parameters for the
 *      conversion.
 *  - `utm_zone` (std_msgs/msg/Int32): Optional messages with forced UTM zone.
 *  - TF (only if `~target_frame` is nonempty)
 *
 *  Published topics:
 *  - `~azimuth_out` or `~azimuth_out/SUFFIX`: The transformed azimuth. If `~target_append_suffix` is true, the variant
 *                                             with topic name suffix will be used (e.g. `~azimuth_out/mag/enu/deg`).
 *                                             The type of the published message is determined by `~target_type`.
 *
 * Parameters:
 * - `~queue_size` (int, default 10): Queue size for the subscribers and publishers.
 * - `~target_unit` (str, 'deg' or 'rad', default: 'rad'): Angular unit to be used in the transformed messages.
 * - `~target_orientation` (str, 'enu' or 'ned', default: 'enu'): ENU or NED orientation to be used in the
 *                                                                transformed messages.
 * - `~target_reference` (str, 'magnetic', 'geographic' or 'UTM', default: 'geographic'): North reference to be used in
 *                                                                                        the transformed messages.
 * - `~target_type` (str, 'azimuth', 'quaternion', 'pose' or 'imu', default 'azimuth'): The Type of output messages.
 * - `~target_append_suffix` (bool, default false): If true, the output topic will be suffixed with a metadata-based
 *                                                  string.
 * - `~target_frame` (str, default: no change): TF frame to transform the messages to. Please note that frames that are
 *                                              too "titled" from gravity will not make much sense.
 * - `~subscribe_fix` (bool, default true): Whether to subscribe `fix` topic. In some cases, you don't need it.
 * - `~subscribe_utm` (bool, default true): Whether to subscribe `utm_zone` topic. It is fully optional.
 * - `~input_orientation` (str, 'enu' or 'ned', default: unspecified): ENU or NED orientation to be used to interpret
 *                                                                     input messages (in case orientation cannot be
 *                                                                     derived either from message contents or topic
 *                                                                     name).
 * - `~input_reference` (str, 'magnetic', 'geographic' or 'UTM', default: no change): North reference to be used to
 *                                                                                    interpret input messages (in case
 *                                                                                    reference cannot be derived either
 *                                                                                    from message contents or topic
 *                                                                                    name).
 * - `~input_variance` (double, optional, rad^2): If specified, this variance will be used in the output messages
 *                                                if variance cannot be determined from the input messages (e.g. for
 *                                                `QuaternionStamped`).
 * - `~strict` (bool, default true): If true, conversions between magnetic and geographic North will fail if the used
 *                                   magnetic model is used outside its declared bounds of validity (mostly year and
 *                                   altitude).
 * - All parameters consumed by `CompassConverter` (most interesting are `initial_lat`, `initial_lon`, that can relieve
 *   this nodelet from subscribing `fix` topic, if you know the approximate coordinates in advance).
 */
CompassTransformerNodelet::CompassTransformerNodelet(const rclcpp::NodeOptions& options)
    : rclcpp::Node("compass_transformer_nodelet", options), target_type_(OutputType::Azimuth) {
  setBuffer(std::make_shared<tf2_ros::Buffer>(get_clock()), true);
}

CompassTransformerNodelet::~CompassTransformerNodelet() {
  listener_.reset();
  buffer_.reset();
  converter_.reset();
}

void CompassTransformerNodelet::init() {
  // CompassConverter params:
  declare_parameter<double>("magnetic_declination", -9999.);
  declare_parameter<std::string>("magnetic_model", std::string());
  declare_parameter<std::string>("magnetic_models_path", std::string());
  declare_parameter<double>("utm_grid_convergence", -1.);
  declare_parameter<int>("utm_zone", -1);
  declare_parameter<bool>("keep_utm_zone", true);
  declare_parameter<double>("initial_lat", -1.);
  declare_parameter<double>("initial_lon", -1.);
  declare_parameter<double>("initial_alt", -1.);
  declare_parameter<bool>("use_wall_time_for_declination", false);
  // UniversalAzimuthSubscriber params:
  declare_parameter<std::string>("input_orientation", std::string());
  declare_parameter<std::string>("input_reference", std::string());
  declare_parameter<double>("input_variance", -1.);
  // Custom params:
  declare_parameter<int>("queue_size", 10);
  declare_parameter<std::string>("target_unit", compass_interfaces::unitToString(Az::UNIT_RAD));
  declare_parameter<std::string>("target_orientation", compass_interfaces::orientationToString(Az::ORIENTATION_ENU));
  declare_parameter<std::string>("target_reference", compass_interfaces::referenceToString(Az::REFERENCE_GEOGRAPHIC));
  declare_parameter<std::string>("target_type", outputTypeToString(target_type_));
  declare_parameter<bool>("target_append_suffix", false);
  declare_parameter<std::string>("target_frame", std::string());
  declare_parameter<std::string>("out_frame_id", std::string());
  declare_parameter<bool>("subscribe_fix", true);
  declare_parameter<bool>("subscribe_utm", true);
  declare_parameter<bool>("strict", true);

  const auto queue_size = get_parameter_or<int>("queue_size", 10);

  const uint8_t target_unit = compass_interfaces::parseUnit(
    get_parameter_or<std::string>("target_unit", compass_interfaces::unitToString(Az::UNIT_RAD)));
  const uint8_t target_orientation = compass_interfaces::parseOrientation(
    get_parameter_or<std::string>("target_orientation", compass_interfaces::orientationToString(Az::ORIENTATION_ENU)));
  const uint8_t target_reference = compass_interfaces::parseReference(
    get_parameter_or<std::string>("target_reference", compass_interfaces::referenceToString(Az::REFERENCE_GEOGRAPHIC)));

  target_type_ = parseOutputType(get_parameter_or<std::string>("target_type", outputTypeToString(target_type_)));

  const auto target_append_suffix = get_parameter_or<bool>("target_append_suffix", false);
  target_frame_ = get_parameter_or<std::string>("target_frame", std::string());
  out_frame_id_ = get_parameter_or<std::string>("out_frame_id", std::string());
  const auto subscribe_fix = get_parameter_or<bool>("subscribe_fix", true);
  const auto subscribe_utm_zone = get_parameter_or<bool>("subscribe_utm", true);

  const auto log = get_logger();
  const auto clock = get_clock();
  converter_ = std::make_shared<CompassConverter>(this, get_parameter_or<bool>("strict", true));
  converter_->configFromParams();

  std::string output_topic_suffix;
  std::string topic_name;
  const rclcpp::SystemDefaultsQoS qos;
  switch (target_type_) {
    case OutputType::Imu:
      output_topic_suffix = getAzimuthTopicSuffix<sensor_msgs::msg::Imu>(
        target_unit, target_orientation, target_reference);
      topic_name = target_append_suffix ? "azimuth_out/" + output_topic_suffix : "azimuth_out";
      pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(topic_name, qos);
    break;
    case OutputType::Pose:
      output_topic_suffix = getAzimuthTopicSuffix<geometry_msgs::msg::PoseWithCovarianceStamped>(
        target_unit, target_orientation, target_reference);
      topic_name = target_append_suffix ? "azimuth_out/" + output_topic_suffix : "azimuth_out";
      pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name, qos);
    break;
    case OutputType::Quaternion:
      output_topic_suffix = getAzimuthTopicSuffix<geometry_msgs::msg::QuaternionStamped>(
        target_unit, target_orientation, target_reference);
      topic_name = target_append_suffix ? "azimuth_out/" + output_topic_suffix : "azimuth_out";
      pub_quat_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(topic_name, qos);
    break;
    default:
      output_topic_suffix = getAzimuthTopicSuffix<Az>(target_unit, target_orientation, target_reference);
      topic_name = target_append_suffix ? "azimuth_out/" + output_topic_suffix : "azimuth_out";
      pub_az_ = create_publisher<Az>(topic_name, qos);
    break;
  }

  azimuth_input_ = std::make_unique<UniversalAzimuthSubscriber>(this, "azimuth_in", queue_size);
  azimuth_input_->configFromParams();

  compass_filter_ = std::make_unique<CompassFilter>(
    this, converter_, *azimuth_input_, target_unit, target_orientation, target_reference);

  if (subscribe_fix) {
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
    const rclcpp::QoS qos(10);
#else
    const rmw_qos_profile_t qos = rclcpp::QoS(10).get_rmw_qos_profile();
#endif
    fix_input_ = std::make_unique<message_filters::Subscriber<Fix>>(this, "gps/fix", qos);
    compass_filter_->connectFixInput(*fix_input_);
  }

  if (subscribe_utm_zone) {
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
    const rclcpp::QoS qos(10);
#else
    const rmw_qos_profile_t qos = rclcpp::QoS(10).get_rmw_qos_profile();
#endif
    utm_zone_input_ = std::make_unique<message_filters::Subscriber<std_msgs::msg::Int32>>(this, "utm_zone", qos);
    compass_filter_->connectUTMZoneInput(*utm_zone_input_);
  }

  if (target_frame_.empty()) {
    compass_filter_->registerCallback(&CompassTransformerNodelet::publish, this);
  } else {
#ifdef TF2_ROS_HAS_NODE_INTERFACES
    tf_filter_ = std::make_unique<tf2_ros::MessageFilter<Az>>(
      *compass_filter_, *buffer_, target_frame_, queue_size, *this, std::chrono::milliseconds(100));
#else
    tf_filter_ = std::make_unique<tf2_ros::MessageFilter<Az>>(
      *compass_filter_, *buffer_, target_frame_, queue_size, get_node_logging_interface(),
      get_node_clock_interface(), std::chrono::milliseconds(100));
#endif
    tf_filter_->registerCallback(&CompassTransformerNodelet::transformAndPublish, this);
    // registerFailureCallback IS CURRENTLY DISABLED IN TF2_ROS FOR "UNKNOWN REASONS" ...
    // tfFilter->registerFailureCallback(std::bind_front(&CompassTransformerNodelet::failedCb, this));
  }

  RCLCPP_INFO(
    log, "Publishing azimuth to topic %s (type %s).", topic_name.c_str(), outputTypeToString(target_type_).c_str());
}

void CompassTransformerNodelet::setBuffer(tf2_ros::Buffer::SharedPtr buffer, const bool using_dedicated_thread) {
  buffer_ = buffer;
  buffer_->setUsingDedicatedThread(using_dedicated_thread);
#ifdef TF2_ROS_HAS_NODE_INTERFACES
  const auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(*this);
#else
  const auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
#endif
  buffer_->setCreateTimerInterface(timer_interface);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, true);
}

void CompassTransformerNodelet::publish(const Az::ConstSharedPtr& msg) {
  switch (target_type_) {
    case OutputType::Imu:
    {
      auto maybe_imu = converter_->convertToImu(*msg);
      if (maybe_imu.has_value()) {
        if (!out_frame_id_.empty()) {
          maybe_imu->header.frame_id = out_frame_id_;
        }
        pub_imu_->publish<sensor_msgs::msg::Imu>(maybe_imu.value());
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000., "%s", maybe_imu.error().c_str());
      }
      break;
    }
    case OutputType::Pose:
    {
      auto maybe_pose = converter_->convertToPose(*msg);
      if (maybe_pose.has_value()) {
        if (!out_frame_id_.empty()) {
          maybe_pose->header.frame_id = out_frame_id_;
        }
        pub_pose_->publish(*maybe_pose);
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000., "%s", maybe_pose.error().c_str());
      }
      break;
    }
    case OutputType::Quaternion:
    {
      auto maybe_quat = converter_->convertToQuaternion(*msg);

      if (maybe_quat.has_value()) {
        if (!out_frame_id_.empty()) {
          maybe_quat->header.frame_id = out_frame_id_;
        }
        pub_quat_->publish(*maybe_quat);
      } else {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000., "%s", maybe_quat.error().c_str());
      }
      break;
    }
    default:
    {
      if (out_frame_id_.empty()) {
        pub_az_->publish(*msg);
      } else {
        auto copy = *msg;
        copy.header.frame_id = out_frame_id_;
        pub_az_->publish(copy);
      }
      break;
    }
  }
}

void CompassTransformerNodelet::transformAndPublish(const Az::ConstSharedPtr& msg) {
  try {
    Az::SharedPtr out_msg(new Az {});
    *out_msg = buffer_->transform(*msg, target_frame_, tf2::durationFromSec(0.1));
    publish(out_msg);
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000., "Azimuth transformation failed: %s", e.what());
  }
}

void CompassTransformerNodelet::failedCb(
    const Az::ConstSharedPtr& msg,
    const tf2_ros::filter_failure_reasons::FilterFailureReason reason) {
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000.,
    "Can't transform incoming Azimuth data to frame %s. Reason %d", target_frame_.c_str(), reason);
}

}  // namespace compass_conversions

RCLCPP_COMPONENTS_REGISTER_NODE(compass_conversions::CompassTransformerNodelet)
