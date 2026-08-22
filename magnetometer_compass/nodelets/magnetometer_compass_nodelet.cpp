// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Compute various azimuth values based on a magnetometer, IMU orientation and possibly also GPS coordinates.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <string>

#include <compass_conversions/compass_converter.hpp>
#include <compass_conversions/topic_names.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <magnetometer_compass/magnetometer_compass.hpp>
#include <magnetometer_compass/magnetometer_compass_nodelet.hpp>
#include <magnetometer_compass/tf2_sensor_msgs.hpp>
#include <magnetometer_pipeline/message_filter.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace magnetometer_compass {

using Az = compass_interfaces::msg::Azimuth;
using Quat = geometry_msgs::msg::QuaternionStamped;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Imu = sensor_msgs::msg::Imu;
using Field = sensor_msgs::msg::MagneticField;

typedef message_filters::sync_policies::ApproximateTime<Imu, Field> SyncPolicy;

MagnetometerCompassNodelet::MagnetometerCompassNodelet(const rclcpp::NodeOptions& options)
    : rclcpp::Node("magnetometer_compass_nodelet", options), buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
      listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_)) {
  buffer_->setUsingDedicatedThread(true);
#ifdef TF2_ROS_HAS_NODE_INTERFACES
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(*this);
#else
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
#endif
  buffer_->setCreateTimerInterface(timer_interface);
}

void MagnetometerCompassNodelet::setBuffer(tf2_ros::Buffer::SharedPtr buffer, const bool using_dedicated_thread) {
  buffer_ = buffer;
  buffer_->setUsingDedicatedThread(using_dedicated_thread);
#ifdef TF2_ROS_HAS_NODE_INTERFACES
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(*this);
#else
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
#endif
  buffer->setCreateTimerInterface(timer_interface);
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
}

void MagnetometerCompassNodelet::init() {
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
  // MagnetometerCompass params:
  declare_parameter<double>("initial_variance", -1.);
  declare_parameter<double>("low_pass_ratio", -1.);
  // Custom params:
  declare_parameter<std::string>("frame", "base_link");
  declare_parameter<bool>("strict", true);
  declare_parameter<bool>("publish_mag_unbiased", publish_mag_unbiased_);
  declare_parameter<bool>("subscribe_mag_unbiased", subscribe_mag_unbiased_);
  // MagnetometerBiasRemover params:
  declare_parameter<double>("initial_mag_bias_x", -1.);
  declare_parameter<double>("initial_mag_bias_y", -1.);
  declare_parameter<double>("initial_mag_bias_z", -1.);
  declare_parameter<std::vector<double>>("initial_mag_scaling_matrix", std::vector<double>(1, 1.0));

  rclcpp::Node::SharedPtr imu_nh = create_sub_node("imu");
  rclcpp::Node::SharedPtr compass_nh = create_sub_node("compass");

  frame_ = get_parameter_or<std::string>("frame", "base_link");

  const auto strict = get_parameter_or<bool>("strict", true);
  converter_ = std::make_shared<compass_conversions::CompassConverter>(this, strict);
  converter_->configFromParams();

  compass_ = std::make_shared<MagnetometerCompass>(*this, frame_, buffer_);
  compass_->configFromParams();

  publish_mag_unbiased_ = get_parameter_or<bool>("publish_mag_unbiased", publish_mag_unbiased_);
  subscribe_mag_unbiased_ = get_parameter_or<bool>("subscribe_mag_unbiased", subscribe_mag_unbiased_);

  if (publish_mag_unbiased_ && subscribe_mag_unbiased_) {
    throw std::runtime_error("Cannot simultaneously subscribe and publish unbiased magnetometer.");
  }

  // Set default publishers
  mag_publishers_.ned_.publish_deg_ = true;

  bool publish = publish_mag_unbiased_;

  // TODO: do not pass two nodes to these, but just one (once the sub_nodes inherit parameters (in jazzy they don't)...)
  mag_publishers_.init(compass_nh, this, converter_, "publish", "", Az::REFERENCE_MAGNETIC, "mag");
  publish |= mag_publishers_.publish_;
  true_publishers_.init(compass_nh, this, converter_, "publish", "", Az::REFERENCE_GEOGRAPHIC, "true");
  publish |= true_publishers_.publish_;
  utm_publishers_.init(compass_nh, this, converter_, "publish", "", Az::REFERENCE_UTM, "utm");
  publish |= utm_publishers_.publish_;

  if (!publish) {
    RCLCPP_WARN(
      get_logger(), "No publishers have been requested. Please, set one of the publish_* parameters to true.");
  }

  if (publish_mag_unbiased_) {
    mag_unbiased_pub_ = imu_nh->create_publisher<Field>("mag_unbiased", rclcpp::SystemDefaultsQoS());
  }

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  rclcpp::QoS imu_qos(100);
  rclcpp::QoS mag_qos(100);
  rclcpp::QoS bias_qos(10);
#else
  rmw_qos_profile_t imu_qos = rclcpp::QoS(100).get_rmw_qos_profile();
  rmw_qos_profile_t mag_qos = rclcpp::QoS(100).get_rmw_qos_profile();
  rmw_qos_profile_t bias_qos = rclcpp::QoS(10).get_rmw_qos_profile();
#endif
  // Until https://github.com/ros2/message_filters/issues/227 is resolved, message_filters::Subscriber() doesn't
  // handle sub-namespaces correctly and we have to handle them explicitly here.
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  const std::string imu_ns = imu_nh->get_sub_namespace() + "/";
#else
  const std::string imu_ns = "";
#endif
  imu_sub_ = std::make_unique<message_filters::Subscriber<Imu>>(imu_nh, imu_ns + "data", imu_qos);

  // Check if we should try to unbias the magnetometer ourselves or if you already got it unbiased on the input.
  if (subscribe_mag_unbiased_) {
    mag_sub_ = std::make_unique<message_filters::Subscriber<Field>>(imu_nh, imu_ns + "mag_unbiased", mag_qos);
    sync_sub_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *imu_sub_, *mag_sub_);
  } else {
    mag_sub_ = std::make_unique<message_filters::Subscriber<Field>>(imu_nh, imu_ns + "mag", mag_qos);
    mag_bias_sub_ = std::make_unique<message_filters::Subscriber<Field>>(imu_nh, imu_ns + "mag_bias", bias_qos);
    mag_bias_remover_filter_ = std::make_unique<magnetometer_pipeline::BiasRemoverFilter>(
      *this, *mag_sub_, *mag_bias_sub_);
    mag_bias_remover_filter_->configFromParams();

    sync_sub_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *imu_sub_, *mag_bias_remover_filter_);
  }
  sync_sub_->registerCallback(&MagnetometerCompassNodelet::imuMagCb, this);

  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix& msg) { fixCb(msg); });
}

MagnetometerCompassNodelet::~MagnetometerCompassNodelet() = default;

AzimuthPublishersConfigForOrientation::AzimuthPublishersConfigForOrientation() = default;

void AzimuthPublishersConfigForOrientation::init(
    rclcpp::Node::SharedPtr namespace_node, rclcpp::Node* param_node,
    const std::shared_ptr<compass_conversions::CompassConverter>& converter,
    const std::string& param_prefix, const std::string& topic_prefix, const uint8_t reference,
    const uint8_t orientation, const std::string& reference_str, const std::string& orientation_str) {
  namespace_node_ = namespace_node;
  param_node_ = param_node;
  converter_ = converter;

  auto prefix = param_prefix + "_" + reference_str + "_azimuth_" + orientation_str + "_";

  param_node->declare_parameter<bool>(prefix + "quat", publish_quat_);
  param_node->declare_parameter<bool>(prefix + "imu", publish_imu_);
  param_node->declare_parameter<bool>(prefix + "pose", publish_pose_);
  param_node->declare_parameter<bool>(prefix + "rad", publish_rad_);
  param_node->declare_parameter<bool>(prefix + "deg", publish_deg_);

  publish_quat_ = param_node->get_parameter_or<bool>(prefix + "quat", publish_quat_);
  publish_imu_ = param_node->get_parameter_or<bool>(prefix + "imu", publish_imu_);
  publish_pose_ = param_node->get_parameter_or<bool>(prefix + "pose", publish_pose_);
  publish_rad_ = param_node->get_parameter_or<bool>(prefix + "rad", publish_rad_);
  publish_deg_ = param_node->get_parameter_or<bool>(prefix + "deg", publish_deg_);
  publish_ = publish_quat_ || publish_imu_ || publish_pose_ || publish_deg_ || publish_rad_;

  using compass_conversions::getAzimuthTopicSuffix;

  prefix = cras::appendIfNonEmpty(topic_prefix, "/");

  if (publish_quat_) {
    quat_pub_ = namespace_node->create_publisher<Quat>(
      prefix + getAzimuthTopicSuffix<Quat>(Az::UNIT_RAD, orientation, reference), rclcpp::SystemDefaultsQoS());
  }
  if (publish_imu_) {
    imu_pub_ = namespace_node->create_publisher<Imu>(
      prefix + getAzimuthTopicSuffix<Imu>(Az::UNIT_RAD, orientation, reference), rclcpp::SystemDefaultsQoS());
  }
  if (publish_pose_) {
    pose_pub_ = namespace_node->create_publisher<Pose>(
      prefix + getAzimuthTopicSuffix<Pose>(Az::UNIT_RAD, orientation, reference), rclcpp::SystemDefaultsQoS());
  }
  if (publish_rad_) {
    rad_pub_ = namespace_node->create_publisher<Az>(
      prefix + getAzimuthTopicSuffix<Az>(Az::UNIT_RAD, orientation, reference), rclcpp::SystemDefaultsQoS());
  }
  if (publish_deg_) {
    deg_pub_ = namespace_node->create_publisher<Az>(
      prefix + getAzimuthTopicSuffix<Az>(Az::UNIT_DEG, orientation, reference), rclcpp::SystemDefaultsQoS());
  }
}

AzimuthPublishersConfig::AzimuthPublishersConfig() = default;

void AzimuthPublishersConfig::init(
    rclcpp::Node::SharedPtr namespace_node, rclcpp::Node* param_node,
    const std::shared_ptr<compass_conversions::CompassConverter>& converter,
    const std::string& param_prefix, const std::string& topic_prefix,
    const uint8_t reference, const std::string& reference_str) {
  namespace_node_ = namespace_node;
  param_node_ = param_node;
  converter_ = converter;
  ned_.init(
    namespace_node, param_node, converter, param_prefix, topic_prefix, reference,
    Az::ORIENTATION_NED, reference_str, "ned");
  enu_.init(
    namespace_node, param_node, converter, param_prefix, topic_prefix, reference,
    Az::ORIENTATION_ENU, reference_str, "enu");
  publish_ = ned_.publish_ || enu_.publish_;
}

void MagnetometerCompassNodelet::imuMagCb(const Imu& imu, const Field& mag_unbiased) {
  if (publish_mag_unbiased_) {
    mag_unbiased_pub_->publish(mag_unbiased);
  }

  const auto maybe_azimuth = compass_->computeAzimuth(imu, mag_unbiased);
  if (!maybe_azimuth.has_value()) {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000., "%s", maybe_azimuth.error().c_str());
    return;
  }

  Imu imu_in_body;
  try {
    // No timeout because computeAzimuth() has already waited for this exact transform
    buffer_->transform(imu, imu_in_body, frame_);
  } catch (const tf2::TransformException& e) {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000.,
      "Could not transform IMU data to frame %s because: %s", frame_.c_str(), e.what());
    return;
  }

  const auto& ned_azimuth_msg = *maybe_azimuth;
  mag_publishers_.publishAzimuths(ned_azimuth_msg, imu_in_body);

  if (true_publishers_.publish_) {
    const auto maybe_true_ned_azimuth_msg = converter_->convertAzimuth(
      ned_azimuth_msg, ned_azimuth_msg.unit, ned_azimuth_msg.orientation, Az::REFERENCE_GEOGRAPHIC);
    if (maybe_true_ned_azimuth_msg) {
      true_publishers_.publishAzimuths(*maybe_true_ned_azimuth_msg, imu_in_body);
    } else {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000.,
        "%s", maybe_true_ned_azimuth_msg.error().c_str());
    }
  }

  if (utm_publishers_.publish_) {
    const auto maybe_utm_ned_azimuth_msg = converter_->convertAzimuth(
      ned_azimuth_msg, ned_azimuth_msg.unit, ned_azimuth_msg.orientation, Az::REFERENCE_UTM);
    if (maybe_utm_ned_azimuth_msg.has_value()) {
      utm_publishers_.publishAzimuths(*maybe_utm_ned_azimuth_msg, imu_in_body);
    } else {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000., "%s",
        maybe_utm_ned_azimuth_msg.error().c_str());
    }
  }
}

void AzimuthPublishersConfig::publishAzimuths(const Az& ned_azimuth, const Imu& imu_in_body) {
  if (!publish_) {
    return;
  }

  if (ned_.publish_) {
    auto imu_ned = imu_in_body;  // If IMU message should not be published, we fake it here with the ENU-referenced one
    if (ned_.publish_imu_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = imu_in_body.header.stamp;
      tf.header.frame_id = imu_in_body.header.frame_id + "_ned";
      tf2::convert(enu_to_ned_, tf.transform.rotation);
      tf2::doTransform(imu_in_body, imu_ned, tf);
    }
    ned_.publishAzimuths(ned_azimuth, imu_ned);
  }

  if (enu_.publish_) {
    // Rotate to ENU
    auto maybe_enu_azimuth = converter_->convertAzimuth(
      ned_azimuth, ned_azimuth.unit, Az::ORIENTATION_ENU, ned_azimuth.reference);

    if (maybe_enu_azimuth.has_value()) {
      enu_.publishAzimuths(*maybe_enu_azimuth, imu_in_body);
    } else {
      RCLCPP_ERROR_THROTTLE(namespace_node_->get_logger(), *namespace_node_->get_clock(), 1000.,
                            "Could not convert from NED to ENU: %s", maybe_enu_azimuth.error().c_str());
    }
  }
}

void AzimuthPublishersConfigForOrientation::publishAzimuths(const Az& azimuth_rad, const Imu& imu_in_body) {
  if (publish_quat_) {
    const auto maybe_quat = converter_->convertToQuaternion(azimuth_rad);
    if (maybe_quat.has_value()) {
      quat_pub_->publish(*maybe_quat);
    } else {
      RCLCPP_ERROR_THROTTLE(
        namespace_node_->get_logger(), *namespace_node_->get_clock(), 1000., "%s", maybe_quat.error().c_str());
    }
  }

  if (publish_imu_) {
    const auto maybe_quat = converter_->convertToQuaternion(azimuth_rad);
    if (!maybe_quat.has_value()) {
      RCLCPP_ERROR_THROTTLE(
        namespace_node_->get_logger(), *namespace_node_->get_clock(), 1000., "%s", maybe_quat.error().c_str());
    } else {
      // The IMU message comes in an arbitrarily-referenced frame, and we adjust its yaw to become georeferenced.
      double azimuth_yaw = cras::getYaw(maybe_quat->quaternion);

      tf2::Quaternion imu_rot;
      tf2::convert(imu_in_body.orientation, imu_rot);
      double roll, pitch, yaw;
      cras::getRPY(imu_rot, roll, pitch, yaw);

      tf2::Quaternion desired_rot;
      desired_rot.setRPY(roll, pitch, azimuth_yaw);

      const auto diff_rot = desired_rot.inverse() * imu_rot;

      sensor_msgs::msg::Imu imu_msg;

      geometry_msgs::msg::TransformStamped tf;
      tf.header = imu_in_body.header;
      tf2::convert(diff_rot, tf.transform.rotation);
      tf2::doTransform(imu_in_body, imu_msg, tf);

      imu_msg.orientation_covariance[8] = azimuth_rad.variance;

      imu_pub_->publish(imu_msg);
    }
  }

  if (publish_pose_) {
    const auto maybe_pose = converter_->convertToPose(azimuth_rad);
    if (maybe_pose.has_value()) {
      pose_pub_->publish(*maybe_pose);
    } else {
      RCLCPP_ERROR_THROTTLE(
        namespace_node_->get_logger(), *namespace_node_->get_clock(), 1000., "%s", maybe_pose.error().c_str());
    }
  }

  if (publish_rad_) {
    rad_pub_->publish(azimuth_rad);
  }

  if (publish_deg_) {
    const auto maybe_azimuth_deg = converter_->convertAzimuth(
      azimuth_rad, Az::UNIT_DEG, azimuth_rad.orientation, azimuth_rad.reference);
    if (maybe_azimuth_deg.has_value()) {
      deg_pub_->publish(*maybe_azimuth_deg);
    } else {
      RCLCPP_ERROR_THROTTLE(
        namespace_node_->get_logger(), *namespace_node_->get_clock(), 1000., "%s", maybe_azimuth_deg.error().c_str());
    }
  }
}

void MagnetometerCompassNodelet::fixCb(const sensor_msgs::msg::NavSatFix& fix) {
  converter_->setNavSatPos(fix);
}

}  // namespace magnetometer_compass

RCLCPP_COMPONENTS_REGISTER_NODE(magnetometer_compass::MagnetometerCompassNodelet)
