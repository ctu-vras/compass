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

namespace magnetometer_compass
{

using Az = compass_interfaces::msg::Azimuth;
using Quat = geometry_msgs::msg::QuaternionStamped;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Imu = sensor_msgs::msg::Imu;
using Field = sensor_msgs::msg::MagneticField;

typedef message_filters::sync_policies::ApproximateTime<Imu, Field> SyncPolicy;

MagnetometerCompassNodelet::MagnetometerCompassNodelet(const rclcpp::NodeOptions& options)
  : rclcpp::Node("magnetometer_compass_nodelet", options),
    buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    listener(std::make_shared<tf2_ros::TransformListener>(*buffer))
{
  this->buffer->setUsingDedicatedThread(true);
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  buffer->setCreateTimerInterface(timer_interface);
}

void MagnetometerCompassNodelet::setBuffer(tf2_ros::Buffer::SharedPtr buffer, const bool using_dedicated_thread)
{
  this->buffer = buffer;
  this->buffer->setUsingDedicatedThread(using_dedicated_thread);
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  buffer->setCreateTimerInterface(timer_interface);
  this->listener = std::make_shared<tf2_ros::TransformListener>(*this->buffer);
}

void MagnetometerCompassNodelet::init()
{
  // CompassConverter params:
  this->declare_parameter<double>("magnetic_declination", -9999.);
  this->declare_parameter<std::string>("magnetic_model", std::string());
  this->declare_parameter<std::string>("magnetic_models_path", std::string());
  this->declare_parameter<double>("utm_grid_convergence", -1.);
  this->declare_parameter<int>("utm_zone", -1);
  this->declare_parameter<bool>("keep_utm_zone", true);
  this->declare_parameter<double>("initial_lat", -1.);
  this->declare_parameter<double>("initial_lon", -1.);
  this->declare_parameter<double>("initial_alt", -1.);
  this->declare_parameter<bool>("use_wall_time_for_declination", false);
  // MagnetometerCompass params:
  this->declare_parameter<double>("initial_variance", -1.);
  this->declare_parameter<double>("low_pass_ratio", -1.);
  // Custom params:
  this->declare_parameter<std::string>("frame", "base_link");
  this->declare_parameter<bool>("strict", true);
  this->declare_parameter<bool>("publish_mag_unbiased", this->publishMagUnbiased);
  this->declare_parameter<bool>("subscribe_mag_unbiased", this->subscribeMagUnbiased);
  // MagnetometerBiasRemover params:
  this->declare_parameter<double>("initial_mag_bias_x", -1.);
  this->declare_parameter<double>("initial_mag_bias_y", -1.);
  this->declare_parameter<double>("initial_mag_bias_z", -1.);
  this->declare_parameter<std::vector<double>>("initial_mag_scaling_matrix", std::vector<double>(1, 1.0));

  rclcpp::Node::SharedPtr imuNh = this->create_sub_node("imu");
  rclcpp::Node::SharedPtr compassNh = this->create_sub_node("compass");

  this->frame = this->get_parameter_or<std::string>("frame", "base_link");

  const auto strict = this->get_parameter_or<bool>("strict", true);
  this->converter = std::make_shared<compass_conversions::CompassConverter>(this, strict);
  this->converter->configFromParams();

  this->compass = std::make_shared<MagnetometerCompass>(*this, this->frame, this->buffer);
  this->compass->configFromParams();

  this->publishMagUnbiased = this->get_parameter_or<bool>("publish_mag_unbiased", this->publishMagUnbiased);
  this->subscribeMagUnbiased = this->get_parameter_or<bool>("subscribe_mag_unbiased", this->subscribeMagUnbiased);

  if (this->publishMagUnbiased && this->subscribeMagUnbiased)
    throw std::runtime_error("Cannot simultaneously subscribe and publish unbiased magnetometer.");

  // Set default publishers
  this->magPublishers.ned.publishDeg = true;

  bool publish = this->publishMagUnbiased;

  // TODO: do not pass two nodes to these, but just one (once the sub_nodes inherit parameters (in jazzy they don't)...)
  this->magPublishers.init(compassNh, this, this->converter, "publish", "", Az::REFERENCE_MAGNETIC, "mag");
  publish |= this->magPublishers.publish;
  this->truePublishers.init(compassNh, this, this->converter, "publish", "", Az::REFERENCE_GEOGRAPHIC, "true");
  publish |= this->truePublishers.publish;
  this->utmPublishers.init(compassNh, this, this->converter, "publish", "", Az::REFERENCE_UTM, "utm");
  publish |= this->utmPublishers.publish;

  if (!publish)
    RCLCPP_WARN(this->get_logger(),
      "No publishers have been requested. Please, set one of the publish_* parameters to true.");

  if (this->publishMagUnbiased)
    this->magUnbiasedPub = imuNh->create_publisher<Field>("mag_unbiased", rclcpp::SystemDefaultsQoS());

#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  rclcpp::QoS imuQos(100);
  rclcpp::QoS magQos(100);
  rclcpp::QoS biasQos(10);
#else
  rmw_qos_profile_t imuQos = rclcpp::QoS(100).get_rmw_qos_profile();
  rmw_qos_profile_t magQos = rclcpp::QoS(100).get_rmw_qos_profile();
  rmw_qos_profile_t biasQos = rclcpp::QoS(10).get_rmw_qos_profile();
#endif
  // Until https://github.com/ros2/message_filters/issues/227 is resolved, message_filters::Subscriber() doesn't
  // handle sub-namespaces correctly and we have to handle them explicitly here.
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  const std::string imuNs = imuNh->get_sub_namespace() + "/";
#else
  const std::string imuNs = "";
#endif
  this->imuSub = std::make_unique<message_filters::Subscriber<Imu>>(imuNh, imuNs + "data", imuQos);

  // Check if we should try to unbias the magnetometer ourselves or if you already got it unbiased on the input.
  if (this->subscribeMagUnbiased)
  {
    this->magSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, imuNs + "mag_unbiased", magQos);
    this->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *this->imuSub, *this->magSub);
  }
  else
  {
    this->magSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, imuNs + "mag", magQos);
    this->magBiasSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, imuNs + "mag_bias", biasQos);
    this->magBiasRemoverFilter = std::make_unique<magnetometer_pipeline::BiasRemoverFilter>(
      *this, *this->magSub, *this->magBiasSub);
    this->magBiasRemoverFilter->configFromParams();

    this->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *this->imuSub, *this->magBiasRemoverFilter);
  }
  this->syncSub->registerCallback(&MagnetometerCompassNodelet::imuMagCb, this);

  this->fixSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::NavSatFix& msg) { this->fixCb(msg); });
}

MagnetometerCompassNodelet::~MagnetometerCompassNodelet() = default;

AzimuthPublishersConfigForOrientation::AzimuthPublishersConfigForOrientation() = default;

void AzimuthPublishersConfigForOrientation::init(
  rclcpp::Node::SharedPtr namespace_node, rclcpp::Node* param_node,
  const std::shared_ptr<compass_conversions::CompassConverter>& converter,
  const std::string& paramPrefix, const std::string& topicPrefix, const uint8_t reference, const uint8_t orientation,
  const std::string& referenceStr, const std::string& orientationStr)
{
  this->namespace_node = namespace_node;
  this->param_node = param_node;
  this->converter = converter;

  auto prefix = paramPrefix + "_" + referenceStr + "_azimuth_" + orientationStr + "_";

  param_node->declare_parameter<bool>(prefix + "quat", this->publishQuat);
  param_node->declare_parameter<bool>(prefix + "imu", this->publishImu);
  param_node->declare_parameter<bool>(prefix + "pose", this->publishPose);
  param_node->declare_parameter<bool>(prefix + "rad", this->publishRad);
  param_node->declare_parameter<bool>(prefix + "deg", this->publishDeg);

  this->publishQuat = param_node->get_parameter_or<bool>(prefix + "quat", this->publishQuat);
  this->publishImu = param_node->get_parameter_or<bool>(prefix + "imu", this->publishImu);
  this->publishPose = param_node->get_parameter_or<bool>(prefix + "pose", this->publishPose);
  this->publishRad = param_node->get_parameter_or<bool>(prefix + "rad", this->publishRad);
  this->publishDeg = param_node->get_parameter_or<bool>(prefix + "deg", this->publishDeg);
  this->publish = this->publishQuat || this->publishImu || this->publishPose || this->publishDeg || this->publishRad;

  using compass_conversions::getAzimuthTopicSuffix;

  prefix = cras::appendIfNonEmpty(topicPrefix, "/");

  if (this->publishQuat)
    this->quatPub = namespace_node->create_publisher<Quat>(
      prefix + getAzimuthTopicSuffix<Quat>(Az::UNIT_RAD, orientation, reference), rclcpp::SystemDefaultsQoS());
  if (this->publishImu)
    this->imuPub = namespace_node->create_publisher<Imu>(
      prefix + getAzimuthTopicSuffix<Imu>(Az::UNIT_RAD, orientation, reference), rclcpp::SystemDefaultsQoS());
  if (this->publishPose)
    this->posePub = namespace_node->create_publisher<Pose>(
      prefix + getAzimuthTopicSuffix<Pose>(Az::UNIT_RAD, orientation, reference), rclcpp::SystemDefaultsQoS());
  if (this->publishRad)
    this->radPub = namespace_node->create_publisher<Az>(
      prefix + getAzimuthTopicSuffix<Az>(Az::UNIT_RAD, orientation, reference), rclcpp::SystemDefaultsQoS());
  if (this->publishDeg)
    this->degPub = namespace_node->create_publisher<Az>(
      prefix + getAzimuthTopicSuffix<Az>(Az::UNIT_DEG, orientation, reference), rclcpp::SystemDefaultsQoS());
}

AzimuthPublishersConfig::AzimuthPublishersConfig() = default;

void AzimuthPublishersConfig::init(
  rclcpp::Node::SharedPtr namespace_node, rclcpp::Node* param_node,
  const std::shared_ptr<compass_conversions::CompassConverter>& converter,
  const std::string& paramPrefix, const std::string& topicPrefix,
  const uint8_t reference, const std::string& referenceStr)
{
  this->namespace_node = namespace_node;
  this->param_node = param_node;
  this->converter = converter;
  this->ned.init(namespace_node, param_node, converter, paramPrefix, topicPrefix, reference,
    Az::ORIENTATION_NED, referenceStr, "ned");
  this->enu.init(namespace_node, param_node, converter, paramPrefix, topicPrefix, reference,
    Az::ORIENTATION_ENU, referenceStr, "enu");
  this->publish = this->ned.publish || this->enu.publish;
}

void MagnetometerCompassNodelet::imuMagCb(const Imu& imu, const Field& magUnbiased)
{
  if (this->publishMagUnbiased)
    this->magUnbiasedPub->publish(magUnbiased);

  const auto maybeAzimuth = this->compass->computeAzimuth(imu, magUnbiased);
  if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000., "%s", maybeAzimuth.error().c_str());
    return;
  }

  Imu imuInBody;
  try
  {
    // No timeout because computeAzimuth() has already waited for this exact transform
    this->buffer->transform(imu, imuInBody, this->frame);
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000.,
      "Could not transform IMU data to frame %s because: %s", this->frame.c_str(), e.what());
    return;
  }

  const auto& nedAzimuthMsg = *maybeAzimuth;
  this->magPublishers.publishAzimuths(nedAzimuthMsg, imuInBody);

  if (this->truePublishers.publish)
  {
    const auto maybeTrueNedAzimuthMsg = this->converter->convertAzimuth(
      nedAzimuthMsg, nedAzimuthMsg.unit, nedAzimuthMsg.orientation, Az::REFERENCE_GEOGRAPHIC);
    if (maybeTrueNedAzimuthMsg)
    {
      this->truePublishers.publishAzimuths(*maybeTrueNedAzimuthMsg, imuInBody);
    }
    else
    {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000.,
        "%s", maybeTrueNedAzimuthMsg.error().c_str());
    }
  }

  if (this->utmPublishers.publish)
  {
    const auto maybeUTMNedAzimuthMsg = this->converter->convertAzimuth(
      nedAzimuthMsg, nedAzimuthMsg.unit, nedAzimuthMsg.orientation, Az::REFERENCE_UTM);
    if (maybeUTMNedAzimuthMsg.has_value())
    {
      this->utmPublishers.publishAzimuths(*maybeUTMNedAzimuthMsg, imuInBody);
    }
    else
    {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000., "%s",
        maybeUTMNedAzimuthMsg.error().c_str());
    }
  }
}

void AzimuthPublishersConfig::publishAzimuths(const Az& nedAzimuth, const Imu& imuInBody)
{
  if (!this->publish)
    return;

  if (this->ned.publish)
  {
    auto imuNed = imuInBody;  // If IMU message should not be published, we fake it here with the ENU-referenced one
    if (this->ned.publishImu)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = imuInBody.header.stamp;
      tf.header.frame_id = imuInBody.header.frame_id + "_ned";
      tf2::convert(this->enuToNed, tf.transform.rotation);
      tf2::doTransform(imuInBody, imuNed, tf);
    }
    this->ned.publishAzimuths(nedAzimuth, imuNed);
  }

  if (this->enu.publish)
  {
    // Rotate to ENU
    auto maybeEnuAzimuth = this->converter->convertAzimuth(
      nedAzimuth, nedAzimuth.unit, Az::ORIENTATION_ENU, nedAzimuth.reference);

    if (maybeEnuAzimuth.has_value())
      this->enu.publishAzimuths(*maybeEnuAzimuth, imuInBody);
    else
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000.,
        "Could not convert from NED to ENU: %s", maybeEnuAzimuth.error().c_str());
  }
}

void AzimuthPublishersConfigForOrientation::publishAzimuths(const Az& azimuthRad, const Imu& imuInBody)
{
  if (this->publishQuat)
  {
    const auto maybeQuat = this->converter->convertToQuaternion(azimuthRad);
    if (maybeQuat.has_value())
      this->quatPub->publish(*maybeQuat);
    else
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000.,
        "%s", maybeQuat.error().c_str());
  }

  if (this->publishImu)
  {
    const auto maybeQuat = this->converter->convertToQuaternion(azimuthRad);
    if (!maybeQuat.has_value())
    {
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000.,
        "%s", maybeQuat.error().c_str());
    }
    else
    {
      // The IMU message comes in an arbitrarily-referenced frame, and we adjust its yaw to become georeferenced.
      double azimuthYaw = cras::getYaw(maybeQuat->quaternion);

      tf2::Quaternion imuRot;
      tf2::convert(imuInBody.orientation, imuRot);
      double roll, pitch, yaw;
      cras::getRPY(imuRot, roll, pitch, yaw);

      tf2::Quaternion desiredRot;
      desiredRot.setRPY(roll, pitch, azimuthYaw);

      const auto diffRot = desiredRot.inverse() * imuRot;

      sensor_msgs::msg::Imu imuMsg;

      geometry_msgs::msg::TransformStamped tf;
      tf.header = imuInBody.header;
      tf2::convert(diffRot, tf.transform.rotation);
      tf2::doTransform(imuInBody, imuMsg, tf);

      imuMsg.orientation_covariance[8] = azimuthRad.variance;

      this->imuPub->publish(imuMsg);
    }
  }

  if (this->publishPose)
  {
    const auto maybePose = this->converter->convertToPose(azimuthRad);
    if (maybePose.has_value())
      this->posePub->publish(*maybePose);
    else
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000.,
        "%s", maybePose.error().c_str());
  }

  if (this->publishRad)
  {
    this->radPub->publish(azimuthRad);
  }

  if (this->publishDeg)
  {
    const auto maybeAzimuthDeg = this->converter->convertAzimuth(
      azimuthRad, Az::UNIT_DEG, azimuthRad.orientation, azimuthRad.reference);
    if (maybeAzimuthDeg.has_value())
      this->degPub->publish(*maybeAzimuthDeg);
    else
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000.,
        "%s", maybeAzimuthDeg.error().c_str());
  }
}

void MagnetometerCompassNodelet::fixCb(const sensor_msgs::msg::NavSatFix& fix)
{
  this->converter->setNavSatPos(fix);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(magnetometer_compass::MagnetometerCompassNodelet)
