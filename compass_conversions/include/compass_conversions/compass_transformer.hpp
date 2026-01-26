#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_interfaces::msg::Azimuth messages.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <string>

#include <compass_conversions/compass_converter.hpp>
#include <compass_conversions/message_filter.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace compass_conversions
{

enum class OutputType
{
  Azimuth,
  Imu,
  Pose,
  Quaternion,
};

OutputType parseOutputType(const std::string& outputType);
std::string outputTypeToString(OutputType type);

class CompassTransformerNodelet : public rclcpp::Node
{
public:
  explicit CompassTransformerNodelet(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CompassTransformerNodelet() override;

  void init();
  void setBuffer(tf2_ros::Buffer::SharedPtr buffer, bool using_dedicated_thread);

protected:
  void publish(const compass_interfaces::msg::Azimuth::ConstSharedPtr& msg);
  void transformAndPublish(const compass_interfaces::msg::Azimuth::ConstSharedPtr& msg);
  void failedCb(const compass_interfaces::msg::Azimuth::ConstSharedPtr& msg,
                tf2_ros::filter_failure_reasons::FilterFailureReason reason);

  std::shared_ptr<CompassConverter> converter;
  std::unique_ptr<UniversalAzimuthSubscriber> azimuthInput;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> fixInput;
  std::unique_ptr<message_filters::Subscriber<std_msgs::msg::Int32>> utmZoneInput;
  std::unique_ptr<CompassFilter> compassFilter;
  std::unique_ptr<tf2_ros::MessageFilter<compass_interfaces::msg::Azimuth>> tfFilter;

  rclcpp::Publisher<compass_interfaces::msg::Azimuth>::SharedPtr pub_az;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_quat;

  std::string targetFrame;
  std::string outFrameId;
  OutputType targetType {OutputType::Azimuth};
  tf2_ros::Buffer::SharedPtr buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
};

}
