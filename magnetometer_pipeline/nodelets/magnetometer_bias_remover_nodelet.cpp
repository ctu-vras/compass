// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Remove known bias from 3-axis magnetometer.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <functional>
#include <memory>
#include <vector>

#include <magnetometer_pipeline/magnetometer_bias_remover_nodelet.hpp>
#include <magnetometer_pipeline/message_filter.hpp>
#include <message_filters/subscriber.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace magnetometer_pipeline {

using Field = sensor_msgs::msg::MagneticField;

MagnetometerBiasRemoverNodelet::MagnetometerBiasRemoverNodelet(const rclcpp::NodeOptions& options)
    : rclcpp::Node("magnetometer_bias_remover_nodelet", options) {
  onInit();
}

void MagnetometerBiasRemoverNodelet::onInit() {
  declare_parameter<double>("initial_mag_bias_x", -1.);
  declare_parameter<double>("initial_mag_bias_y", -1.);
  declare_parameter<double>("initial_mag_bias_z", -1.);
  declare_parameter<std::vector<double>>("initial_mag_scaling_matrix", std::vector<double>(1, 1.0));

  rclcpp::Node::SharedPtr topic_nh = create_sub_node("imu");

  mag_unbiased_pub_ = topic_nh->create_publisher<Field>("mag_unbiased", 10);

  // Until https://github.com/ros2/message_filters/issues/227 is resolved, message_filters::Subscriber() doesn't
  // handle sub-namespaces correctly and we have to handle them explicitly here.
#if MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES
  const std::string mag_ns = topic_nh->get_sub_namespace() + "/";
  rclcpp::QoS mag_qos(100);
  rclcpp::QoS mag_bias_qos(10);
#else
  const std::string mag_ns = "";
  rmw_qos_profile_t mag_qos = rclcpp::QoS(100).get_rmw_qos_profile();
  rmw_qos_profile_t mag_bias_qos = rclcpp::QoS(10).get_rmw_qos_profile();
#endif
  mag_sub_ = std::make_unique<message_filters::Subscriber<Field>>(topic_nh, mag_ns + "mag", mag_qos);
  mag_bias_sub_ = std::make_unique<message_filters::Subscriber<Field>>(topic_nh, mag_ns + "mag_bias", mag_bias_qos);

  remover_ = std::make_unique<BiasRemoverFilter>(*this, *mag_sub_, *mag_bias_sub_);
  remover_->configFromParams();
  remover_->registerCallback(
    std::function<void(const Field&)>(std::bind_front(&MagnetometerBiasRemoverNodelet::cb, this)));
}

void MagnetometerBiasRemoverNodelet::cb(const Field& msg) {
  mag_unbiased_pub_->publish(msg);
}

MagnetometerBiasRemoverNodelet::~MagnetometerBiasRemoverNodelet() = default;

}  // namespace magnetometer_pipeline

RCLCPP_COMPONENTS_REGISTER_NODE(magnetometer_pipeline::MagnetometerBiasRemoverNodelet)
