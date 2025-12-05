// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for azimuth visualization.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/test_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <magnetometer_compass/visualize_azimuth_nodelet.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using Az = compass_interfaces::msg::Azimuth;
using Quat = geometry_msgs::msg::QuaternionStamped;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Fix = sensor_msgs::msg::NavSatFix;

using namespace std::chrono_literals;

class VisualizeAzimuthNodelet : public cras::RclcppTestFixture {};

std::shared_ptr<magnetometer_compass::VisualizeAzimuthNodelet> createNodelet(
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
{
  return std::make_shared<magnetometer_compass::VisualizeAzimuthNodelet>(node_options);
}

TEST_F(VisualizeAzimuthNodelet, Basic)  // NOLINT
{
  auto node = createNodelet();
  node->init();

  std::optional<Pose> lastPose;
  auto poseCb = [&lastPose](const Pose::ConstSharedPtr& msg)
  {
    lastPose = *msg;
  };

  auto sub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
  auto pub_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default);
  size_t dep = 1;
  pub_qos.depth = dep;
  sub_qos.depth = dep;

  std::list<rclcpp::PublisherBase::SharedPtr> pubs;
  auto azPub = node->create_publisher<Az>("visualize_azimuth/azimuth", rclcpp::SystemDefaultsQoS(pub_qos));
  pubs.push_back(azPub);
  auto fixPub = node->create_publisher<Fix>("gps/fix", rclcpp::SystemDefaultsQoS(pub_qos).transient_local());
  pubs.push_back(fixPub);

  std::list<rclcpp::SubscriptionBase::SharedPtr> subs;
  rclcpp::SensorDataQoS qos(sub_qos);
  auto visSub = node->create_subscription<Pose>("visualize_azimuth/azimuth_vis", qos, poseCb); subs.push_back(visSub);

  const auto pubTest = [](const rclcpp::PublisherBase::SharedPtr p) {return p->get_subscription_count() == 0;};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pubTest); ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for publisher connections.");
  }

  const auto subTest = [](const rclcpp::SubscriptionBase::SharedPtr p) {return p->get_publisher_count() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    executor.spin_all(10ms);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pubTest));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  rclcpp::Time time_rclcpp = cras::parseTime("2024-11-18T13:00:00Z");
  builtin_interfaces::msg::Time time;
  time.sec = time_rclcpp.seconds();
  time.nanosec = time_rclcpp.nanoseconds() % 1'000'000'000;

  Az azimuth;
  azimuth.header.frame_id = "base_link";
  azimuth.header.stamp = time;
  azimuth.azimuth = 0;
  azimuth.variance = 0;
  azimuth.unit = Az::UNIT_RAD;
  azimuth.orientation = Az::ORIENTATION_ENU;
  azimuth.reference = Az::REFERENCE_UTM;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 50 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.w, 1e-6);

  lastPose.reset();
  azimuth.azimuth = M_PI_2;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(0, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, lastPose->pose.pose.orientation.w, 1e-6);

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
  }

  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(-M_SQRT1_2, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.w, 1e-6);

  // We haven't yet supplied a fix, so change of reference is not possible

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azimuth.reference = Az::REFERENCE_MAGNETIC;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 5 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
  }
  ASSERT_FALSE(lastPose.has_value());

  Fix fix;
  fix.header.stamp = time;
  fix.header.frame_id = "gps";
  fix.latitude = 50.090806436;
  fix.longitude = 14.133202857;
  fix.altitude = 445.6146;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fixPub->publish(fix);
  // Wait until the fix arrives
  executor.spin_all(200ms);

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azPub->publish(azimuth);
  for (size_t i = 0; i < 50 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_all(100ms);
  }

  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(-0.671109, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(0.741358, lastPose->pose.pose.orientation.w, 1e-6);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
