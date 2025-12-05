#pragma once
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
#include <compass_interfaces/msg/azimuth.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <magnetometer_compass/magnetometer_compass.hpp>
#include <magnetometer_compass/tf2_sensor_msgs.hpp>
#include <magnetometer_pipeline/message_filter.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace magnetometer_compass
{

using Az = compass_interfaces::msg::Azimuth;
using Quat = geometry_msgs::msg::QuaternionStamped;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Imu = sensor_msgs::msg::Imu;
using Field = sensor_msgs::msg::MagneticField;

typedef message_filters::sync_policies::ApproximateTime<Imu, Field> SyncPolicy;

struct AzimuthPublishersConfigForOrientation
{
  std::shared_ptr<compass_conversions::CompassConverter> converter;

  rclcpp::Node::SharedPtr namespace_node;
  rclcpp::Node* param_node;

  rclcpp::Publisher<Quat>::SharedPtr quatPub;
  rclcpp::Publisher<Imu>::SharedPtr imuPub;
  rclcpp::Publisher<Pose>::SharedPtr posePub;
  rclcpp::Publisher<Az>::SharedPtr radPub;
  rclcpp::Publisher<Az>::SharedPtr degPub;

  bool publishQuat{false};
  bool publishImu{false};
  bool publishPose{false};
  bool publishRad{false};
  bool publishDeg{false};

  bool publish{false};

  AzimuthPublishersConfigForOrientation();

  void init(
    rclcpp::Node::SharedPtr namespace_node, rclcpp::Node* param_node,
    const std::shared_ptr<compass_conversions::CompassConverter>& converter,
    const std::string& paramPrefix, const std::string& topicPrefix, uint8_t reference, uint8_t orientation,
    const std::string& referenceStr, const std::string& orientationStr);

  void publishAzimuths(const Az& azimuthRad, const Imu& imuInBody);
};

struct AzimuthPublishersConfig
{
  std::shared_ptr<compass_conversions::CompassConverter> converter;

  rclcpp::Node::SharedPtr namespace_node;
  rclcpp::Node* param_node;

  AzimuthPublishersConfigForOrientation ned;
  AzimuthPublishersConfigForOrientation enu;

  bool publish{false};

  const tf2::Quaternion nedToEnu{-M_SQRT2 / 2, -M_SQRT2 / 2, 0, 0};
  const tf2::Quaternion enuToNed{this->nedToEnu.inverse()};

  AzimuthPublishersConfig();

  void init(
    rclcpp::Node::SharedPtr namespace_node, rclcpp::Node* param_node,
    const std::shared_ptr<compass_conversions::CompassConverter>& converter,
    const std::string& paramPrefix, const std::string& topicPrefix, uint8_t reference, const std::string& referenceStr);

  void publishAzimuths(const Az& nedAzimuth, const Imu& imuInBody);
};

/**
 * \brief Compute various azimuth values based on a magnetometer, IMU orientation and possibly also GPS coordinates.
 *
 * Because there is no well established Azimuth message in ROS, this node publishes custom compass_interfaces/Azimuth
 * as well as a few other formats that are capable of carrying orientation information. It also offers the azimuth
 * values in both radians and degrees, because radians are the ROS standard, while degrees are more used in the
 * geographic area. There are tens of possible combinations of the output data formats, so each of the published
 * topics has a boolean parameter that enables it. By default, there is only one enabled output topic.
 *
 * Explaining some basic terms so that you know what output is the right for you:
 *
 * Orientation:
 * - NED (North-East-Down): Azimuth will be 0 pointing to north, and increases clockwise. This is consistent with the
 *                          azimuth used in cartography and by tourists.
 * - ENU (East-North-Up): Azimuth will be 0 pointing to east, and increases counter-clockwise. This is consistent with
 *                        REP-103 and robot_localization package.
 * References for north:
 * - Magnetic: points towards the magnetic north of Earth (travels in time).
 * - Geographic ("true"): points towards the geographic Earth (i.e. the WGS84 North Pole). It is static in time.
 * - UTM: points in the north direction on the cartesian UTM grid (similar to Geographic, but it can slightly diverge
 *        at the edges of UTM maps). You probably want this azimuth reference for navigation tasks in UTM coordinates.
 *
 * Magnetic azimuth can be computed directly from the magnetometer and IMU orientation. To compute the other two
 * references, you need to provide the latitude, longitude, altitude and time in addition to the magnetometer and
 * IMU orientation. These are the required inputs to compute magnetic declination and UTM grid convergence, which are
 * the offsets by which geographic and UTM references differ from the magnetic. This is why this compass node subscribes
 * to the GPS fix messages. Until at least a single GPS fix message is received, neither geographic- nor UTM-referenced
 * data are published. If you do not have a GPS receiver, you can alternatively provide these values in parameters.
 *
 * For the magnetometer to work correctly, it is required to measure its bias. This node listens on the `imu/mag_bias`
 * topic for this measurement, and until at least one message arrives, the node will not publish anything. If you do not
 * have a node publishing the bias, you can alternatively provide it via parameters. Depending on the application, it
 * may be required to re-estimate the bias from time to time even during runtime.
 *
 * Subscribed topics:
 * - `imu/data` (`sensor_msgs/Imu`): Output from an IMU or an orientation filtering algorithm. It should have valid
 *                                   contents of `orientation` and at least roll and pitch should be estimated as
 *                                   well as possible (relative to the gravity vector). These messages should come at
 *                                   the same rate as the magnetometer data (or faster). Make sure the orientation
 *                                   is reported in ENU frame (use imu_transformer package to transform it from NED).
 * - `imu/mag` (`sensor_msgs/MagneticField`): 3-axis magnetometer raw measurements (bias not removed) (disabled by param
 *                                            `~subscribe_mag_unbiased`).
 * - `imu/mag_bias` (`sensor_msgs/MagneticField`): Bias of the magnetometer. This value will be subtracted from the
 *                                                 incoming magnetometer measurements. Messages on this topic do not
 *                                                 need to come repeatedly if the bias does not change. Disabled by
 *                                                 param `~subscribe_mag_unbiased`.
 * - `imu/mag_unbiased` (`sensor_msgs/MagneticField`): 3-axis magnetometer unbiased measurements (enabled by param
 *                                                     `~subscribe_mag_unbiased`).
 * - `gps/fix` (`sensor_msgs/NavSatFix`, optional): GPS fix messages from which the latitude, longitude, altitude and
 *                                                  current year can be read. These are further used to compute
 *                                                  magnetic declination and UTM grid convergence factor if requested.
 * - TF: This node requires a (usually static) transform between `~frame` and the frame ID of the IMU and magnetometer
 *       messages.
 *
 * Published topics (see above for explanation):
 * - `imu/mag_unbiased` (`sensor_msgs/MagneticField`, enabled by param `~publish_mag_unbiased`, off by default):
 *     The magnetic field measurement with bias removed.
 *
 * - `compass/mag/ned/deg` (`compass_interfaces/Azimuth`, enabled by param `~publish_mag_azimuth_ned_deg`, on by default):
 *     Magnetic azimuth in NED in degrees (the same values you can see on touristic magnetic compasses).
 * - `compass/mag/ned/rad` (`compass_interfaces/Azimuth`, enabled by param `~publish_mag_azimuth_ned_rad`, off by default):
 *     Magnetic azimuth in NED in radians.
 * - `compass/mag/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_mag_azimuth_ned_quat`, off by default):
 *     Magnetic azimuth in NED as a quaternion.
 * - `compass/mag/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_mag_azimuth_ned_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards magnetic NED frame.
 * - `compass/mag/ned/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_mag_azimuth_ned_pose`, off by default):
 *     Magnetic azimuth in NED as a pose (translation will always be zero).
 *
 * - `compass/mag/enu/deg` (`compass_interfaces/Azimuth`, enabled by param `~publish_mag_azimuth_enu_deg`, off by default):
 *     Magnetic azimuth in ENU in degrees.
 * - `compass/mag/enu/rad` (`compass_interfaces/Azimuth`, enabled by param `~publish_mag_azimuth_enu_rad`, off by default):
 *     Magnetic azimuth in ENU in radians.
 * - `compass/mag/enu/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_mag_azimuth_enu_quat`, off by default):
 *     Magnetic azimuth in ENU as a quaternion.
 * - `compass/mag/enu/imu` (`sensor_msgs/Imu`, enabled by param `~publish_mag_azimuth_enu_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards magnetic ENU frame.
 * - `compass/mag/enu/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_mag_azimuth_enu_pose`, off by default):
 *     Magnetic azimuth in ENU as a pose (translation will always be zero).
 *
 * - `compass/true/ned/deg` (`compass_interfaces/Azimuth`, enabled by param `~publish_true_azimuth_ned_deg`, off by default):
 *     Geographic ("true") azimuth in NED in degrees.
 * - `compass/true/ned/rad` (`compass_interfaces/Azimuth`, enabled by param `~publish_true_azimuth_ned_rad`, off by default):
 *     Geographic ("true") azimuth in NED in radians.
 * - `compass/true/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_true_azimuth_ned_quat`, off by default):
 *     Geographic ("true") azimuth in NED as a quaternion.
 * - `compass/true/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_true_azimuth_ned_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards geographic ("true") NED frame.
 * - `compass/true/ned/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_true_azimuth_ned_pose`, off by default):
 *     Geographic ("true") azimuth in NED as a pose (translation will always be zero).
 *
 * - `compass/true/enu/deg` (`compass_interfaces/Azimuth`, enabled by param `~publish_true_azimuth_enu_deg`, off by default):
 *     Geographic ("true") azimuth in ENU in degrees.
 * - `compass/true/enu/rad` (`compass_interfaces/Azimuth`, enabled by param `~publish_true_azimuth_enu_rad`, off by default):
 *     Geographic ("true") azimuth in ENU in radians.
 * - `compass/true/enu/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_true_azimuth_enu_quat`, off by default):
 *     Geographic ("true") azimuth in ENU as a quaternion.
 * - `compass/true/enu/imu` (`sensor_msgs/Imu`, enabled by param `~publish_true_azimuth_enu_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards geographic ("true") ENU frame.
 * - `compass/true/enu/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_true_azimuth_enu_pose`, off by default):
 *     Geographic ("true") azimuth in ENU as a pose (translation will always be zero).
 *
 * - `compass/utm/ned/deg` (`compass_interfaces/Azimuth`, enabled by param `~publish_utm_azimuth_ned_deg`, off by default):
 *     UTM heading in NED in degrees.
 * - `compass/utm/ned/rad` (`compass_interfaces/Azimuth`, enabled by param `~publish_utm_azimuth_ned_rad`, off by default):
 *     UTM heading in NED in radians.
 * - `compass/utm/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_utm_azimuth_ned_quat`, off by default):
 *     UTM heading in NED as a quaternion.
 * - `compass/utm/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_utm_azimuth_ned_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards UTM NED frame.
 * - `compass/utm/ned/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_utm_azimuth_ned_pose`, off by default):
 *     UTM heading in NED as a pose (translation will always be zero).
 *
 * - `compass/utm/enu/deg` (`compass_interfaces/Azimuth`, enabled by param `~publish_utm_azimuth_enu_deg`, off by default):
 *     UTM heading in ENU in degrees.
 * - `compass/utm/enu/rad` (`compass_interfaces/Azimuth`, enabled by param `~publish_utm_azimuth_enu_rad`, off by default):
 *     UTM heading in ENU in radians.
 * - `compass/utm/enu/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_utm_azimuth_enu_quat`, off by default):
 *     UTM heading in ENU as a quaternion.
 * - `compass/utm/enu/imu` (`sensor_msgs/Imu`, enabled by param `~publish_utm_azimuth_enu_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards UTM ENU frame.
 * - `compass/utm/enu/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_utm_azimuth_enu_pose`, off by default):
 *     UTM heading in ENU as a pose (translation will always be zero).
 *
 * Parameters:
 * - All the `publish_*` parameters mentioned above.
 * - Please note that you cannot combine both `~subscribe_mag_unbiased` and `~publish_mag_unbiased` set to true.
 *   Such configuration is invalid and the node will not start.
 * - `~frame` (string, default `base_link`): Frame into which the IMU and magnetometer data should be transformed.
 * - `~low_pass_ratio` (double, default 0.95): The azimuth is filtered with a low-pass filter. This sets its
 *                                             aggressivity (0 means raw measurements, 1 means no updates).
 * - `~initial_mag_bias_x` (double, no default, optional): Magnetometer bias in the X axis.
 * - `~initial_mag_bias_y` (double, no default, optional): Magnetometer bias in the Y axis.
 * - `~initial_mag_bias_z` (double, no default, optional): Magnetometer bias in the Z axis.
 *   - If you specify any of the `~initial_mag_bias_*` params, the node does not need to receive the bias messages.
 * - `~initial_lat` (double, no default, optional): Latitude in degrees.
 * - `~initial_lon` (double, no default, optional): Longitude in degrees.
 *   - If you specify both `~initial_lat` and `~initial_lon`, the node does not need to receive the GPS fix messages.
 * - `~initial_alt` (double, default 0): Altitude in meters (it is usually okay to omit it and use the default).
 * - `~initial_year` (int, no default, optional): If set, overrides the current time for declination computation.
 * - `~initial_variance` (double, default 0): Variance of the measurement used at startup (in rad^2).
 * - `~magnetic_declination` (double, no default, optional, radians): If this parameter is set, the magnetic models are
 *                                                                    ignored and this value for declination is forced.
 *                                                                    This can be useful either if you know the value
 *                                                                    in advance or in simulation.
 * - `~magnetic_models_path` (string, defaults to the pre-installed directory): Directory with WMM magnetic field
 *      models. You usually do not need to use other than the preinstalled models. But if you do, specify the path to
 *      the custom models directory here.
 * - `~magnetic_model` (string, defaults to autodetection by year): Name of the magnetic field model to use. If omitted,
 *      an automated decision is made based on the current year (or `~initial_year`, if set). This model is used for
 *      computing magnetic declination.
 */
class MagnetometerCompassNodelet : public rclcpp::Node
{
public:
  explicit MagnetometerCompassNodelet(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MagnetometerCompassNodelet() override;

  void init();
  void setBuffer(tf2_ros::Buffer::SharedPtr buffer, bool using_dedicated_thread);

protected:
  //! \brief Joint callback when IMU and magnetometer messages are received.
  //! \param[in] imu IMU data. Only `orientation` is relevant, and it should contain filtered absolute orientation.
  //! \param[in] mag Magnetometer data (biased).
  void imuMagCb(const sensor_msgs::msg::Imu& imu, const sensor_msgs::msg::MagneticField& mag);

  //! \brief Callback for GPS fix (so that the node can compute magnetic declination and UTM grid convergence).
  //! \param[in] fix The fix message. Only `latitude`, `longitude`, `altitude` and `header.stamp` are relevant.
  void fixCb(const sensor_msgs::msg::NavSatFix& fix);

  //! \brief TF frame in which the compass should be expressed. Usually base_link.
  std::string frame;

  tf2_ros::Buffer::SharedPtr buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;

  std::shared_ptr<compass_conversions::CompassConverter> converter;
  std::shared_ptr<magnetometer_compass::MagnetometerCompass> compass;
  std::unique_ptr<message_filters::Subscriber<Imu>> imuSub;
  std::unique_ptr<message_filters::Subscriber<Field>> magSub;
  std::unique_ptr<message_filters::Subscriber<Field>> magBiasSub;
  std::unique_ptr<message_filters::Subscriber<Field>> magUnbiasedSub;
  std::unique_ptr<magnetometer_pipeline::BiasRemoverFilter> magBiasRemoverFilter;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> syncSub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fixSub;

  rclcpp::Publisher<Field>::SharedPtr magUnbiasedPub;
  bool publishMagUnbiased{false};
  bool subscribeMagUnbiased{false};

  AzimuthPublishersConfig magPublishers;
  AzimuthPublishersConfig truePublishers;
  AzimuthPublishersConfig utmPublishers;
};

}
