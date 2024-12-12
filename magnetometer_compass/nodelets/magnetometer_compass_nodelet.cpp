// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Compute various azimuth values based on a magnetometer, IMU orientation and possibly also GPS coordinates.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <compass_conversions/compass_converter.h>
#include <compass_conversions/topic_names.h>
#include <compass_msgs/Azimuth.h>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <imu_transformer/tf2_sensor_msgs.h>
#include <magnetometer_compass/magnetometer_compass.h>
#include <magnetometer_pipeline/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace magnetometer_compass
{
using Az = compass_msgs::Azimuth;
using Quat = geometry_msgs::QuaternionStamped;
using Pose = geometry_msgs::PoseWithCovarianceStamped;
using Imu = sensor_msgs::Imu;
using Field = sensor_msgs::MagneticField;

typedef message_filters::sync_policies::ApproximateTime<Imu, Field> SyncPolicy;

struct AzimuthPublishersConfigForOrientation : public cras::HasLogger
{
  std::shared_ptr<compass_conversions::CompassConverter> converter;

  ros::Publisher quatPub;
  ros::Publisher imuPub;
  ros::Publisher posePub;
  ros::Publisher radPub;
  ros::Publisher degPub;

  bool publishQuat{false};
  bool publishImu{false};
  bool publishPose{false};
  bool publishRad{false};
  bool publishDeg{false};

  bool publish{false};

  explicit AzimuthPublishersConfigForOrientation(const cras::LogHelperPtr& log);

  void init(
    ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::shared_ptr<compass_conversions::CompassConverter>& converter,
    const std::string& paramPrefix, const std::string& topicPrefix, uint8_t reference, uint8_t orientation,
    const std::string& referenceStr, const std::string& orientationStr);

  void publishAzimuths(const Az& azimuthRad, const Imu& imuInBody);
};

struct AzimuthPublishersConfig :  public cras::HasLogger
{
  std::shared_ptr<compass_conversions::CompassConverter> converter;

  AzimuthPublishersConfigForOrientation ned;
  AzimuthPublishersConfigForOrientation enu;

  bool publish{false};

  const tf2::Quaternion nedToEnu{-M_SQRT2 / 2, -M_SQRT2 / 2, 0, 0};
  const tf2::Quaternion enuToNed{this->nedToEnu.inverse()};

  explicit AzimuthPublishersConfig(const cras::LogHelperPtr& log);

  void init(
    ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::shared_ptr<compass_conversions::CompassConverter>& converter,
    const std::string& paramPrefix, const std::string& topicPrefix, uint8_t reference, const std::string& referenceStr);

  void publishAzimuths(const Az& nedAzimuth, const Imu& imuInBody);
};

/**
 * \brief Compute various azimuth values based on a magnetometer, IMU orientation and possibly also GPS coordinates.
 *
 * Because there is no well established Azimuth message in ROS, this node publishes our custom compass_msgs/Azimuth
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
 * - `compass/mag/ned/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_mag_azimuth_ned_deg`, on by default):
 *     Magnetic azimuth in NED in degrees (the same values you can see on touristic magnetic compasses).
 * - `compass/mag/ned/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_mag_azimuth_ned_rad`, off by default):
 *     Magnetic azimuth in NED in radians.
 * - `compass/mag/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_mag_azimuth_ned_quat`, off by default):
 *     Magnetic azimuth in NED as a quaternion.
 * - `compass/mag/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_mag_azimuth_ned_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards magnetic NED frame.
 * - `compass/mag/ned/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_mag_azimuth_ned_pose`, off by default):
 *     Magnetic azimuth in NED as a pose (translation will always be zero).
 *
 * - `compass/mag/enu/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_mag_azimuth_enu_deg`, off by default):
 *     Magnetic azimuth in ENU in degrees.
 * - `compass/mag/enu/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_mag_azimuth_enu_rad`, off by default):
 *     Magnetic azimuth in ENU in radians.
 * - `compass/mag/enu/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_mag_azimuth_enu_quat`, off by default):
 *     Magnetic azimuth in ENU as a quaternion.
 * - `compass/mag/enu/imu` (`sensor_msgs/Imu`, enabled by param `~publish_mag_azimuth_enu_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards magnetic ENU frame.
 * - `compass/mag/enu/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_mag_azimuth_enu_pose`, off by default):
 *     Magnetic azimuth in ENU as a pose (translation will always be zero).
 *
 * - `compass/true/ned/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_true_azimuth_ned_deg`, off by default):
 *     Geographic ("true") azimuth in NED in degrees.
 * - `compass/true/ned/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_true_azimuth_ned_rad`, off by default):
 *     Geographic ("true") azimuth in NED in radians.
 * - `compass/true/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_true_azimuth_ned_quat`, off by default):
 *     Geographic ("true") azimuth in NED as a quaternion.
 * - `compass/true/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_true_azimuth_ned_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards geographic ("true") NED frame.
 * - `compass/true/ned/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_true_azimuth_ned_pose`, off by default):
 *     Geographic ("true") azimuth in NED as a pose (translation will always be zero).
 *
 * - `compass/true/enu/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_true_azimuth_enu_deg`, off by default):
 *     Geographic ("true") azimuth in ENU in degrees.
 * - `compass/true/enu/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_true_azimuth_enu_rad`, off by default):
 *     Geographic ("true") azimuth in ENU in radians.
 * - `compass/true/enu/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_true_azimuth_enu_quat`, off by default):
 *     Geographic ("true") azimuth in ENU as a quaternion.
 * - `compass/true/enu/imu` (`sensor_msgs/Imu`, enabled by param `~publish_true_azimuth_enu_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards geographic ("true") ENU frame.
 * - `compass/true/enu/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_true_azimuth_enu_pose`, off by default):
 *     Geographic ("true") azimuth in ENU as a pose (translation will always be zero).
 *
 * - `compass/utm/ned/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_utm_azimuth_ned_deg`, off by default):
 *     UTM heading in NED in degrees.
 * - `compass/utm/ned/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_utm_azimuth_ned_rad`, off by default):
 *     UTM heading in NED in radians.
 * - `compass/utm/ned/quat` (`geometry_msgs/QuaternionStamped`, enabled by param `~publish_utm_azimuth_ned_quat`, off by default):
 *     UTM heading in NED as a quaternion.
 * - `compass/utm/ned/imu` (`sensor_msgs/Imu`, enabled by param `~publish_utm_azimuth_ned_imu`, off by default):
 *     The incoming IMU message rotated in yaw such that its frame becomes georeferenced towards UTM NED frame.
 * - `compass/utm/ned/pose` (`geometry_msgs/PoseWithCovarianceStamped`, enabled by param `~publish_utm_azimuth_ned_pose`, off by default):
 *     UTM heading in NED as a pose (translation will always be zero).
 *
 * - `compass/utm/enu/deg` (`compass_msgs/Azimuth`, enabled by param `~publish_utm_azimuth_enu_deg`, off by default):
 *     UTM heading in ENU in degrees.
 * - `compass/utm/enu/rad` (`compass_msgs/Azimuth`, enabled by param `~publish_utm_azimuth_enu_rad`, off by default):
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
class MagnetometerCompassNodelet : public cras::Nodelet
{
public:
  MagnetometerCompassNodelet();
  ~MagnetometerCompassNodelet() override;

protected:
  void onInit() override;

  //! \brief Joint callback when IMU and magnetometer messages are received.
  //! \param[in] imu IMU data. Only `orientation` is relevant, and it should contain filtered absolute orientation.
  //! \param[in] mag Magnetometer data (biased).
  void imuMagCb(const sensor_msgs::Imu& imu, const sensor_msgs::MagneticField& mag);

  //! \brief Callback for GPS fix (so that the node can compute magnetic declination and UTM grid convergence).
  //! \param[in] fix The fix message. Only `latitude`, `longitude`, `altitude` and `header.stamp` are relevant.
  void fixCb(const sensor_msgs::NavSatFix& fix);

  //! \brief TF frame in which the compass should be expressed. Usually base_link.
  std::string frame;

  std::shared_ptr<compass_conversions::CompassConverter> converter;
  std::shared_ptr<magnetometer_compass::MagnetometerCompass> compass;
  std::unique_ptr<message_filters::Subscriber<Imu>> imuSub;
  std::unique_ptr<message_filters::Subscriber<Field>> magSub;
  std::unique_ptr<message_filters::Subscriber<Field>> magBiasSub;
  std::unique_ptr<message_filters::Subscriber<Field>> magUnbiasedSub;
  std::unique_ptr<magnetometer_pipeline::BiasRemoverFilter> magBiasRemoverFilter;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> syncSub;
  ros::Subscriber fixSub;

  ros::Publisher magUnbiasedPub;
  bool publishMagUnbiased{false};
  bool subscribeMagUnbiased{false};

  AzimuthPublishersConfig magPublishers;
  AzimuthPublishersConfig truePublishers;
  AzimuthPublishersConfig utmPublishers;
};

MagnetometerCompassNodelet::MagnetometerCompassNodelet() :
  magPublishers(this->log), truePublishers(this->log), utmPublishers(this->log)
{
}

MagnetometerCompassNodelet::~MagnetometerCompassNodelet() = default;

void MagnetometerCompassNodelet::onInit()
{
  cras::Nodelet::onInit();

  auto nh = this->getNodeHandle();
  auto pnh = this->getPrivateNodeHandle();

  auto params = this->privateParams();

  this->frame = params->getParam("frame", "base_link");

  const auto strict = params->getParam("strict", true);
  this->converter = std::make_shared<compass_conversions::CompassConverter>(this->getLogger(), strict);
  this->converter->configFromParams(*params);

  this->compass = std::make_shared<MagnetometerCompass>(this->log, this->frame, this->getBufferPtr());
  this->compass->configFromParams(*params);

  this->publishMagUnbiased = params->getParam("publish_mag_unbiased", this->publishMagUnbiased);
  this->subscribeMagUnbiased = params->getParam("subscribe_mag_unbiased", this->subscribeMagUnbiased);

  if (this->publishMagUnbiased && this->subscribeMagUnbiased)
    throw std::runtime_error("Cannot simultaneously subscribe and publish unbiased magnetometer.");

  // Set default publishers
  this->magPublishers.ned.publishDeg = true;

  bool publish = this->publishMagUnbiased;

  ros::NodeHandle compassNh(nh, "compass");
  // Read the publish_* parameters and set up the selected publishers
  this->magPublishers.init(compassNh, pnh, this->converter, "publish", "", Az::REFERENCE_MAGNETIC, "mag");
  publish |= this->magPublishers.publish;
  this->truePublishers.init(compassNh, pnh, this->converter, "publish", "", Az::REFERENCE_GEOGRAPHIC, "true");
  publish |= this->truePublishers.publish;
  this->utmPublishers.init(compassNh, pnh, this->converter, "publish", "", Az::REFERENCE_UTM, "utm");
  publish |= this->utmPublishers.publish;

  if (!publish)
    CRAS_WARN("No publishers have been requested. Please, set one of the publish_* parameters to true.");

  ros::NodeHandle imuNh(nh, "imu");

  if (this->publishMagUnbiased)
    this->magUnbiasedPub = imuNh.advertise<Field>("mag_unbiased", 10);

  this->imuSub = std::make_unique<message_filters::Subscriber<Imu>>(imuNh, "data", 100);

  // Check if we should try to unbias the magnetometer ourselves or if you already got it unbiased on the input.
  if (this->subscribeMagUnbiased)
  {
    this->magSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag_unbiased", 100);
    this->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *this->imuSub, *this->magSub);
  }
  else
  {
    this->magSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag", 100);
    this->magBiasSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag_bias", 10);
    this->magBiasRemoverFilter = std::make_unique<magnetometer_pipeline::BiasRemoverFilter>(
      this->getLogger(), *this->magSub, *this->magBiasSub);
    this->magBiasRemoverFilter->configFromParams(*params);

    this->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *this->imuSub, *this->magBiasRemoverFilter);
  }
  this->syncSub->registerCallback(&MagnetometerCompassNodelet::imuMagCb, this);

  this->fixSub = nh.subscribe("gps/fix", 10, &MagnetometerCompassNodelet::fixCb, this);
}

AzimuthPublishersConfigForOrientation::AzimuthPublishersConfigForOrientation(const cras::LogHelperPtr& log) :
  cras::HasLogger(log)
{
}

void AzimuthPublishersConfigForOrientation::init(
  ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::shared_ptr<compass_conversions::CompassConverter>& converter,
  const std::string& paramPrefix, const std::string& topicPrefix, const uint8_t reference, const uint8_t orientation,
  const std::string& referenceStr, const std::string& orientationStr)
{
  this->converter = converter;

  auto prefix = paramPrefix + "_" + referenceStr + "_azimuth_" + orientationStr + "_";
  this->publishQuat = pnh.param(prefix + "quat", this->publishQuat);
  this->publishImu = pnh.param(prefix + "imu", this->publishImu);
  this->publishPose = pnh.param(prefix + "pose", this->publishPose);
  this->publishRad = pnh.param(prefix + "rad", this->publishRad);
  this->publishDeg = pnh.param(prefix + "deg", this->publishDeg);

  this->publish = this->publishQuat || this->publishImu || this->publishPose || this->publishDeg || this->publishRad;

  using compass_conversions::getAzimuthTopicSuffix;

  prefix = cras::appendIfNonEmpty(topicPrefix, "/");
  if (this->publishQuat)
    this->quatPub = nh.advertise<Quat>(prefix + getAzimuthTopicSuffix<Quat>(Az::UNIT_RAD, orientation, reference), 10);
  if (this->publishImu)
    this->imuPub = nh.advertise<Imu>(prefix + getAzimuthTopicSuffix<Imu>(Az::UNIT_RAD, orientation, reference), 10);
  if (this->publishPose)
    this->posePub = nh.advertise<Pose>(prefix + getAzimuthTopicSuffix<Pose>(Az::UNIT_RAD, orientation, reference), 10);
  if (this->publishRad)
    this->radPub = nh.advertise<Az>(prefix + getAzimuthTopicSuffix<Az>(Az::UNIT_RAD, orientation, reference), 10);
  if (this->publishDeg)
    this->degPub = nh.advertise<Az>(prefix + getAzimuthTopicSuffix<Az>(Az::UNIT_DEG, orientation, reference), 10);
}

AzimuthPublishersConfig::AzimuthPublishersConfig(const cras::LogHelperPtr& log) :
  cras::HasLogger(log), ned(log), enu(log)
{
}

void AzimuthPublishersConfig::init(
  ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::shared_ptr<compass_conversions::CompassConverter>& converter,
  const std::string& paramPrefix, const std::string& topicPrefix,
  const uint8_t reference, const std::string& referenceStr)
{
  this->converter = converter;
  this->ned.init(nh, pnh, converter, paramPrefix, topicPrefix, reference, Az::ORIENTATION_NED, referenceStr, "ned");
  this->enu.init(nh, pnh, converter, paramPrefix, topicPrefix, reference, Az::ORIENTATION_ENU, referenceStr, "enu");
  this->publish = this->ned.publish || this->enu.publish;
}

void MagnetometerCompassNodelet::imuMagCb(const Imu& imu, const Field& magUnbiased)
{
  if (this->publishMagUnbiased)
    this->magUnbiasedPub.publish(magUnbiased);

  const auto maybeAzimuth = this->compass->computeAzimuth(imu, magUnbiased);
  if (!maybeAzimuth.has_value())
  {
    CRAS_ERROR_DELAYED_THROTTLE(1.0, "%s", maybeAzimuth.error().c_str());
    return;
  }

  Imu imuInBody;
  try
  {
    // No timeout because computeAzimuth() has already waited for this exact transform
    this->getBuffer().transform(imu, imuInBody, this->frame);
  }
  catch (const tf2::TransformException& e)
  {
    CRAS_ERROR_DELAYED_THROTTLE(1.0,
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
      this->truePublishers.publishAzimuths(*maybeTrueNedAzimuthMsg, imuInBody);
    else
      CRAS_ERROR_DELAYED_THROTTLE(1.0, "%s", maybeTrueNedAzimuthMsg.error().c_str());
  }

  if (this->utmPublishers.publish)
  {
    const auto maybeUTMNedAzimuthMsg = this->converter->convertAzimuth(
      nedAzimuthMsg, nedAzimuthMsg.unit, nedAzimuthMsg.orientation, Az::REFERENCE_UTM);
    if (maybeUTMNedAzimuthMsg.has_value())
      this->utmPublishers.publishAzimuths(*maybeUTMNedAzimuthMsg, imuInBody);
    else
      CRAS_ERROR_DELAYED_THROTTLE(1.0, "%s", maybeUTMNedAzimuthMsg.error().c_str());
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
      geometry_msgs::TransformStamped tf;
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
      CRAS_ERROR_THROTTLE(1.0, "Could not convert from NED to ENU: %s", maybeEnuAzimuth.error().c_str());
  }
}

void AzimuthPublishersConfigForOrientation::publishAzimuths(const Az& azimuthRad, const Imu& imuInBody)
{
  if (this->publishQuat)
  {
    const auto maybeQuat = this->converter->convertToQuaternion(azimuthRad);
    if (maybeQuat.has_value())
      this->quatPub.publish(*maybeQuat);
    else
      CRAS_ERROR_THROTTLE(1.0, "%s", maybeQuat.error().c_str());
  }

  if (this->publishImu)
  {
    const auto maybeQuat = this->converter->convertToQuaternion(azimuthRad);
    if (!maybeQuat.has_value())
    {
      CRAS_ERROR_THROTTLE(1.0, "%s", maybeQuat.error().c_str());
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

      sensor_msgs::Imu imuMsg;

      geometry_msgs::TransformStamped tf;
      tf.header = imuInBody.header;
      tf2::convert(diffRot, tf.transform.rotation);
      tf2::doTransform(imuInBody, imuMsg, tf);

      imuMsg.orientation_covariance[8] = azimuthRad.variance;

      this->imuPub.publish(imuMsg);
    }
  }

  if (this->publishPose)
  {
    const auto maybePose = this->converter->convertToPose(azimuthRad);
    if (maybePose.has_value())
      this->posePub.publish(*maybePose);
    else
      CRAS_ERROR_THROTTLE(1.0, "%s", maybePose.error().c_str());
  }

  if (this->publishRad)
  {
    this->radPub.publish(azimuthRad);
  }

  if (this->publishDeg)
  {
    const auto maybeAzimuthDeg = this->converter->convertAzimuth(
      azimuthRad, Az::UNIT_DEG, azimuthRad.orientation, azimuthRad.reference);
    if (maybeAzimuthDeg.has_value())
      this->degPub.publish(*maybeAzimuthDeg);
    else
      CRAS_ERROR_THROTTLE(1.0, "%s", maybeAzimuthDeg.error().c_str());
  }
}

void MagnetometerCompassNodelet::fixCb(const sensor_msgs::NavSatFix& fix)
{
  this->converter->setNavSatPos(fix);
}

}

PLUGINLIB_EXPORT_CLASS(magnetometer_compass::MagnetometerCompassNodelet, nodelet::Nodelet)
