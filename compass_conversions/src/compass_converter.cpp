// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert between various compass representations.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <tl/expected.hpp>

#include <angles/angles.h>
#include <compass_conversions/compass_converter.hpp>
#include <compass_conversions/topic_names.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/tf2_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <magnetic_model/magnetic_model.hpp>
#include <magnetic_model/magnetic_model_manager.hpp>
#include <message_filters/message_event.hpp>
#include <message_filters/message_traits.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace compass_conversions
{

using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Quat = geometry_msgs::msg::QuaternionStamped;

/**
 * \brief Private data of CompassConverter.
 */
struct CompassConverterPrivate
{
  //! \brief Magnetic model manager.
  std::unique_ptr<magnetic_model::MagneticModelManager> magneticModelManager;

  //! \brief Cache of already initialized magnetic field models. Keys are years.
  std::map<uint32_t, std::shared_ptr<magnetic_model::MagneticModel>> magneticModels;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr log;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
};

CompassConverter::CompassConverter(rclcpp::Node* node, const bool strict) : CompassConverter(*node, strict)
{
}

CompassConverter::CompassConverter(RequiredInterfaces node, const bool strict)
  : strict(strict), node(node), data(new CompassConverterPrivate{})
{
  this->data->clock = node.get_node_clock_interface();
  this->data->graph = node.get_node_graph_interface();
  this->data->log = node.get_node_logging_interface();
  this->data->params = node.get_node_parameters_interface();
  this->data->topics = node.get_node_topics_interface();

  this->data->magneticModelManager = std::make_unique<magnetic_model::MagneticModelManager>(node);
}

CompassConverter::~CompassConverter() = default;

void CompassConverter::configFromParams()
{
  cras::TempLocale l(LC_ALL, "en_US.UTF-8");  // Support printing ° signs

  const auto log = this->data->log;
  const auto params = this->data->params;

  if (params->has_parameter("magnetic_declination") &&
    params->get_parameter("magnetic_declination").as_double() != -9999.)
  {
    RCLCPP_WARN(log->get_logger(), "Forcing magnetic declination to be %f",
      params->get_parameter("magnetic_declination").as_double());
    this->forceMagneticDeclination(params->get_parameter("magnetic_declination").as_double());
  }
  else if (params->has_parameter("magnetic_declination"))
  {
    this->forcedMagneticModelName = params->get_parameter("magnetic_model").as_string();
  }
  if (params->has_parameter("magnetic_models_path") &&
      !params->get_parameter("magnetic_models_path").as_string().empty())
    this->setMagneticModelPath(params->get_parameter("magnetic_models_path").as_string());

  if (params->has_parameter("utm_grid_convergence") && params->get_parameter("utm_grid_convergence").as_double() != -1.)
    this->forceUTMGridConvergence(params->get_parameter("utm_grid_convergence").as_double());

  if (params->has_parameter("utm_zone") && params->get_parameter("utm_zone").as_int() != -1)
    this->forceUTMZone(params->get_parameter("utm_zone").as_int());

  if (params->has_parameter("use_wall_time_for_declination"))
    this->useWallTimeForDeclination = params->get_parameter("use_wall_time_for_declination").as_bool();

  this->setKeepUTMZone(params->has_parameter("keep_utm_zone") ?
    params->get_parameter("keep_utm_zone").as_bool() : this->keepUTMZone);

  if (!this->forcedMagneticDeclination.has_value() || !this->forcedUTMGridConvergence.has_value())
  {
    if (params->has_parameter("initial_lat") && params->has_parameter("initial_lon") &&
      params->get_parameter("initial_lat").as_double() != -1 && params->get_parameter("initial_lon").as_double() != -1)
    {
      sensor_msgs::msg::NavSatFix msg;

      msg.latitude = params->has_parameter("initial_lat") ? params->get_parameter("initial_lat").as_double() : 0.0;
      msg.longitude = params->has_parameter("initial_lon") ? params->get_parameter("initial_lon").as_double() : 0.0;
      msg.altitude = params->has_parameter("initial_alt") ? params->get_parameter("initial_alt").as_double() : 0.0;

      std::list<std::string> computedValues;
      if (!this->forcedMagneticDeclination.has_value())
        computedValues.emplace_back("magnetic declination");
      if (!this->forcedUTMGridConvergence.has_value())
        computedValues.emplace_back("UTM grid convergence");

      RCLCPP_INFO(
        log->get_logger(), "Initial GPS coords for computation of %s are %.6f°, %.6f°, altitude %.0f m.",
        cras::join(computedValues, " and ").c_str(), msg.latitude, msg.longitude, msg.altitude);

      this->setNavSatPos(msg);
    }
  }
}

void CompassConverter::forceMagneticDeclination(const std::optional<double>& declination)
{
  this->forcedMagneticDeclination = declination;
}

void CompassConverter::forceUTMGridConvergence(const std::optional<double>& convergence)
{
  this->forcedUTMGridConvergence = this->lastUTMGridConvergence = convergence;
}

void CompassConverter::setMagneticModelPath(const std::optional<std::string>& modelPath)
{
  this->data->magneticModelManager->setModelPath(modelPath);
}

void CompassConverter::forceMagneticModelName(const std::string& model)
{
  this->forcedMagneticModelName = model;
}

void CompassConverter::setUseWallTimeForDeclination(const bool use)
{
  this->useWallTimeForDeclination = use;
}

void CompassConverter::setKeepUTMZone(const bool keep)
{
  this->keepUTMZone = keep;
}

cras::expected<double, std::string> CompassConverter::getMagneticDeclination(const rclcpp::Time& stamp) const
{
  if (this->forcedMagneticDeclination.has_value())
    return *this->forcedMagneticDeclination;

  if (!this->lastFix.has_value())
    return cras::make_unexpected("Cannot determine magnetic declination without GNSS pose.");

  return this->computeMagneticDeclination(*this->lastFix, stamp);
}

cras::expected<double, std::string> CompassConverter::computeMagneticDeclination(
  const sensor_msgs::msg::NavSatFix& fix, const rclcpp::Time& stamp) const
{
  const auto clock = this->node.get<NodeClockInterface>();
  const auto modelStamp = this->useWallTimeForDeclination ? clock->get_clock()->now() : stamp;
  const auto year = cras::getYear(modelStamp);

  if (this->data->magneticModels[year] == nullptr)
  {
    const auto modelName = !this->forcedMagneticModelName.empty() ?
      this->forcedMagneticModelName : this->data->magneticModelManager->getBestMagneticModelName(modelStamp);

    const auto model = this->data->magneticModelManager->getMagneticModel(modelName, this->strict);
    if (!model.has_value())
      return cras::make_unexpected(std::format(
        "Could not create magnetic field model {} for year {} because of the following error: {}",
        modelName, std::to_string(year), model.error()));
    this->data->magneticModels[year] = *model;
  }

  const auto& magModel = *this->data->magneticModels[year];

  const auto fieldComponents = magModel.getMagneticFieldComponents(fix, modelStamp);
  if (!fieldComponents.has_value())
    return cras::make_unexpected(fieldComponents.error());

  return fieldComponents->values.declination;
}

cras::expected<double, std::string> CompassConverter::getUTMGridConvergence() const
{
  if (this->forcedUTMGridConvergence.has_value())
    return *this->forcedUTMGridConvergence;

  if (!this->lastUTMGridConvergence.has_value())
    return cras::make_unexpected("UTM grid convergence has not yet been determined from GNSS pose.");

  return *this->lastUTMGridConvergence;
}

cras::expected<int, std::string> CompassConverter::getUTMZone() const
{
  if (this->forcedUTMZone.has_value())
    return *this->forcedUTMZone;

  if (!this->lastUTMZone.has_value())
    return cras::make_unexpected("UTM zone has not yet been determined from GNSS pose.");

  return *this->lastUTMZone;
}

void CompassConverter::forceUTMZone(const std::optional<int>& zone)
{
  if (zone.has_value() && (zone < GeographicLib::UTMUPS::MINZONE || zone > GeographicLib::UTMUPS::MAXZONE))
    RCLCPP_WARN(this->data->log->get_logger(), "Invalid UTM zone: %d", *zone);
  else
    this->forcedUTMZone = this->lastUTMZone = zone;
}

cras::expected<std::pair<double, int>, std::string> CompassConverter::computeUTMGridConvergenceAndZone(
  const sensor_msgs::msg::NavSatFix& fix, const std::optional<int>& utmZone) const
{
  if (utmZone.has_value() && (*utmZone < GeographicLib::UTMUPS::MINZONE || *utmZone > GeographicLib::UTMUPS::MAXZONE))
    return cras::make_unexpected(std::format("Invalid UTM zone: {}", std::to_string(*utmZone)));

  try
  {
    int zone;
    bool isNorthHemisphere;
    double northing, easting, utmGridConvergence, projectionScale;
    const int setzone = utmZone.value_or(GeographicLib::UTMUPS::STANDARD);

    GeographicLib::UTMUPS::Forward(fix.latitude, fix.longitude,
      zone, isNorthHemisphere, easting, northing, utmGridConvergence, projectionScale, setzone);

    return std::make_pair(angles::from_degrees(utmGridConvergence), zone);
  }
  catch (const GeographicLib::GeographicErr& e)
  {
    return cras::make_unexpected(std::format("Could not get UTM grid convergence: {}", e.what()));
  }
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertAzimuth(
  const compass_interfaces::msg::Azimuth& azimuth,
  const Unit unit, const Orientation orientation, const Reference reference) const
{
  // Fast track for no conversion
  if (azimuth.unit == unit && azimuth.orientation == orientation && azimuth.reference == reference)
    return azimuth;

  Az result = azimuth;
  result.unit = unit;
  result.orientation = orientation;
  result.reference = reference;

  // Convert the input to NED radians
  if (azimuth.unit == Az::UNIT_DEG)
    result.azimuth = angles::from_degrees(result.azimuth);
  if (azimuth.orientation == Az::ORIENTATION_ENU)
    result.azimuth = M_PI_2 - result.azimuth;

  // When going magnetic->true, we need to add declination in NED.
  // When going true->UTM, we need to subtract grid convergence in NED.

  // Now convert between references in NED radians
  if (azimuth.reference != result.reference)
  {
    if (azimuth.reference == Az::REFERENCE_MAGNETIC)
    {
      const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);

      if (!magneticDeclination.has_value())
      {
        return cras::make_unexpected(std::format(
          "Cannot convert magnetic azimuth to true without knowing magnetic declination. Error: {}",
          magneticDeclination.error()));
      }
      result.azimuth += *magneticDeclination;

      if (result.reference == Az::REFERENCE_UTM)
      {
        const auto convergence = this->getUTMGridConvergence();
        if (!convergence.has_value())
        {
          return cras::make_unexpected(std::format(
            "Cannot convert true azimuth to UTM without knowing UTM grid convergence. Error: {}", convergence.error()));
        }
        result.azimuth -= *convergence;
      }
    }
    else if (azimuth.reference == Az::REFERENCE_GEOGRAPHIC)
    {
      if (result.reference == Az::REFERENCE_MAGNETIC)
      {
        const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);
        if (!magneticDeclination.has_value())
        {
          return cras::make_unexpected(std::format(
            "Cannot convert true azimuth to magnetic without knowing magnetic declination. Error: {}",
            magneticDeclination.error()));
        }
        result.azimuth -= *magneticDeclination;
      }
      else if (result.reference == Az::REFERENCE_UTM)
      {
        const auto convergence = this->getUTMGridConvergence();
        if (!convergence.has_value())
        {
          return cras::make_unexpected(std::format(
            "Cannot convert true azimuth to UTM without knowing UTM grid convergence. Error: {}", convergence.error()));
        }
        result.azimuth -= *convergence;
      }
    }
    else if (azimuth.reference == Az::REFERENCE_UTM)
    {
      const auto convergence = this->getUTMGridConvergence();
      if (!convergence.has_value())
      {
        return cras::make_unexpected(std::format(
          "Cannot convert UTM azimuth to true without knowing UTM grid convergence. Error: {}", convergence.error()));
      }
      result.azimuth += *convergence;

      if (result.reference == Az::REFERENCE_MAGNETIC)
      {
        const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);
        if (!magneticDeclination.has_value())
        {
          return cras::make_unexpected(std::format(
            "Cannot convert true azimuth to magnetic without knowing magnetic declination. Error: {}",
            magneticDeclination.error()));
        }
        result.azimuth -= *magneticDeclination;
      }
    }
  }

  // Reference is correct now; convert to the output unit and orientation
  if (result.orientation == Az::ORIENTATION_ENU)
    result.azimuth = M_PI_2 - result.azimuth;
  result.azimuth = angles::normalize_angle_positive(result.azimuth);
  if (result.unit == Az::UNIT_DEG)
    result.azimuth = angles::to_degrees(result.azimuth);

  if (azimuth.unit == Az::UNIT_RAD && result.unit == Az::UNIT_DEG)
    result.variance = std::pow(angles::to_degrees(std::sqrt(azimuth.variance)), 2);
  else if (azimuth.unit == Az::UNIT_DEG && result.unit == Az::UNIT_RAD)
    result.variance = std::pow(angles::from_degrees(std::sqrt(azimuth.variance)), 2);

  return result;
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertQuaternion(
  const geometry_msgs::msg::QuaternionStamped& quat, const Variance variance,
  const Unit unit, const Orientation orientation, const Reference reference) const
{
  return this->convertQuaternion(quat.quaternion, quat.header, variance, unit, orientation, reference);
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertQuaternion(
  const geometry_msgs::msg::Quaternion& quat, const std_msgs::msg::Header& header, const Variance variance,
  const Unit unit, const Orientation orientation, const Reference reference) const
{
  tf2::Quaternion q;
  tf2::fromMsg(quat, q);
  if (q.length2() < 1e-6)
    return cras::make_unexpected("Invalid quaternion (all zeros).");

  compass_interfaces::msg::Azimuth result;
  result.header = header;
  result.azimuth = cras::getYaw(quat);
  result.variance = variance;
  if (unit == Az::UNIT_DEG)
  {
    result.azimuth = angles::to_degrees(result.azimuth);
    result.variance = std::pow(angles::to_degrees(std::sqrt(variance)), 2);
  }
  result.orientation = orientation;
  result.unit = unit;
  result.reference = reference;
  return result;
}

cras::expected<geometry_msgs::msg::QuaternionStamped, std::string> CompassConverter::convertToQuaternion(
  const compass_interfaces::msg::Azimuth& azimuth) const
{
  tf2::Stamped<tf2::Quaternion> quat;
  quat.frame_id_ = azimuth.header.frame_id;
  quat.stamp_ = tf2_ros::fromMsg(azimuth.header.stamp);
  quat.setRPY(0, 0, azimuth.azimuth * (azimuth.unit == Az::UNIT_RAD ? 1 : M_PI / 180.0));
  return tf2::toMsg(quat);
}

cras::expected<geometry_msgs::msg::PoseWithCovarianceStamped, std::string> CompassConverter::convertToPose(
  const compass_interfaces::msg::Azimuth& azimuth) const
{
  const auto maybeQuat = this->convertToQuaternion(azimuth);
  if (!maybeQuat.has_value())
    return cras::make_unexpected(std::format("Could not convert azimuth to pose: {}", maybeQuat.error()));

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = azimuth.header;
  pose.pose.pose.orientation = maybeQuat->quaternion;
  pose.pose.covariance[0 * 6 + 0] = std::pow(40000, 2);
  pose.pose.covariance[1 * 6 + 1] = std::pow(40000, 2);
  pose.pose.covariance[2 * 6 + 2] = std::pow(40000, 2);
  pose.pose.covariance[3 * 6 + 3] = 4 * M_PI * M_PI;
  pose.pose.covariance[4 * 6 + 4] = 4 * M_PI * M_PI;
  pose.pose.covariance[5 * 6 + 5] = azimuth.variance;

  return pose;
}

cras::expected<sensor_msgs::msg::Imu, std::string> CompassConverter::convertToImu(
  const compass_interfaces::msg::Azimuth& azimuth) const
{
  const auto maybeQuat = this->convertToQuaternion(azimuth);
  if (!maybeQuat.has_value())
    return cras::make_unexpected(std::format("Could not convert azimuth to pose: {}", maybeQuat.error()));

  sensor_msgs::msg::Imu imu;
  imu.header = azimuth.header;
  imu.linear_acceleration_covariance[0] = -1;
  imu.angular_velocity_covariance[0] = -1;
  imu.orientation = maybeQuat->quaternion;
  imu.orientation_covariance[0 * 3 + 0] = 4 * M_PI * M_PI;
  imu.orientation_covariance[1 * 3 + 1] = 4 * M_PI * M_PI;
  imu.orientation_covariance[2 * 3 + 2] = azimuth.variance;

  return imu;
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertQuaternionMsgEvent(
  const std::string& topic, const message_filters::MessageEvent<geometry_msgs::msg::QuaternionStamped const>& quatEvent,
  const Variance variance, const Unit unit, const std::optional<Orientation>& orientation,
  const std::optional<Reference>& reference) const
{
  auto msgOrientation = orientation;
  auto msgReference = reference;
  if (!msgOrientation.has_value() || !msgReference.has_value())
  {
    const auto maybeAzimuthParams = compass_conversions::parseAzimuthTopicName(topic);
    if (maybeAzimuthParams.has_value())
    {
      msgOrientation = std::get<1>(*maybeAzimuthParams);
      msgReference = std::get<2>(*maybeAzimuthParams);
    }
  }

  if (!msgOrientation.has_value() || !msgReference.has_value())
    return cras::make_unexpected("Orientation and reference are not specified and cannot be autodetected.");

  const auto& msg = quatEvent.getConstMessage();
  return this->convertQuaternion(*msg, variance, unit, *msgOrientation, *msgReference);
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertPoseMsgEvent(
  const std::string& topic,
  const message_filters::MessageEvent<geometry_msgs::msg::PoseWithCovarianceStamped const>& poseEvent,
  const Unit unit, const std::optional<Orientation>& orientation, const std::optional<Reference>& reference) const
{
  auto msgOrientation = orientation;
  auto msgReference = reference;
  if (!msgOrientation.has_value() || !msgReference.has_value())
  {
    const auto maybeAzimuthParams = compass_conversions::parseAzimuthTopicName(topic);
    if (maybeAzimuthParams.has_value())
    {
      msgOrientation = std::get<1>(*maybeAzimuthParams);
      msgReference = std::get<2>(*maybeAzimuthParams);
    }
  }

  if (!msgOrientation.has_value() || !msgReference.has_value())
    return cras::make_unexpected("Orientation and reference are not specified and cannot be autodetected.");

  const auto& msg = poseEvent.getConstMessage();
  return this->convertQuaternion(
    msg->pose.pose.orientation, msg->header, msg->pose.covariance[5 * 6 + 5], unit, *msgOrientation, *msgReference);
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertImuMsgEvent(
  const std::string& topic, const message_filters::MessageEvent<sensor_msgs::msg::Imu>& imuEvent,
  const Unit unit, const std::optional<Orientation>& orientation, const std::optional<Reference>& reference) const
{
  auto msgOrientation = orientation;
  auto msgReference = reference;
  if (!msgOrientation.has_value() || !msgReference.has_value())
  {
    const auto maybeAzimuthParams = compass_conversions::parseAzimuthTopicName(topic);
    if (maybeAzimuthParams.has_value())
    {
      msgOrientation = std::get<1>(*maybeAzimuthParams);
      msgReference = std::get<2>(*maybeAzimuthParams);
    }
  }

  // IMUs should output orientation in ENU frame
  if (!msgOrientation.has_value())
    msgOrientation = Az::ORIENTATION_ENU;

  if (!msgReference.has_value())
    return cras::make_unexpected("Reference is not specified and cannot be autodetected.");

  const auto& msg = imuEvent.getConstMessage();
  return this->convertQuaternion(
    msg->orientation, msg->header, msg->orientation_covariance[2 * 3 + 2], unit, *msgOrientation, *msgReference);
}

template<typename Msg>
std::shared_ptr<Msg> deserializeMessage(const std::shared_ptr<const rclcpp::SerializedMessage> msg)
{
  std::shared_ptr<Msg> deserialized_msg(new Msg);
  rclcpp::Serialization<Msg> serializer;
  serializer.deserialize_message(msg.get(), deserialized_msg.get());
  return deserialized_msg;
}

cras::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertSerializedMsgEvent(
  const std::string& topic, const message_filters::MessageEvent<rclcpp::SerializedMessage const>& event,
  const Variance variance, const Unit unit, const std::optional<Orientation>& orientation,
  const std::optional<Reference>& reference) const
{
  const auto pubInfo = this->data->graph->get_publishers_info_by_topic(topic);
  if (pubInfo.empty())
    return cras::make_unexpected("Could not detect type of topic " + topic);

  const auto& topicType = pubInfo[0].topic_type();
  if (topicType == rosidl_generator_traits::name<Az>())
  {
    return *deserializeMessage<Az>(event.getConstMessage());
  }
  else if (topicType == rosidl_generator_traits::name<Quat>())
  {
    const message_filters::DefaultMessageCreator<Quat> creator{};
    return this->convertQuaternionMsgEvent(topic,
      {deserializeMessage<Quat>(event.getConstMessage()), event.getReceiptTime(), true, creator},
      variance, unit, orientation, reference);
  }
  else if (topicType == rosidl_generator_traits::name<Pose>())
  {
    const message_filters::DefaultMessageCreator<Pose> creator{};
    return this->convertPoseMsgEvent(topic,
      {deserializeMessage<Pose>(event.getConstMessage()), event.getReceiptTime(), true, creator},
      unit, orientation, reference);
  }
  else if (topicType == rosidl_generator_traits::name<Imu>())
  {
    const message_filters::DefaultMessageCreator<Imu> creator{};
    return this->convertImuMsgEvent(topic,
      {deserializeMessage<Imu>(event.getConstMessage()), event.getReceiptTime(), true, creator},
      unit, orientation, reference);
  }
  else
  {
    return cras::make_unexpected("Invalid topic type " + topicType);
  }
}

template<class M> using ME = message_filters::MessageEvent<M>;
template<class M> using Creator = message_filters::DefaultMessageCreator<M>;

void CompassConverter::setNavSatPos(const sensor_msgs::msg::NavSatFix& fix)
{
  const auto log = this->data->log;
  const auto clock = this->data->clock;

  if (fix.status.status == -1)
  {
    RCLCPP_WARN_THROTTLE(log->get_logger(), *clock->get_clock(), 5000., "Ignoring GPS msg with status -1");
    return;
  }

  this->lastFix = fix;

  if (!this->forcedUTMGridConvergence.has_value())
  {
    const auto maybeConvergenceAndZone = this->computeUTMGridConvergenceAndZone(fix, this->forcedUTMZone);
    if (!maybeConvergenceAndZone.has_value())
    {
      RCLCPP_WARN_THROTTLE(log->get_logger(), *clock->get_clock(), 10000.,
        "Error computing UTM grid convergence: %s", maybeConvergenceAndZone.error().c_str());
    }
    else
    {
      const auto [convergence, zone] = *maybeConvergenceAndZone;

      this->lastUTMZone = zone;
      if (this->keepUTMZone && !this->forcedUTMZone.has_value())
        this->forcedUTMZone = zone;

      this->lastUTMGridConvergence = convergence;
    }
  }
}

}
