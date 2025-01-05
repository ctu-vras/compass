// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_msgs::Azimuth messages.
 * \author Martin Pecka
 */

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <compass_conversions/message_filter.h>
#include <compass_conversions/tf2_compass_msgs.h>
#include <compass_conversions/topic_names.h>
#include <compass_msgs/Azimuth.h>
#include <compass_msgs/string_utils.h>
#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/tf2_utils/message_filter.hpp>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int32.h>
#include <tf2/exceptions.h>
#include <tf2_ros/message_filter.h>

using Az = compass_msgs::Azimuth;
using Fix = sensor_msgs::NavSatFix;

namespace compass_conversions
{

enum class OutputType
{
  Azimuth,
  Imu,
  Pose,
  Quaternion,
};

OutputType parseOutputType(const std::string& outputType)
{
  const auto output = cras::toLower(outputType);
  if (output == "azimuth")
    return OutputType::Azimuth;
  else if (output == "imu")
    return OutputType::Imu;
  else if (output == "pose")
    return OutputType::Pose;
  else if (output == "quaternion" || output == "quat")
    return OutputType::Quaternion;
  else
    throw std::runtime_error("Unknown output type: " + outputType);
}

std::string outputTypeToString(const OutputType type)
{
  switch (type)
  {
    case OutputType::Azimuth:
      return "azimuth";
    case OutputType::Imu:
      return "imu";
    case OutputType::Pose:
      return "pose";
    case OutputType::Quaternion:
      return "quaternion";
    default:
      throw std::runtime_error(cras::format("Unknown output type: %d", static_cast<int>(type)));
  }
}

/**
 * \brief Nodelet for transforming one type and parametrization of azimuth to another type, parametrization and
 *        TF frame.
 *
 * Subscribed topics:
 * - `~azimuth_in` (compass_msgs/Azimuth or geometry_msgs/QuaternionStamped or geometry_msgs/PoseWithCovarianceStamped
 *     or sensor_msgs/Imu): The input azimuth. The name of the topic (if you remap it) can be used to autodetect some
 *     metadata for the conversion.
 *  - `fix` (sensor_msgs/NavSatFix): GNSS fix messages that can be used to determine some parameters for the conversion.
 *  - `utm_zone` (std_msgs/Int32): Optional messages with forced UTM zone.
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
class CompassTransformerNodelet : public cras::Nodelet
{
protected:
  void onInit() override
  {
    cras::Nodelet::onInit();

    const auto params = this->privateParams();

    // Start reading params

    const auto queue_size = params->getParam("queue_size", 10_sz, "messages");
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    const auto targetUnit = params->getParam<decltype(Az::unit), std::string>("target_unit", Az::UNIT_RAD, "",
      cras::GetParamConvertingOptions<decltype(Az::unit), std::string>(
        &compass_msgs::unitToString, &compass_msgs::parseUnit));

    const auto targetOrientation = params->getParam<decltype(Az::orientation), std::string>(
      "target_orientation", Az::ORIENTATION_ENU, "",
      cras::GetParamConvertingOptions<decltype(Az::orientation), std::string>(
        &compass_msgs::orientationToString, &compass_msgs::parseOrientation));

    const auto targetReference = params->getParam<decltype(Az::reference), std::string>(
      "target_reference", Az::REFERENCE_GEOGRAPHIC, "",
      cras::GetParamConvertingOptions<decltype(Az::reference), std::string>(
       &compass_msgs::referenceToString, &compass_msgs::parseReference));

    this->targetType = params->getParam<OutputType, std::string>("target_type", this->targetType, "",
      cras::GetParamConvertingOptions<OutputType, std::string>(&outputTypeToString, &parseOutputType));

    const auto targetAppendSuffix = params->getParam("target_append_suffix", false);

    this->targetFrame = params->getParam("target_frame", std::string());

    const auto subscribeFix = params->getParam("subscribe_fix", true);
    const auto subscribeUTMZone = params->getParam("subscribe_utm", true);

    // End reading params

    const auto log = this->getLogger();

    this->converter = std::make_shared<CompassConverter>(log, params->getParam("strict", true));
    this->converter->configFromParams(*params);

    auto outputNh = targetAppendSuffix ? ros::NodeHandle(pnh, "azimuth_out") : pnh;

    std::string outputTopicSuffix;
    switch (this->targetType)
    {
      case OutputType::Imu:
        outputTopicSuffix = getAzimuthTopicSuffix<sensor_msgs::Imu>(targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<sensor_msgs::Imu>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      case OutputType::Pose:
        outputTopicSuffix = getAzimuthTopicSuffix<geometry_msgs::PoseWithCovarianceStamped>(
          targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      case OutputType::Quaternion:
        outputTopicSuffix = getAzimuthTopicSuffix<geometry_msgs::QuaternionStamped>(
          targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<geometry_msgs::QuaternionStamped>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      default:
        outputTopicSuffix = getAzimuthTopicSuffix<Az>(targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<Az>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
    }

    this->azimuthInput = std::make_unique<UniversalAzimuthSubscriber>(this->log, pnh, "azimuth_in", queue_size);
    this->azimuthInput->configFromParams(*params);

    this->compassFilter = std::make_unique<CompassFilter>(
      log, this->converter, *this->azimuthInput, targetUnit, targetOrientation, targetReference);

    if (subscribeFix)
    {
      this->fixInput = std::make_unique<message_filters::Subscriber<Fix>>(nh, "fix", queue_size);
      this->compassFilter->connectFixInput(*this->fixInput);
    }

    if (subscribeUTMZone)
    {
      this->utmZoneInput = std::make_unique<message_filters::Subscriber<std_msgs::Int32>>(nh, "utm_zone", queue_size);
      this->compassFilter->connectUTMZoneInput(*this->utmZoneInput);
    }

    if (targetFrame.empty())
    {
      this->compassFilter->registerCallback(&CompassTransformerNodelet::publish, this);
    }
    else
    {
      this->tfFilter = std::make_unique<cras::TfMessageFilter<Az>>(
        log, *this->compassFilter, this->getBuffer().getRawBuffer(), targetFrame, queue_size, nh);
      this->tfFilter->registerCallback(&CompassTransformerNodelet::transformAndPublish, this);
      this->tfFilter->registerFailureCallback(cras::bind_front(&CompassTransformerNodelet::failedCb, this));
    }

    CRAS_INFO("Publishing azimuth to topic %s (type %s).",
      ros::names::resolve(this->pub.getTopic()).c_str(), outputTypeToString(this->targetType).c_str());
  }

  void publish(const Az::ConstPtr& msg)
  {
    switch (this->targetType)
    {
      case OutputType::Imu:
      {
        const auto maybeImu = this->converter->convertToImu(*msg);
        if (maybeImu.has_value())
          this->pub.publish(*maybeImu);
        else
          CRAS_ERROR_THROTTLE(1.0, "%s", maybeImu.error().c_str());
        break;
      }
      case OutputType::Pose:
      {
        const auto maybePose = this->converter->convertToPose(*msg);
        if (maybePose.has_value())
          this->pub.publish(*maybePose);
        else
          CRAS_ERROR_THROTTLE(1.0, "%s", maybePose.error().c_str());
        break;
      }
      case OutputType::Quaternion:
      {
        const auto maybeQuat = this->converter->convertToQuaternion(*msg);
        if (maybeQuat.has_value())
          this->pub.publish(*maybeQuat);
        else
          CRAS_ERROR_THROTTLE(1.0, "%s", maybeQuat.error().c_str());
        break;
      }
      default:
        this->pub.publish(msg);
        break;
    }
  }

  void transformAndPublish(const Az::ConstPtr& msg)
  {
    try
    {
      Az::Ptr outMsg(new Az{});
      *outMsg = this->getBuffer().transform(*msg, this->targetFrame, ros::Duration(0.1));
      this->publish(outMsg);
    }
    catch (const tf2::TransformException& e)
    {
      CRAS_WARN_THROTTLE(1.0, "Azimuth transformation failed: %s", e.what());
    }
  }

  void failedCb(const Az::ConstPtr& /*msg*/, const tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    CRAS_WARN_THROTTLE(1.0, "Can't transform incoming Azimuth data to frame %s. Reason %d",
      this->targetFrame.c_str(), reason);
  }

  std::shared_ptr<CompassConverter> converter;
  std::unique_ptr<UniversalAzimuthSubscriber> azimuthInput;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> fixInput;
  std::unique_ptr<message_filters::Subscriber<std_msgs::Int32>> utmZoneInput;
  std::unique_ptr<CompassFilter> compassFilter;
  std::unique_ptr<cras::TfMessageFilter<compass_msgs::Azimuth>> tfFilter;
  ros::Publisher pub;
  std::string targetFrame;
  OutputType targetType {OutputType::Azimuth};
};

}

PLUGINLIB_EXPORT_CLASS(compass_conversions::CompassTransformerNodelet, nodelet::Nodelet)
