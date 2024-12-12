// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for naming topics according to the type of Azimuth message they carry.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <string>
#include <tuple>

#include <compass_msgs/Azimuth.h>
#include <compass_conversions/topic_names.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>

using Az = compass_msgs::Azimuth;
using Imu = sensor_msgs::Imu;
using Pose = geometry_msgs::PoseWithCovarianceStamped;
using Quat = geometry_msgs::QuaternionStamped;

static const auto rad = Az::UNIT_RAD;
static const auto deg = Az::UNIT_DEG;
static const auto enu = Az::ORIENTATION_ENU;
static const auto ned = Az::ORIENTATION_NED;
static const auto mag = Az::REFERENCE_MAGNETIC;
static const auto geo = Az::REFERENCE_GEOGRAPHIC;
static const auto utm = Az::REFERENCE_UTM;

TEST(TopicNames, GetSuffix)  // NOLINT
{
  EXPECT_EQ("mag/enu/rad", compass_conversions::getAzimuthTopicSuffix<Az>(rad, enu, mag));
  EXPECT_EQ("mag/enu/deg", compass_conversions::getAzimuthTopicSuffix<Az>(deg, enu, mag));
  EXPECT_EQ("mag/enu/imu", compass_conversions::getAzimuthTopicSuffix<Imu>(rad, enu, mag));
  EXPECT_EQ("mag/enu/pose", compass_conversions::getAzimuthTopicSuffix<Pose>(rad, enu, mag));
  EXPECT_EQ("mag/enu/quat", compass_conversions::getAzimuthTopicSuffix<Quat>(rad, enu, mag));

  EXPECT_EQ("mag/ned/rad", compass_conversions::getAzimuthTopicSuffix<Az>(rad, ned, mag));
  EXPECT_EQ("mag/ned/deg", compass_conversions::getAzimuthTopicSuffix<Az>(deg, ned, mag));
  EXPECT_EQ("mag/ned/imu", compass_conversions::getAzimuthTopicSuffix<Imu>(rad, ned, mag));
  EXPECT_EQ("mag/ned/pose", compass_conversions::getAzimuthTopicSuffix<Pose>(rad, ned, mag));
  EXPECT_EQ("mag/ned/quat", compass_conversions::getAzimuthTopicSuffix<Quat>(rad, ned, mag));

  EXPECT_EQ("true/enu/rad", compass_conversions::getAzimuthTopicSuffix<Az>(rad, enu, geo));
  EXPECT_EQ("true/enu/deg", compass_conversions::getAzimuthTopicSuffix<Az>(deg, enu, geo));
  EXPECT_EQ("true/enu/imu", compass_conversions::getAzimuthTopicSuffix<Imu>(rad, enu, geo));
  EXPECT_EQ("true/enu/pose", compass_conversions::getAzimuthTopicSuffix<Pose>(rad, enu, geo));
  EXPECT_EQ("true/enu/quat", compass_conversions::getAzimuthTopicSuffix<Quat>(rad, enu, geo));

  EXPECT_EQ("true/ned/rad", compass_conversions::getAzimuthTopicSuffix<Az>(rad, ned, geo));
  EXPECT_EQ("true/ned/deg", compass_conversions::getAzimuthTopicSuffix<Az>(deg, ned, geo));
  EXPECT_EQ("true/ned/imu", compass_conversions::getAzimuthTopicSuffix<Imu>(rad, ned, geo));
  EXPECT_EQ("true/ned/pose", compass_conversions::getAzimuthTopicSuffix<Pose>(rad, ned, geo));
  EXPECT_EQ("true/ned/quat", compass_conversions::getAzimuthTopicSuffix<Quat>(rad, ned, geo));

  EXPECT_EQ("utm/enu/rad", compass_conversions::getAzimuthTopicSuffix<Az>(rad, enu, utm));
  EXPECT_EQ("utm/enu/deg", compass_conversions::getAzimuthTopicSuffix<Az>(deg, enu, utm));
  EXPECT_EQ("utm/enu/imu", compass_conversions::getAzimuthTopicSuffix<Imu>(rad, enu, utm));
  EXPECT_EQ("utm/enu/pose", compass_conversions::getAzimuthTopicSuffix<Pose>(rad, enu, utm));
  EXPECT_EQ("utm/enu/quat", compass_conversions::getAzimuthTopicSuffix<Quat>(rad, enu, utm));

  EXPECT_EQ("utm/ned/rad", compass_conversions::getAzimuthTopicSuffix<Az>(rad, ned, utm));
  EXPECT_EQ("utm/ned/deg", compass_conversions::getAzimuthTopicSuffix<Az>(deg, ned, utm));
  EXPECT_EQ("utm/ned/imu", compass_conversions::getAzimuthTopicSuffix<Imu>(rad, ned, utm));
  EXPECT_EQ("utm/ned/pose", compass_conversions::getAzimuthTopicSuffix<Pose>(rad, ned, utm));
  EXPECT_EQ("utm/ned/quat", compass_conversions::getAzimuthTopicSuffix<Quat>(rad, ned, utm));
}

TEST(TopicNames, ParseTopicOnlySuffix)  // NOLINT
{
  EXPECT_EQ(std::make_tuple(rad, enu, mag), compass_conversions::parseAzimuthTopicName("mag/enu/rad"));
  EXPECT_EQ(std::make_tuple(rad, enu, mag), compass_conversions::parseAzimuthTopicName("mag/enu/imu"));
  EXPECT_EQ(std::make_tuple(rad, enu, mag), compass_conversions::parseAzimuthTopicName("mag/enu/pose"));
  EXPECT_EQ(std::make_tuple(rad, enu, mag), compass_conversions::parseAzimuthTopicName("mag/enu/quat"));
  EXPECT_EQ(std::make_tuple(deg, enu, mag), compass_conversions::parseAzimuthTopicName("mag/enu/deg"));

  EXPECT_EQ(std::make_tuple(rad, ned, mag), compass_conversions::parseAzimuthTopicName("mag/ned/rad"));
  EXPECT_EQ(std::make_tuple(rad, ned, mag), compass_conversions::parseAzimuthTopicName("mag/ned/imu"));
  EXPECT_EQ(std::make_tuple(rad, ned, mag), compass_conversions::parseAzimuthTopicName("mag/ned/pose"));
  EXPECT_EQ(std::make_tuple(rad, ned, mag), compass_conversions::parseAzimuthTopicName("mag/ned/quat"));
  EXPECT_EQ(std::make_tuple(deg, ned, mag), compass_conversions::parseAzimuthTopicName("mag/ned/deg"));

  EXPECT_EQ(std::make_tuple(rad, enu, geo), compass_conversions::parseAzimuthTopicName("true/enu/rad"));
  EXPECT_EQ(std::make_tuple(rad, enu, geo), compass_conversions::parseAzimuthTopicName("true/enu/imu"));
  EXPECT_EQ(std::make_tuple(rad, enu, geo), compass_conversions::parseAzimuthTopicName("true/enu/pose"));
  EXPECT_EQ(std::make_tuple(rad, enu, geo), compass_conversions::parseAzimuthTopicName("true/enu/quat"));
  EXPECT_EQ(std::make_tuple(deg, enu, geo), compass_conversions::parseAzimuthTopicName("true/enu/deg"));

  EXPECT_EQ(std::make_tuple(rad, ned, geo), compass_conversions::parseAzimuthTopicName("true/ned/rad"));
  EXPECT_EQ(std::make_tuple(rad, ned, geo), compass_conversions::parseAzimuthTopicName("true/ned/imu"));
  EXPECT_EQ(std::make_tuple(rad, ned, geo), compass_conversions::parseAzimuthTopicName("true/ned/pose"));
  EXPECT_EQ(std::make_tuple(rad, ned, geo), compass_conversions::parseAzimuthTopicName("true/ned/quat"));
  EXPECT_EQ(std::make_tuple(deg, ned, geo), compass_conversions::parseAzimuthTopicName("true/ned/deg"));

  EXPECT_EQ(std::make_tuple(rad, enu, utm), compass_conversions::parseAzimuthTopicName("utm/enu/rad"));
  EXPECT_EQ(std::make_tuple(rad, enu, utm), compass_conversions::parseAzimuthTopicName("utm/enu/imu"));
  EXPECT_EQ(std::make_tuple(rad, enu, utm), compass_conversions::parseAzimuthTopicName("utm/enu/pose"));
  EXPECT_EQ(std::make_tuple(rad, enu, utm), compass_conversions::parseAzimuthTopicName("utm/enu/quat"));
  EXPECT_EQ(std::make_tuple(deg, enu, utm), compass_conversions::parseAzimuthTopicName("utm/enu/deg"));

  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName("utm/ned/rad"));
  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName("utm/ned/imu"));
  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName("utm/ned/pose"));
  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName("utm/ned/quat"));
  EXPECT_EQ(std::make_tuple(deg, ned, utm), compass_conversions::parseAzimuthTopicName("utm/ned/deg"));
}

TEST(TopicNames, ParseTopicNotOnlySuffix)  // NOLINT
{
  EXPECT_EQ(std::make_tuple(rad, enu, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/enu/rad"));
  EXPECT_EQ(std::make_tuple(rad, enu, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/enu/imu"));
  EXPECT_EQ(std::make_tuple(rad, enu, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/enu/pose"));
  EXPECT_EQ(std::make_tuple(rad, enu, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/enu/quat"));
  EXPECT_EQ(std::make_tuple(deg, enu, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/enu/deg"));

  EXPECT_EQ(std::make_tuple(rad, ned, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/ned/rad"));
  EXPECT_EQ(std::make_tuple(rad, ned, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/ned/imu"));
  EXPECT_EQ(std::make_tuple(rad, ned, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/ned/pose"));
  EXPECT_EQ(std::make_tuple(rad, ned, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/ned/quat"));
  EXPECT_EQ(std::make_tuple(deg, ned, mag), compass_conversions::parseAzimuthTopicName("azimuth/test/mag/ned/deg"));

  EXPECT_EQ(std::make_tuple(rad, enu, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/enu/rad"));
  EXPECT_EQ(std::make_tuple(rad, enu, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/enu/imu"));
  EXPECT_EQ(std::make_tuple(rad, enu, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/enu/pose"));
  EXPECT_EQ(std::make_tuple(rad, enu, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/enu/quat"));
  EXPECT_EQ(std::make_tuple(deg, enu, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/enu/deg"));

  EXPECT_EQ(std::make_tuple(rad, ned, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/ned/rad"));
  EXPECT_EQ(std::make_tuple(rad, ned, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/ned/imu"));
  EXPECT_EQ(std::make_tuple(rad, ned, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/ned/pose"));
  EXPECT_EQ(std::make_tuple(rad, ned, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/ned/quat"));
  EXPECT_EQ(std::make_tuple(deg, ned, geo), compass_conversions::parseAzimuthTopicName("azimuth/test/true/ned/deg"));

  EXPECT_EQ(std::make_tuple(rad, enu, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/enu/rad"));
  EXPECT_EQ(std::make_tuple(rad, enu, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/enu/imu"));
  EXPECT_EQ(std::make_tuple(rad, enu, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/enu/pose"));
  EXPECT_EQ(std::make_tuple(rad, enu, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/enu/quat"));
  EXPECT_EQ(std::make_tuple(deg, enu, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/enu/deg"));

  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/ned/rad"));
  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/ned/imu"));
  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/ned/pose"));
  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/ned/quat"));
  EXPECT_EQ(std::make_tuple(deg, ned, utm), compass_conversions::parseAzimuthTopicName("azimuth/test/utm/ned/deg"));
}

TEST(TopicNames, ParseTopicWrong)  // NOLINT
{
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("azimuth").has_value());
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("azimuth/test").has_value());
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("foo").has_value());
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("rad/enu/mag").has_value());

  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("foo/ned/deg").has_value());
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("mag/foo/deg").has_value());
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("mag/ned/foo").has_value());

  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("azimuth/test/foo/ned/deg").has_value());
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("azimuth/test/mag/foo/deg").has_value());
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName("azimuth/test/mag/ned/foo").has_value());
}

TEST(TopicNames, ParseTopicFromConnectionHeader)  // NOLINT
{
  boost::shared_ptr<ros::M_string> header;
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName(header).has_value());

  header.reset(new ros::M_string{});
  EXPECT_FALSE(compass_conversions::parseAzimuthTopicName(header).has_value());

  (*header)["topic"] = "azimuth/test/utm/ned/rad";
  EXPECT_EQ(std::make_tuple(rad, ned, utm), compass_conversions::parseAzimuthTopicName(header));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
