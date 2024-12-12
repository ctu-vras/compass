// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer_bias_remover.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>

#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/param_utils/param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <magnetometer_pipeline/bias_remover.h>
#include <sensor_msgs/MagneticField.h>
#include <XmlRpcValue.h>

using Field = sensor_msgs::MagneticField;

TEST(MagnetometerBiasRemover, Basic)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto remover = magnetometer_pipeline::MagnetometerBiasRemover(log);

  EXPECT_FALSE(remover.hasBias());

  ros::Time time(1664286802, 187375068);

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093;
  mag.magnetic_field.y = -0.538677;
  mag.magnetic_field.z = 0.157033;

  EXPECT_FALSE(remover.hasBias());
  EXPECT_FALSE(remover.removeBias(mag).has_value());

  // Set bias. Now it should have everything.

  Field bias;
  bias.header.stamp = time;
  bias.header.frame_id = "imu";
  bias.magnetic_field.x = -0.097227663;
  bias.magnetic_field.y = -0.692264333;
  bias.magnetic_field.z = 0;
  remover.setBias(bias);

  EXPECT_TRUE(remover.hasBias());
  auto maybeMagUnbiased = remover.removeBias(mag);
  ASSERT_TRUE(maybeMagUnbiased.has_value());

  EXPECT_EQ(time, maybeMagUnbiased->header.stamp);
  EXPECT_EQ("imu", maybeMagUnbiased->header.frame_id);
  EXPECT_NEAR(0.360320, maybeMagUnbiased->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, maybeMagUnbiased->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, maybeMagUnbiased->magnetic_field.z, 1e-6);

  // New data

  time = {1664286802, 197458028};

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200;
  mag.magnetic_field.y = -0.533960;
  mag.magnetic_field.z = 0.149800;

  maybeMagUnbiased = remover.removeBias(mag);
  ASSERT_TRUE(maybeMagUnbiased.has_value());

  EXPECT_EQ(time, maybeMagUnbiased->header.stamp);
  EXPECT_EQ("imu", maybeMagUnbiased->header.frame_id);
  EXPECT_NEAR(0.361427, maybeMagUnbiased->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, maybeMagUnbiased->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, maybeMagUnbiased->magnetic_field.z, 1e-6);
}

TEST(MagnetometerBiasRemover, ConfigFromParams)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto remover = magnetometer_pipeline::MagnetometerBiasRemover(log);
  EXPECT_FALSE(remover.hasBias());

  XmlRpc::XmlRpcValue config;
  config.begin();  // set to dict type
  config["initial_mag_bias_x"] = -0.097227663;
  config["initial_mag_bias_y"] = -0.692264333;
  config["initial_mag_bias_z"] = 0;

  cras::BoundParamHelper params(log, std::make_shared<cras::XmlRpcValueGetParamAdapter>(config, ""));
  remover.configFromParams(params);

  EXPECT_TRUE(remover.hasBias());

  ros::Time time(1664286802, 187375068);

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093;
  mag.magnetic_field.y = -0.538677;
  mag.magnetic_field.z = 0.157033;

  auto maybeMagUnbiased = remover.removeBias(mag);
  ASSERT_TRUE(maybeMagUnbiased.has_value());

  EXPECT_EQ(time, maybeMagUnbiased->header.stamp);
  EXPECT_EQ("imu", maybeMagUnbiased->header.frame_id);
  EXPECT_NEAR(0.360320, maybeMagUnbiased->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, maybeMagUnbiased->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, maybeMagUnbiased->magnetic_field.z, 1e-6);

  // New data

  time = {1664286802, 197458028};

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200;
  mag.magnetic_field.y = -0.533960;
  mag.magnetic_field.z = 0.149800;

  maybeMagUnbiased = remover.removeBias(mag);
  ASSERT_TRUE(maybeMagUnbiased.has_value());

  EXPECT_EQ(time, maybeMagUnbiased->header.stamp);
  EXPECT_EQ("imu", maybeMagUnbiased->header.frame_id);
  EXPECT_NEAR(0.361427, maybeMagUnbiased->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, maybeMagUnbiased->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, maybeMagUnbiased->magnetic_field.z, 1e-6);
}

TEST(MagnetometerBiasRemover, ConfigFromParamsWithScale)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  // const auto log = std::make_shared<cras::MemoryLogHelper>();
  const auto log = std::make_shared<cras::NodeLogHelper>();

  auto remover = magnetometer_pipeline::MagnetometerBiasRemover(log);
  EXPECT_FALSE(remover.hasBias());

  XmlRpc::XmlRpcValue config;
  config.begin();  // set to dict type
  config["initial_mag_bias_x"] = -0.097227663;
  config["initial_mag_bias_y"] = -0.692264333;
  config["initial_mag_bias_z"] = 0;
  config["initial_mag_scaling_matrix"][0 * 3 + 0] = 2.0;
  config["initial_mag_scaling_matrix"][0 * 3 + 1] = 0.0;
  config["initial_mag_scaling_matrix"][0 * 3 + 2] = 0.0;
  config["initial_mag_scaling_matrix"][1 * 3 + 0] = 0.0;
  config["initial_mag_scaling_matrix"][1 * 3 + 1] = 1.0;
  config["initial_mag_scaling_matrix"][1 * 3 + 2] = 0.0;
  config["initial_mag_scaling_matrix"][2 * 3 + 0] = 0.0;
  config["initial_mag_scaling_matrix"][2 * 3 + 1] = 0.0;
  config["initial_mag_scaling_matrix"][2 * 3 + 2] = 1.0;

  cras::BoundParamHelper params(log, std::make_shared<cras::XmlRpcValueGetParamAdapter>(config, ""));
  remover.configFromParams(params);

  EXPECT_TRUE(remover.hasBias());

  ros::Time time(1664286802, 187375068);

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093;
  mag.magnetic_field.y = -0.538677;
  mag.magnetic_field.z = 0.157033;

  auto maybeMagUnbiased = remover.removeBias(mag);
  ASSERT_TRUE(maybeMagUnbiased.has_value());

  EXPECT_EQ(time, maybeMagUnbiased->header.stamp);
  EXPECT_EQ("imu", maybeMagUnbiased->header.frame_id);
  EXPECT_NEAR(2 * 0.360320, maybeMagUnbiased->magnetic_field.x, 1e-5);
  EXPECT_NEAR(0.153587, maybeMagUnbiased->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, maybeMagUnbiased->magnetic_field.z, 1e-6);

  // New data

  time = {1664286802, 197458028};

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200;
  mag.magnetic_field.y = -0.533960;
  mag.magnetic_field.z = 0.149800;

  maybeMagUnbiased = remover.removeBias(mag);
  ASSERT_TRUE(maybeMagUnbiased.has_value());

  EXPECT_EQ(time, maybeMagUnbiased->header.stamp);
  EXPECT_EQ("imu", maybeMagUnbiased->header.frame_id);
  EXPECT_NEAR(2 * 0.361427, maybeMagUnbiased->magnetic_field.x, 1e-5);
  EXPECT_NEAR(0.158304, maybeMagUnbiased->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, maybeMagUnbiased->magnetic_field.z, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
