// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer bias removing message filter.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>

#include <boost/functional.hpp>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <magnetometer_pipeline/message_filter.h>
#include <message_filters/simple_filter.h>
#include <ros/message_event.h>
#include <sensor_msgs/MagneticField.h>
#include <XmlRpcValue.h>

using Field = sensor_msgs::MagneticField;

template<class T>
class TestInput : public message_filters::SimpleFilter<T>
{
public:
  void add(const typename T::ConstPtr& msg)
  {
    // Pass a complete MessageEvent to avoid calling ros::Time::now() to determine the missing timestamp
    this->signalMessage(ros::MessageEvent<T const>(msg, msg->header.stamp));
  }

  void subscribe()
  {
  }
};

TEST(MessageFilter, Basic)  // NOLINT
{
  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  TestInput<Field> magInput;
  TestInput<Field> magBiasInput;
  magnetometer_pipeline::BiasRemoverFilter filter(log, magInput, magBiasInput);

  Field::ConstPtr outMessage;
  const auto cb = [&outMessage](const ros::MessageEvent<Field const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(boost::function<void(const ros::MessageEvent<Field const>&)>(cb));

  ros::Time time(1664286802, 187375068);

  Field::Ptr mag(new Field);
  mag->header.stamp = time;
  mag->header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.263093;
  mag->magnetic_field.y = -0.538677;
  mag->magnetic_field.z = 0.157033;

  Field::Ptr bias(new Field);
  bias->header.stamp = time;
  bias->header.frame_id = "imu";
  bias->magnetic_field.x = -0.097227663;
  bias->magnetic_field.y = -0.692264333;
  bias->magnetic_field.z = 0;

  outMessage.reset();
  magInput.add(mag);

  EXPECT_EQ(nullptr, outMessage);

  outMessage.reset();
  magBiasInput.add(bias);
  EXPECT_EQ(nullptr, outMessage);

  outMessage.reset();
  magInput.add(mag);
  ASSERT_NE(nullptr, outMessage);
  EXPECT_EQ(time, outMessage->header.stamp);
  EXPECT_EQ("imu", outMessage->header.frame_id);
  EXPECT_NEAR(0.360320, outMessage->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, outMessage->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, outMessage->magnetic_field.z, 1e-6);

  // New data

  time = {1664286802, 197458028};

  mag->header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.264200;
  mag->magnetic_field.y = -0.533960;
  mag->magnetic_field.z = 0.149800;

  outMessage.reset();
  magInput.add(mag);
  ASSERT_NE(nullptr, outMessage);
  EXPECT_EQ(time, outMessage->header.stamp);
  EXPECT_EQ("imu", outMessage->header.frame_id);
  EXPECT_NEAR(0.361427, outMessage->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, outMessage->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, outMessage->magnetic_field.z, 1e-6);
}

TEST(MessageFilter, ConfigFromParams)  // NOLINT
{
  const auto log = std::make_shared<cras::MemoryLogHelper>();
  // const auto log = std::make_shared<cras::NodeLogHelper>();

  TestInput<Field> magInput;
  TestInput<Field> magBiasInput;
  magnetometer_pipeline::BiasRemoverFilter filter(log, magInput, magBiasInput);

  Field::ConstPtr outMessage;
  const auto cb = [&outMessage](const ros::MessageEvent<Field const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(boost::function<void(const ros::MessageEvent<Field const>&)>(cb));

  XmlRpc::XmlRpcValue config;
  config.begin();  // set to dict type
  config["initial_mag_bias_x"] = -0.097227663;
  config["initial_mag_bias_y"] = -0.692264333;
  config["initial_mag_bias_z"] = 0;

  cras::BoundParamHelper params(log, std::make_shared<cras::XmlRpcValueGetParamAdapter>(config, ""));
  filter.configFromParams(params);

  ros::Time time(1664286802, 187375068);

  Field::Ptr mag(new Field);
  mag->header.stamp = time;
  mag->header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.263093;
  mag->magnetic_field.y = -0.538677;
  mag->magnetic_field.z = 0.157033;

  outMessage.reset();
  magInput.add(mag);
  ASSERT_NE(nullptr, outMessage);
  EXPECT_EQ(time, outMessage->header.stamp);
  EXPECT_EQ("imu", outMessage->header.frame_id);
  EXPECT_NEAR(0.360320, outMessage->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, outMessage->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, outMessage->magnetic_field.z, 1e-6);

  // New data

  time = {1664286802, 197458028};

  mag->header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.264200;
  mag->magnetic_field.y = -0.533960;
  mag->magnetic_field.z = 0.149800;

  outMessage.reset();
  magInput.add(mag);
  ASSERT_NE(nullptr, outMessage);
  EXPECT_EQ(time, outMessage->header.stamp);
  EXPECT_EQ("imu", outMessage->header.frame_id);
  EXPECT_NEAR(0.361427, outMessage->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, outMessage->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, outMessage->magnetic_field.z, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
