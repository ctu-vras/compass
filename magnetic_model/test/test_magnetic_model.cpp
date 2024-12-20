// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetic model.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cstdlib>
#include <memory>
#include <string>

#include <angles/angles.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <magnetic_model/magnetic_model.h>
#include <magnetic_model/magnetic_model_manager.h>

TEST(MagneticModel, Construct)  // NOLINT
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);

  ASSERT_NO_THROW(magnetic_model::MagneticModel model(
    log, magnetic_model::MagneticModel::WMM2010, TEST_DATA_DIR, true));

  ASSERT_NO_THROW(magnetic_model::MagneticModel model(
    log, magnetic_model::MagneticModel::WMM2015, TEST_DATA_DIR, true));

  ASSERT_NO_THROW(magnetic_model::MagneticModel model(
    log, magnetic_model::MagneticModel::WMM2020, TEST_DATA_DIR, true));

  ASSERT_NO_THROW(magnetic_model::MagneticModel model(
    log, magnetic_model::MagneticModel::WMM2025, TEST_DATA_DIR, true));

  ASSERT_THROW(magnetic_model::MagneticModel model(
    log, "nonexisting", TEST_DATA_DIR, true), std::invalid_argument);
}

TEST(MagneticModel, GetField)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModel model(log, magnetic_model::MagneticModel::WMM2020, TEST_DATA_DIR, true);

  sensor_msgs::NavSatFix fix;
  fix.header.frame_id = "test";
  fix.header.stamp = cras::parseTime("2024-11-18T13:00:00Z");
  fix.latitude = 51;
  fix.longitude = 10;
  fix.altitude = 200;
  const auto result = model.getMagneticField(fix, fix.header.stamp);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(fix.header, result->field.header);
  EXPECT_NEAR(1.33e-06, result->field.magnetic_field.x, 1e-6);
  EXPECT_NEAR(1.95e-05, result->field.magnetic_field.y, 1e-6);
  EXPECT_NEAR(-4.54e-05, result->field.magnetic_field.z, 1e-6);
  EXPECT_NEAR(1.7161e-14, result->field.magnetic_field_covariance[0 * 3 + 0], 1e-18);
  EXPECT_NEAR(8.8360e-15, result->field.magnetic_field_covariance[1 * 3 + 1], 1e-18);
  EXPECT_NEAR(2.4649e-14, result->field.magnetic_field_covariance[2 * 3 + 2], 1e-18);
  EXPECT_NEAR(5.946503e-08, result->dt.x, 1e-12);
  EXPECT_NEAR(2.142373e-10, result->dt.y, 1e-12);
  EXPECT_NEAR(-5.489079e-08, result->dt.z, 1e-12);
  EXPECT_NEAR(1.31e-07, result->error.x, 1e-12);
  EXPECT_NEAR(9.40e-08, result->error.y, 1e-12);
  EXPECT_NEAR(1.57e-07, result->error.z, 1e-12);
}

TEST(MagneticModel, GetFieldWrongYearStrict)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModel model(log, magnetic_model::MagneticModel::WMM2020, TEST_DATA_DIR, true);

  sensor_msgs::NavSatFix fix;
  fix.header.frame_id = "test";
  fix.header.stamp = cras::parseTime("2014-11-18T13:00:00Z");

  EXPECT_FALSE(model.getMagneticField(fix, fix.header.stamp).has_value());
  EXPECT_FALSE(model.getMagneticFieldComponents(fix, fix.header.stamp).has_value());
}

TEST(MagneticModel, GetFieldWrongYearNonStrict)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModel model(log, magnetic_model::MagneticModel::WMM2020, TEST_DATA_DIR, false);

  sensor_msgs::NavSatFix fix;
  fix.header.frame_id = "test";
  fix.header.stamp = cras::parseTime("2014-11-18T13:00:00Z");

  EXPECT_TRUE(model.getMagneticField(fix, fix.header.stamp).has_value());
  EXPECT_TRUE(model.getMagneticFieldComponents(fix, fix.header.stamp).has_value());
}

TEST(MagneticModel, GetFieldWrongAltitude)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModel model(log, magnetic_model::MagneticModel::WMM2020, TEST_DATA_DIR, true);

  sensor_msgs::NavSatFix fix;
  fix.header.frame_id = "test";
  fix.header.stamp = cras::parseTime("2014-11-18T13:00:00Z");
  fix.altitude = 1e6;

  EXPECT_FALSE(model.getMagneticField(fix, fix.header.stamp).has_value());
  EXPECT_FALSE(model.getMagneticFieldComponents(fix, fix.header.stamp).has_value());
}

TEST(MagneticModel, GetFieldComponentsFromField)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModel model(log, magnetic_model::MagneticModel::WMM2020, TEST_DATA_DIR, true);

  const auto stamp = cras::parseTime("2024-11-18T13:00:00Z");

  magnetic_model::MagneticField field;
  field.field.header.frame_id = "test";
  field.field.header.stamp = stamp;
  field.field.magnetic_field.x = 1.33e-06;
  field.field.magnetic_field.y = 1.95e-05;
  field.field.magnetic_field.z = -4.54e-05;
  field.dt.x = 5.946503e-08;
  field.dt.y = 2.142373e-10;
  field.dt.z = -5.489079e-08;

  const auto result = model.getMagneticFieldComponents(field, stamp);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(1.95e-05, result->values.horizontalMagnitude, 1e-6);
  EXPECT_NEAR(4.95e-05, result->values.totalMagnitude, 1e-6);
  EXPECT_NEAR(angles::from_degrees(4.04), result->values.declination, 1e-3);
  EXPECT_NEAR(1.165, result->values.inclination, 1e-3);
  EXPECT_NEAR(4.2601598e-09, result->dt.horizontalMagnitude, 1e-12);
  EXPECT_NEAR(5.210166e-08, result->dt.totalMagnitude, 1e-12);
  EXPECT_NEAR(3.0346225e-03, result->dt.declination, 1e-6);
  EXPECT_NEAR(3.600e-04, result->dt.inclination, 1e-6);
  EXPECT_NEAR(1.28e-07, result->errors.horizontalMagnitude, 1e-9);
  EXPECT_NEAR(1.45e-07, result->errors.totalMagnitude, 1e-9);
  EXPECT_NEAR(0.007, result->errors.declination, 1e-3);
  EXPECT_NEAR(0.004, result->errors.inclination, 1e-3);

  EXPECT_FALSE(model.getMagneticFieldComponents(field, cras::parseTime("2014-11-18T13:00:00Z")).has_value());
}

TEST(MagneticModel, GetFieldComponentsFromFix)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModel model(log, magnetic_model::MagneticModel::WMM2020, TEST_DATA_DIR, true);

  sensor_msgs::NavSatFix fix;
  fix.header.frame_id = "test";
  fix.header.stamp = cras::parseTime("2024-11-18T13:00:00Z");
  fix.latitude = 51;
  fix.longitude = 10;
  fix.altitude = 200;

  const auto result = model.getMagneticFieldComponents(fix, fix.header.stamp);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(1.95e-05, result->values.horizontalMagnitude, 1e-6);
  EXPECT_NEAR(4.95e-05, result->values.totalMagnitude, 1e-6);
  EXPECT_NEAR(angles::from_degrees(4.04), result->values.declination, 1e-2);
  EXPECT_NEAR(1.165, result->values.inclination, 1e-2);
  EXPECT_NEAR(4.2601598e-09, result->dt.horizontalMagnitude, 1e-10);
  EXPECT_NEAR(5.210166e-08, result->dt.totalMagnitude, 1e-10);
  EXPECT_NEAR(3.0346225e-03, result->dt.declination, 1e-5);
  EXPECT_NEAR(3.600e-04, result->dt.inclination, 1e-5);
  EXPECT_NEAR(1.28e-07, result->errors.horizontalMagnitude, 1e-9);
  EXPECT_NEAR(1.45e-07, result->errors.totalMagnitude, 1e-9);
  EXPECT_NEAR(0.007, result->errors.declination, 1e-3);
  EXPECT_NEAR(0.004, result->errors.inclination, 1e-3);

  // Use with wrong year
  EXPECT_FALSE(model.getMagneticFieldComponents(fix, cras::parseTime("2014-11-18T13:00:00Z")).has_value());
  // Use with wrong altitude
  fix.altitude = 1e6;
  EXPECT_FALSE(model.getMagneticFieldComponents(fix, fix.header.stamp).has_value());
}

TEST(MagneticModelManager, GetBestModelName)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModelManager manager(log, TEST_DATA_DIR);

  EXPECT_EQ(
    magnetic_model::MagneticModel::WMM2025,
    manager.getBestMagneticModelName(cras::parseTime("2029-11-18T13:00:00Z")));

  EXPECT_EQ(
    magnetic_model::MagneticModel::WMM2020,
    manager.getBestMagneticModelName(cras::parseTime("2024-11-18T13:00:00Z")));

  EXPECT_EQ(
    magnetic_model::MagneticModel::WMM2015,
    manager.getBestMagneticModelName(cras::parseTime("2019-11-18T13:00:00Z")));

  EXPECT_EQ(
    magnetic_model::MagneticModel::WMM2010,
    manager.getBestMagneticModelName(cras::parseTime("2014-11-18T13:00:00Z")));

  EXPECT_EQ(
    magnetic_model::MagneticModel::WMM2010,
    manager.getBestMagneticModelName(cras::parseTime("2004-11-18T13:00:00Z")));

  EXPECT_EQ(
    magnetic_model::MagneticModel::WMM2025,
    manager.getBestMagneticModelName(cras::parseTime("2034-11-18T13:00:00Z")));
}

TEST(MagneticModelManager, GetModelByName)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModelManager manager(log, TEST_DATA_DIR);

  ASSERT_TRUE(manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, true).has_value());
  ASSERT_TRUE(manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, false).has_value());
  EXPECT_NE(
    *manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, true),
    *manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, false));

  const auto model = *manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, true);
  EXPECT_TRUE(model->isValid(2024));
  EXPECT_TRUE(model->isValid(2020));
  EXPECT_FALSE(model->isValid(2025));
}

TEST(MagneticModelManager, SetModelPath)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModelManager manager(log, TEST_DATA_DIR);

  EXPECT_TRUE(manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, true).has_value());
  EXPECT_EQ(TEST_DATA_DIR, manager.getModelPath());

  manager.setModelPath("/nonexistent/path");
  EXPECT_FALSE(manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, true).has_value());
  EXPECT_EQ("/nonexistent/path", manager.getModelPath());

  // Use the default location from GeographicLib. We don't know if the user has downloaded the model or not into the
  // default location, so we cannot presume validity of the model. Instead, we just check that the default path gets
  // resolved to a nonempty path.
  manager.setModelPath("");
  EXPECT_FALSE(manager.getModelPath().empty());
  EXPECT_NE("/nonexistent/path", manager.getModelPath());

  const auto oldPath = getenv("GEOGRAPHICLIB_MAGNETIC_PATH");
  setenv("GEOGRAPHICLIB_MAGNETIC_PATH", TEST_DATA_DIR, true);
  manager.setModelPath("");
  EXPECT_TRUE(manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, true).has_value());
  EXPECT_EQ(TEST_DATA_DIR, manager.getModelPath());
  if (oldPath == nullptr)
    unsetenv("GEOGRAPHICLIB_MAGNETIC_PATH");
  else
    setenv("GEOGRAPHICLIB_MAGNETIC_PATH", oldPath, true);

  manager.setModelPath(cras::nullopt);
  EXPECT_TRUE(manager.getMagneticModel(magnetic_model::MagneticModel::WMM2020, true).has_value());
  EXPECT_FALSE(manager.getModelPath().empty());
}

TEST(MagneticModelManager, GetModelByTime)
{
  cras::LogHelperPtr log(new cras::MemoryLogHelper);
  magnetic_model::MagneticModelManager manager(log, TEST_DATA_DIR);

  ASSERT_TRUE(manager.getMagneticModel(cras::parseTime("2024-11-18T13:00:00Z"), true).has_value());
  ASSERT_TRUE(manager.getMagneticModel(cras::parseTime("2014-11-18T13:00:00Z"), true).has_value());
  EXPECT_NE(
    *manager.getMagneticModel(cras::parseTime("2024-11-18T13:00:00Z"), true),
    *manager.getMagneticModel(cras::parseTime("2014-11-18T13:00:00Z"), true));

  auto model = *manager.getMagneticModel(cras::parseTime("2024-11-18T13:00:00Z"), true);
  EXPECT_TRUE(model->isValid(2024));
  EXPECT_TRUE(model->isValid(2020));
  EXPECT_FALSE(model->isValid(2025));

  EXPECT_FALSE(manager.getMagneticModel(cras::parseTime("2004-11-18T13:00:00Z"), true).has_value());
  EXPECT_TRUE(manager.getMagneticModel(cras::parseTime("2004-11-18T13:00:00Z"), false).has_value());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
