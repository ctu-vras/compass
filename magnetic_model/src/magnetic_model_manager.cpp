// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Earth magnetic field model.
 * \author Martin Pecka
 */

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <GeographicLib/MagneticModel.hpp>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <magnetic_model/magnetic_model.h>
#include <magnetic_model/magnetic_model_manager.h>
#include <ros/package.h>
#include <ros/time.h>

namespace magnetic_model
{

/**
 * \brief Private data of MagneticModelManager.
 */
struct MagneticModelManagerPrivate
{
  //! \brief Cache of already initialized magnetic field models. Keys are model names/strictness.
  std::map<std::pair<std::string, bool>, std::shared_ptr<MagneticModel>> magneticModels;

  //! \brief Path to the models on disk. Empty means system default.
  std::string modelPath;
};

MagneticModelManager::MagneticModelManager(const cras::LogHelperPtr& log, const cras::optional<std::string>& modelPath):
  cras::HasLogger(log), data(new MagneticModelManagerPrivate{})
{
  this->setModelPath(modelPath);
}

MagneticModelManager::~MagneticModelManager() = default;

std::string MagneticModelManager::getModelPath() const
{
  return this->data->modelPath;
}

void MagneticModelManager::setModelPath(const cras::optional<std::string>& modelPath)
{
  if (modelPath.has_value())
  {
    if (modelPath->empty())
      this->data->modelPath = GeographicLib::MagneticModel::DefaultMagneticPath();
    else
      this->data->modelPath = *modelPath;
  }
  else
  {
    const auto packagePath = ros::package::getPath("magnetic_model");
    if (!packagePath.empty())
    {
      this->data->modelPath = packagePath + "/data/magnetic";
    }
    else
    {
      CRAS_ERROR("Could not resolve package magnetic_model. Is the workspace properly sourced?");
      this->data->modelPath = GeographicLib::MagneticModel::DefaultMagneticPath();
    }
  }

  this->data->magneticModels.clear();

  CRAS_INFO("Using WMM models from directory %s.", this->data->modelPath.c_str());
}

std::string MagneticModelManager::getBestMagneticModelName(const ros::Time& date) const
{
  const auto year = cras::getYear(date);  // If the conversion failed, year would be 0, thus triggering the last branch.
  if (year >= 2020)
    return MagneticModel::WMM2020;
  else if (year >= 2015)
    return MagneticModel::WMM2015;
  else
    return MagneticModel::WMM2010;
}

cras::expected<std::shared_ptr<MagneticModel>, std::string> MagneticModelManager::getMagneticModel(
  const ros::Time& stamp, const bool strict) const
{
  const auto name = this->getBestMagneticModelName(stamp);
  const auto model = this->getMagneticModel(name, strict);
  if (!model.has_value())
    return cras::make_unexpected(model.error());
  if (strict && !model.value()->isValid(stamp))
    return cras::make_unexpected(cras::format(
      "The best magnetic model %s is not valid at time %s.", name.c_str(), cras::to_pretty_string(stamp).c_str()));
  return *model;
}

cras::expected<std::shared_ptr<MagneticModel>, std::string> MagneticModelManager::getMagneticModel(
  const std::string& name, const bool strict) const
{
  const auto key = std::make_pair(name, strict);
  if (this->data->magneticModels.find(key) == this->data->magneticModels.end())
  {
    try
    {
      this->data->magneticModels[key] = std::make_shared<MagneticModel>(this->log, name, this->data->modelPath, strict);
    }
    catch (const std::invalid_argument& e)
    {
      return cras::make_unexpected(e.what());
    }
  }

  return this->data->magneticModels[key];
}

}
