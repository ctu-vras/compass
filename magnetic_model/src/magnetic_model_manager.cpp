// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Earth magnetic field model.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <format>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <GeographicLib/MagneticModel.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <magnetic_model/magnetic_model.hpp>
#include <magnetic_model/magnetic_model_manager.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/time.hpp>

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

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr log;
};

MagneticModelManager::MagneticModelManager(RequiredInterfaces node, const std::optional<std::string>& modelPath)
  : data(new MagneticModelManagerPrivate{}), node(node)
{
  this->data->log = node.get_node_logging_interface();
  this->setModelPath(modelPath);
}

MagneticModelManager::~MagneticModelManager() = default;

std::string MagneticModelManager::getModelPath() const
{
  return this->data->modelPath;
}

void MagneticModelManager::setModelPath(const std::optional<std::string>& modelPath)
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
    const auto packagePath = ament_index_cpp::get_package_share_directory("magnetic_model");
    if (!packagePath.empty())
    {
      this->data->modelPath = packagePath + "/data/magnetic";
    }
    else
    {
      RCLCPP_ERROR(this->data->log->get_logger(),
        "Could not resolve package magnetic_model. Is the workspace properly sourced?");
      this->data->modelPath = GeographicLib::MagneticModel::DefaultMagneticPath();
    }
  }

  this->data->magneticModels.clear();

  RCLCPP_INFO(this->data->log->get_logger(), "Using WMM models from directory %s.", this->data->modelPath.c_str());
}

std::string MagneticModelManager::getBestMagneticModelName(const rclcpp::Time& date) const
{
  // If the conversion failed, year would be 0, thus triggering the last branch.
  const auto year = cras::getYear(date);
  if (year >= 2025)
    return MagneticModel::WMM2025;
  else if (year >= 2020)
    return MagneticModel::WMM2020;
  else if (year >= 2015)
    return MagneticModel::WMM2015;
  else if (year >= 2010)
    return MagneticModel::WMM2010;
  else
    return MagneticModel::IGRF14;
}

cras::expected<std::shared_ptr<MagneticModel>, std::string> MagneticModelManager::getMagneticModel(
  const rclcpp::Time& stamp, const bool strict) const
{
  const auto name = this->getBestMagneticModelName(stamp);
  const auto model = this->getMagneticModel(name, strict);
  if (!model.has_value())
    return cras::make_unexpected(model.error());
  if (strict && !model.value()->isValid(stamp))
    return cras::make_unexpected(std::format(
      "The best magnetic model {} is not valid at time {}.", name, cras::to_pretty_string(stamp)));
  return *model;
}

cras::expected<std::shared_ptr<MagneticModel>, std::string> MagneticModelManager::getMagneticModel(
  const std::string& name, const bool strict) const
{
  const auto key = std::make_pair(name, strict);
  if (!this->data->magneticModels.contains(key))
  {
    try
    {
      this->data->magneticModels[key] = std::make_shared<MagneticModel>(
        this->node, name, this->data->modelPath, strict);
    }
    catch (const std::invalid_argument& e)
    {
      return cras::make_unexpected(e.what());
    }
  }

  return this->data->magneticModels[key];
}

}
