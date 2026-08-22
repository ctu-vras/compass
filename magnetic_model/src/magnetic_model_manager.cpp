// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Earth magnetic field model.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <GeographicLib/MagneticModel.hpp>

#include <ament_index_cpp/get_package_prefix.hpp>
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#define AMENT_INDEX_CPP_DONT_USE_STD_FILESYSTEM
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/format.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <magnetic_model/magnetic_model.hpp>
#include <magnetic_model/magnetic_model_manager.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/time.hpp>

namespace magnetic_model {

/**
 * \brief Private data of MagneticModelManager.
 */
struct MagneticModelManagerPrivate {
  //! \brief Cache of already initialized magnetic field models. Keys are model names/strictness.
  std::map<std::pair<std::string, bool>, std::shared_ptr<MagneticModel>> magnetic_models_;

  //! \brief Path to the models on disk. Empty means system default.
  std::string model_path_;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr log_;
};

MagneticModelManager::MagneticModelManager(RequiredInterfaces node, const std::optional<std::string>& model_path)
    : data_(new MagneticModelManagerPrivate{}), node_(node) {
  data_->log_ = node.get_node_logging_interface();
  setModelPath(model_path);
}

MagneticModelManager::~MagneticModelManager() = default;

std::string MagneticModelManager::getModelPath() const{
  return data_->model_path_;
}

void MagneticModelManager::setModelPath(const std::optional<std::string>& model_path) {
  if (model_path.has_value()) {
    if (model_path->empty()) {
      data_->model_path_ = GeographicLib::MagneticModel::DefaultMagneticPath();
    } else {
      data_->model_path_ = *model_path;
    }
  } else {
    try {
#ifdef AMENT_INDEX_CPP_DONT_USE_STD_FILESYSTEM
      data_->model_path_ = ament_index_cpp::get_package_share_directory("magnetic_model") + "/data/magnetic";
#else
      data_->model_path_ =
        (ament_index_cpp::get_package_share_path("magnetic_model") / "data" / "magnetic").string();
#endif
    } catch (const ament_index_cpp::PackageNotFoundError&) {
      RCLCPP_ERROR(data_->log_->get_logger(),
        "Could not resolve package magnetic_model. Is the workspace properly sourced?");
      data_->model_path_ = GeographicLib::MagneticModel::DefaultMagneticPath();
    }
  }

  data_->magnetic_models_.clear();

  RCLCPP_INFO(data_->log_->get_logger(), "Using WMM models from directory %s.", data_->model_path_.c_str());
}

std::string MagneticModelManager::getBestMagneticModelName(const rclcpp::Time& date) const{
  // If the conversion failed, year would be 0, thus triggering the last branch.
  const auto year = cras::getYear(date);
  if (year >= 2025) {
    return MagneticModel::WMM2025;
  } else if (year >= 2020) {
    return MagneticModel::WMM2020;
  } else if (year >= 2015) {
    return MagneticModel::WMM2015;
  } else if (year >= 2010) {
    return MagneticModel::WMM2010;
  } else {
    return MagneticModel::IGRF14;
  }
}

cras::expected<std::shared_ptr<MagneticModel>, std::string> MagneticModelManager::getMagneticModel(
    const rclcpp::Time& stamp, const bool strict) const {
  const auto name = getBestMagneticModelName(stamp);
  const auto model = getMagneticModel(name, strict);
  if (!model.has_value()) {
    return cras::make_unexpected(model.error());
  }
  if (strict && !model.value()->isValid(stamp)) {
    return cras::make_unexpected(cras::format(
      "The best magnetic model {} is not valid at time {}.", name, cras::to_pretty_string(stamp)));
  }
  return *model;
}

cras::expected<std::shared_ptr<MagneticModel>, std::string> MagneticModelManager::getMagneticModel(
    const std::string& name, const bool strict) const {
  const auto key = std::make_pair(name, strict);
  if (!data_->magnetic_models_.contains(key)) {
    try {
      data_->magnetic_models_[key] = std::make_shared<MagneticModel>(
        node_, name, data_->model_path_, strict);
    } catch (const std::invalid_argument& e) {
      return cras::make_unexpected(e.what());
    }
  }

  return data_->magnetic_models_[key];
}

}  // namespace magnetic_model
