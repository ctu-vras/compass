// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Remove known bias from 3-axis magnetometer.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <Eigen/Core>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/param_utils/get_param_specializations/eigen.hpp>
#include <magnetometer_pipeline/bias_remover.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2_eigen/tf2_eigen.h>

namespace magnetometer_pipeline
{
using Field = sensor_msgs::MagneticField;

struct MagnetometerBiasRemoverPrivate
{
  //! \brief Whether magnetometer bias has already been established either from subscriber or initial parameters.
  bool hasBias{false};

  //! \brief Whether magnetometer scale has already been established either from subscriber or initial parameters and
  //!        it is different from identity.
  bool hasScale{false};

  //! \brief Last received value of magnetometer bias.
  Eigen::Vector3d lastBias{0, 0, 0};

  //! \brief Scaling factor of magnetometer measurements (optional).
  Eigen::Matrix3d lastScale{Eigen::Matrix3d::Identity()};
};

MagnetometerBiasRemover::MagnetometerBiasRemover(const cras::LogHelperPtr& log) :
  cras::HasLogger(log), data(new MagnetometerBiasRemoverPrivate{})
{
}

MagnetometerBiasRemover::~MagnetometerBiasRemover() = default;

void MagnetometerBiasRemover::configFromParams(const cras::BoundParamHelper& params)
{
  if (params.hasParam("initial_mag_bias_x") || params.hasParam("initial_mag_bias_y") ||
    params.hasParam("initial_mag_bias_z"))
  {
    sensor_msgs::MagneticField msg;
    msg.magnetic_field.x = params.getParam("initial_mag_bias_x", 0.0, "T");
    msg.magnetic_field.y = params.getParam("initial_mag_bias_y", 0.0, "T");
    msg.magnetic_field.z = params.getParam("initial_mag_bias_z", 0.0, "T");

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> scalingMatrix(msg.magnetic_field_covariance.data());
    scalingMatrix = params.getParam("initial_mag_scaling_matrix", this->data->lastScale);

    this->setBias(msg);

    CRAS_INFO("Initial magnetometer bias is %0.3f %0.3f %0.3f %s scaling factor",
      msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z, this->hasScale() ? "with" : "without");
  }
}

bool MagnetometerBiasRemover::hasBias() const
{
  return this->data->hasBias;
}

bool MagnetometerBiasRemover::hasScale() const
{
  return this->data->hasScale;
}

cras::expected<Field, std::string> MagnetometerBiasRemover::removeBias(const Field& mag)
{
  if (!this->data->hasBias)
    return cras::make_unexpected("Magnetometer bias not available.");

  Eigen::Vector3d field;
  tf2::fromMsg(mag.magnetic_field, field);
  field = this->data->lastScale * (field - this->data->lastBias);

  Field magUnbiased = mag;
  tf2::toMsg(field, magUnbiased.magnetic_field);

  return magUnbiased;
}

void MagnetometerBiasRemover::setBias(const Field& bias)
{
  tf2::fromMsg(bias.magnetic_field, this->data->lastBias);
  this->data->hasBias = true;

  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> covMatrix(bias.magnetic_field_covariance.data());
  if (covMatrix.cwiseAbs().sum() > 1e-10)
    this->data->lastScale = covMatrix;
  this->data->hasScale = (this->data->lastScale - Eigen::Matrix3d::Identity()).cwiseAbs().sum() > 1e-10;
}

}
