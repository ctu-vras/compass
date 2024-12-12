// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Remove known bias from 3-axis magnetometer.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <sensor_msgs/MagneticField.h>

namespace magnetometer_pipeline
{

struct MagnetometerBiasRemoverPrivate;

/**
 * \brief Remove known bias from 3-axis magnetometer.
 */
class MagnetometerBiasRemover : public cras::HasLogger
{
public:
  explicit MagnetometerBiasRemover(const cras::LogHelperPtr& log);
  virtual ~MagnetometerBiasRemover();

  /**
   * \brief Configure the bias remover from ROS parameters.
   * \param[in] params The parameters.
   *
   * The following parameters are read:
   * - `~initial_mag_bias_x` (double, no default, optional): Magnetometer bias in the X axis.
   * - `~initial_mag_bias_y` (double, no default, optional): Magnetometer bias in the Y axis.
   * - `~initial_mag_bias_z` (double, no default, optional): Magnetometer bias in the Z axis.
   * - `~initial_scaling_matrix` (double[9], optional): Magnetometer scaling matrix (row-major).
   */
  void configFromParams(const cras::BoundParamHelper& params);

  /**
   * \brief Whether bias has already been set.
   * \return Whether bias has already been set.
   */
  bool hasBias() const;

  /**
   * \brief Whether bias has already been set and is different from identity.
   * \return Whether bias has already been set.
   */
  bool hasScale() const;

  /**
   * \brief Callback for magnetometer bias.
   * \param[in] bias The bias value. If `magnetic_field_covariance` has non-zero elements, it is interpreted
   *                 as a row-wise scaling matrix.
   */
  void setBias(const sensor_msgs::MagneticField& bias);

  /**
   * \brief Callback for magnetometer measurements.
   * \param[in] mag The raw measured magnetic field strength.
   * \return The measured magnetic field corrected for bias, or error message.
   */
  cras::expected<sensor_msgs::MagneticField, std::string> removeBias(const sensor_msgs::MagneticField& mag);

private:
  std::unique_ptr<MagnetometerBiasRemoverPrivate> data;  //!< PIMPL
};
}
