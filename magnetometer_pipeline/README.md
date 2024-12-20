<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# magnetometer\_pipeline

Calibration and removing of magnetometer bias.

## Node magnetometer\_bias\_remover and nodelet magnetometer\_pipeline/magnetometer\_bias\_remover

For the magnetometer to work correctly, it is required to measure its bias. This node listens on the `imu/mag_bias`
topic for this measurement, and until at least one message arrives, the node will not publish anything. If you do not
have a node publishing the bias, you can alternatively provide it via parameters. Depending on the application, it
may be required to re-estimate the bias from time to time even during runtime.

### Subscribed topics
- `imu/mag` (`sensor_msgs/MagneticField`): 3-axis magnetometer measurements (bias not removed).
- `imu/mag_bias` (`sensor_msgs/MagneticField`): Bias of the magnetometer. This value will be subtracted from the
    incoming magnetometer measurements. Messages on this topic do not need to come repeatedly if the bias does not
    change. The `magnetic_field_covariance` field can be "misused" to carry a 3x3 bias scaling matrix.

### Published topics (see above for explanation)
- `imu/mag_unbiased` (`sensor_msgs/MagneticField`): The magnetic field measurement with bias removed.

If you want to remap all topics under a different namespace than `imu`, it is sufficient to remap just `imu:=new_imu`.

### Parameters
- `~initial_mag_bias_x` (double, no default, optional): Magnetometer bias in the X axis.
- `~initial_mag_bias_y` (double, no default, optional): Magnetometer bias in the Y axis.
- `~initial_mag_bias_z` (double, no default, optional): Magnetometer bias in the Z axis.
- `~initial_scaling_matrix` (double\[9\], optional): Magnetometer scaling matrix (row-major).
- If you specify any of the `~initial_mag_bias_*` params, the node does not need to receive the bias messages.

## Node magnetometer\_bias\_observer

Magnetometer bias estimation node.

### Subscribed topics

- `imu/mag` (`sensor_msgs/MagneticField`): The raw magnetometer measurements.

### Published topics

- `imu/mag_bias` (`sensor_msgs/MagneticField`): The estimated bias.
- `speak/warn` (`std_msgs/String`): Optional topic on which the node reports user instructions.

### Provided services

- `calibrate_magnetometer` (`std_srvs/Trigger`): Call this service to start bias estimation. Rotate the robot in as much
    axes as possible during the calibration.

### Parameters

- `~measuring_time` (double, default 30 s): How long should the bias estimation phase be.
- `~2d_mode` (bool, default true): If true, the calibration expects motion in only 2 axes instead of 3.
- `~2d_mode_ignore_axis` ('X', 'Y' or 'Z', default autodetect): If you know which magnetometer local axis will not be
    used in 2D calibration, you can set it here.
- `~load_from_params` (bool, default false): If true, initial bias estimate will be loaded from ROS params.
- `magnetometer_bias_x` (double, default 0.0): The initial bias estimate for X axis (if `~load_from_params` is true).
- `magnetometer_bias_y` (double, default 0.0): The initial bias estimate for Y axis (if `~load_from_params` is true).
- `magnetometer_bias_z` (double, default 0.0): The initial bias estimate for Z axis (if `~load_from_params` is true).
- `~load_from_file` (bool, default true): If true, the initial bias estimate will be loaded from
    `~/.ros/magnetometer_calib.yaml`.
- `~calibration_file_path` (str, default `~/.ros/magnetometer_calib.yaml`): Path to the calibration file.
- `~save_to_file` (bool, default true): If true, the last estimated bias will be saved to the calibration file.

## `MagnetometerBiasRemover` class

Helper class to remove known bias from 3-axis magnetometer.

## `BiasRemoverFilter` message filter

Message filter providing magnetometer measurements with removed bias.