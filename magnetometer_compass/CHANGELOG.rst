.. SPDX-License-Identifier: BSD-3-Clause
.. SPDX-FileCopyrightText: Czech Technical University in Prague

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package magnetometer_compass
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2024-12-20)
------------------

2.0.2 (2024-12-20)
------------------
* Fixed README rendering on ROS Wiki.
* Fixed nodelet initialization.
* Contributors: Martin Pecka

2.0.1 (2024-12-12)
------------------
* Fixed license issues preventing bloom-release.
* Fixed documentation generation.
* Contributors: Martin Pecka

2.0.0 (2024-12-12)
------------------
* Restructured the repo, created compass_conversions, magnetic_model and magnetometer_pipeline packages.
  The behavior of the compass nodelet should not change.
  The visualize_azimuth Python script was substituted by a C++ node with the same name and compatible behavior.
  The data directory with WMM was moved from magnetometer_compass to magnetic_model.
* Contributors: Martin Pecka

1.0.3 (2023-07-12)
------------------
* Fixed installation of data
* Contributors: Martin Pecka

1.0.2 (2023-07-12)
------------------
* Renamed compass -> magnetometer_compass
* Contributors: Martin Pecka

1.0.1 (2023-06-19)
------------------
* Made use of cras_cpp_common's node_from_nodelet CMake macro.
* Removed imu_transformer workaround
* Added option to force magnetic declination value.
* Added visualization of azimuth.
* Fixed IMU orientation covariance transformation.
* Renamed magnetometer_compass package to compass.
* Added variance to magnetometer compass outputs. It now also outputs fully valid georeferenced IMU messages instead of just writing the orientation into them.
* Contributors: Martin Pecka
