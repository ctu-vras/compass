^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package magnetometer_compass
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
