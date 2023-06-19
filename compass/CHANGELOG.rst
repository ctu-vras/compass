^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package compass
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Made use of cras_cpp_common's node_from_nodelet CMake macro.
* Removed imu_transformer workaround
* Added option to force magnetic declination value.
* Added visualization of azimuth.
* Fixed IMU orientation covariance transformation.
* Renamed magnetometer_compass package to compass.
* Added variance to magnetometer compass outputs. It now also outputs fully valid georeferenced IMU messages instead of just writing the orientation into them.
* Contributors: Martin Pecka
