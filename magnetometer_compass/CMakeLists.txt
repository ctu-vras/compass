cmake_minimum_required(VERSION 3.10.2)
project(magnetometer_compass)

find_package(catkin REQUIRED COMPONENTS
  angles
  compass_msgs
  cras_cpp_common
  geometry_msgs
  imu_transformer
  message_filters
  nodelet
  roscpp
  roslib
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Ubuntu libgeographic-dev package installs into non-standard location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(magnetometer_compass_nodelet src/magnetometer_compass_nodelet.cpp)
add_dependencies(magnetometer_compass_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(magnetometer_compass_nodelet ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
cras_node_from_nodelet(magnetometer_compass_nodelet "${PROJECT_NAME}/magnetometer_compass_nodelet.h" ${PROJECT_NAME}::MagnetometerCompassNodelet OUTPUT_NAME magnetometer_compass)

install(TARGETS magnetometer_compass_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/visualize_azimuth
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
