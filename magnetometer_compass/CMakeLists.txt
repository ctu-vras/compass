# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(magnetometer_compass)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(compass_conversions REQUIRED)
find_package(compass_interfaces REQUIRED)
find_package(cras_cpp_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(magnetometer_pipeline REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

ament_export_dependencies(
  cras_cpp_common
  compass_interfaces
  Eigen3
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

# LIBRARY 0 - do not use in outside projects
add_library(_tf2_sensor_msgs INTERFACE)
target_sources(_tf2_sensor_msgs INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/tf2_sensor_msgs.hpp>)
target_include_directories(_tf2_sensor_msgs INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
if(tf2_sensor_msgs_VERSION VERSION_GREATER_EQUAL 0.44.0)
  target_link_libraries(_tf2_sensor_msgs INTERFACE tf2_sensor_msgs::tf2_sensor_msgs)
  target_compile_definitions(_tf2_sensor_msgs INTERFACE TF2_SENSOR_MSGS_HAS_IMU=1)
else()
  target_link_libraries(_tf2_sensor_msgs INTERFACE
    ${sensor_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    Eigen3::Eigen
  )
endif()

# LIBRARY 1
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${compass_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  compass_conversions::compass_converter
  cras_cpp_common::cras_expected
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  angles::angles
  cras_cpp_common::cras_tf2_utils
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  _tf2_sensor_msgs
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

# LIBRARY 2
add_library(${PROJECT_NAME}_nodelet_lib SHARED
  nodelets/${PROJECT_NAME}_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet_lib PUBLIC
  ${compass_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  compass_conversions::compass_converter
  compass_conversions::compass_topic_names
  ${magnetometer_pipeline_TARGETS}
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
  ${tf2_ros_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_nodelet_lib PUBLIC ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_nodelet_lib PRIVATE
  cras_cpp_common::cras_string_utils
  cras_cpp_common::cras_tf2_utils
  tf2_geometry_msgs::tf2_geometry_msgs
  _tf2_sensor_msgs
)
target_include_directories(${PROJECT_NAME}_nodelet_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
if(${message_filters_VERSION} VERSION_GREATER_EQUAL 7.1.0)
  target_compile_definitions(${PROJECT_NAME}_nodelet_lib PUBLIC
    MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES=1)
endif()
rclcpp_components_register_node(${PROJECT_NAME}_nodelet_lib
  PLUGIN "magnetometer_compass::MagnetometerCompassNodelet"
  EXECUTABLE ${PROJECT_NAME}_nodelet)

# LIBRARY 3
add_library(visualize_azimuth_nodelet_lib SHARED
  nodelets/visualize_azimuth_nodelet.cpp
)
target_link_libraries(visualize_azimuth_nodelet_lib PUBLIC
  ${compass_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  compass_conversions::compass_converter
  compass_conversions::compass_message_filter
  message_filters::message_filters
  rclcpp::rclcpp
)
target_link_libraries(visualize_azimuth_nodelet_lib PRIVATE
  cras_cpp_common::cras_rate_limiter
  rclcpp_components::component
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_include_directories(visualize_azimuth_nodelet_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
if(${message_filters_VERSION} VERSION_GREATER_EQUAL 7.1.0)
  target_compile_definitions(visualize_azimuth_nodelet_lib PUBLIC
    MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES=1)
endif()
rclcpp_components_register_node(visualize_azimuth_nodelet_lib
  PLUGIN "magnetometer_compass::VisualizeAzimuthNodelet"
  EXECUTABLE visualize_azimuth_nodelet)

pluginlib_export_plugin_description_file(rclcpp plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS ${PROJECT_NAME} _tf2_sensor_msgs EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(TARGETS ${PROJECT_NAME}_nodelet_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS visualize_azimuth_nodelet_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(tf2_sensor_msgs REQUIRED)

  ament_add_gtest(test_magnetometer_compass_nodelet test/test_magnetometer_compass_nodelet.cpp)
  target_link_libraries(test_magnetometer_compass_nodelet
    ${builtin_interfaces_TARGETS}
    ${compass_interfaces_TARGETS}
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelet_lib
    angles::angles
    cras_cpp_common::cras_test_utils
    cras_cpp_common::cras_tf2_utils
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_sensor_msgs::tf2_sensor_msgs
    _tf2_sensor_msgs
  )
  target_include_directories(test_magnetometer_compass_nodelet PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

  ament_add_gtest(test_magnetometer_compass test/test_magnetometer_compass.cpp)
  target_link_libraries(test_magnetometer_compass magnetometer_compass angles::angles)
  target_include_directories(test_magnetometer_compass PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )

  ament_add_ros_isolated_gtest(test_visualize_azimuth_nodelet test/test_visualize_azimuth_nodelet.cpp)
  target_link_libraries(test_visualize_azimuth_nodelet
    visualize_azimuth_nodelet_lib
    cras_cpp_common::cras_string_utils
    cras_cpp_common::cras_test_utils
    cras_cpp_common::cras_time_utils
    tf2_geometry_msgs::tf2_geometry_msgs
  )
  target_include_directories(test_visualize_azimuth_nodelet PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
