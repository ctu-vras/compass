# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(magnetometer_pipeline)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(cras_cpp_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(Eigen3 REQUIRED)

ament_export_dependencies(
  cras_cpp_common
  message_filters
  sensor_msgs
  rclcpp
)

# LIBRARY 1
add_library(magnetometer_bias_remover SHARED
  src/bias_remover.cpp
)
target_link_libraries(magnetometer_bias_remover PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  cras_cpp_common::cras_expected
)
target_link_libraries(magnetometer_bias_remover PRIVATE
  cras_cpp_common::cras_string_utils
  Eigen3::Eigen
  tf2_eigen::tf2_eigen
)
target_include_directories(magnetometer_bias_remover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

# LIBRARY 2
add_library(magnetometer_bias_remover_filter SHARED
  src/message_filter.cpp
)
target_link_libraries(magnetometer_bias_remover_filter PUBLIC
  ${sensor_msgs_TARGETS}
  message_filters::message_filters
  rclcpp::rclcpp
)
target_link_libraries(magnetometer_bias_remover_filter PUBLIC magnetometer_bias_remover)
target_include_directories(magnetometer_bias_remover_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

# LIBRARY 3
add_library(magnetometer_bias_remover_nodelet SHARED
  nodelets/magnetometer_bias_remover_nodelet.cpp
)
target_link_libraries(magnetometer_bias_remover_nodelet PUBLIC
  ${sensor_msgs_TARGETS}
  message_filters::message_filters
  rclcpp::rclcpp
)
target_link_libraries(magnetometer_bias_remover_nodelet PUBLIC magnetometer_bias_remover_filter)
target_link_libraries(magnetometer_bias_remover_nodelet PRIVATE
  cras_cpp_common::cras_string_utils
  rclcpp_components::component
)
target_include_directories(magnetometer_bias_remover_nodelet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(magnetometer_bias_remover_nodelet
  PLUGIN "magnetometer_pipeline::MagnetometerBiasRemoverNodelet"
  EXECUTABLE magnetometer_bias_remover_node
)

if(${message_filters_VERSION} VERSION_GREATER_EQUAL 7.1.0)
  target_compile_definitions(magnetometer_bias_remover_nodelet
    PUBLIC MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES=1)
endif()

pluginlib_export_plugin_description_file(rclcpp plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS magnetometer_bias_remover magnetometer_bias_remover_filter EXPORT export_magnetometer_pipeline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
ament_export_targets(export_magnetometer_pipeline HAS_LIBRARY_TARGET)

install(TARGETS magnetometer_bias_remover_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(PROGRAMS nodes/magnetometer_bias_observer.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_magnetometer_bias_remover_nodelet test/test_bias_remover_nodelet.cpp)
  target_link_libraries(test_magnetometer_bias_remover_nodelet
    magnetometer_bias_remover_nodelet cras_cpp_common::cras_string_utils)
  target_include_directories(test_magnetometer_bias_remover_nodelet PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(test_magnetometer_bias_remover test/test_bias_remover.cpp)
  target_link_libraries(test_magnetometer_bias_remover magnetometer_bias_remover)
  target_include_directories(test_magnetometer_bias_remover PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(test_magnetometer_bias_remover_filter test/test_message_filter.cpp)
  target_link_libraries(test_magnetometer_bias_remover_filter
    magnetometer_bias_remover_filter magnetometer_bias_remover)
  target_include_directories(test_magnetometer_bias_remover_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
