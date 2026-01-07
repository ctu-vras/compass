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
target_link_libraries(magnetometer_bias_remover
  PUBLIC ${sensor_msgs_TARGETS} cras_cpp_common::cras_expected rclcpp::rclcpp
  PRIVATE cras_cpp_common::cras_cpp_common Eigen3::Eigen tf2_eigen::tf2_eigen)
target_include_directories(magnetometer_bias_remover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# LIBRARY 2
add_library(magnetometer_bias_remover_filter SHARED
  src/message_filter.cpp
)
target_link_libraries(magnetometer_bias_remover_filter
  PUBLIC magnetometer_bias_remover
  PUBLIC ${sensor_msgs_TARGETS} message_filters::message_filters rclcpp::rclcpp)
target_include_directories(magnetometer_bias_remover_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# LIBRARY 3
add_library(magnetometer_bias_remover_nodelet SHARED
  nodelets/magnetometer_bias_remover_nodelet.cpp
)
target_link_libraries(magnetometer_bias_remover_nodelet
  PUBLIC magnetometer_bias_remover_filter
  PUBLIC ${sensor_msgs_TARGETS} message_filters::message_filters rclcpp::rclcpp
  PRIVATE cras_cpp_common::cras_cpp_common rclcpp_components::component)
target_include_directories(magnetometer_bias_remover_nodelet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(magnetometer_bias_remover_nodelet
  PLUGIN "magnetometer_pipeline::MagnetometerBiasRemoverNodelet"
  EXECUTABLE magnetometer_bias_remover_node)
if(${message_filters_VERSION} VERSION_GREATER_EQUAL 7.1.0)
  target_compile_definitions(magnetometer_bias_remover_nodelet
    PUBLIC MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES=1)
endif()

pluginlib_export_plugin_description_file(rclcpp plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# All shared libraries exported by the project. Do not add component nodes here (they should not be exported).
# All PUBLIC and INTERFACE dependencies of these libraries have to be <depend> and they have to be covered by
# ament_export_dependencies() for each package they depend on.
set(EXPORTED_LIBS
  magnetometer_bias_remover
  magnetometer_bias_remover_filter
)

# Libraries that should not be exported (i.e. external packages will not link to them). E.g. component nodes go here.
# Their dependencies are only <build_depend> and <export_depend> and they need only find_package().
set(NON_EXPORTED_LIBS
  magnetometer_bias_remover_nodelet
)

add_library(${PROJECT_NAME}_version INTERFACE)
ament_generate_version_header(${PROJECT_NAME}_version)

# Create a convenience target ${PROJECT_NAME}::${PROJECT_NAME} that links to all sub-libraries
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE ${EXPORTED_LIBS})

# Install exported libs
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_version ${EXPORTED_LIBS}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Install non-exported libs
install(TARGETS ${NON_EXPORTED_LIBS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(PROGRAMS nodes/magnetometer_bias_observer.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_magnetometer_bias_remover_nodelet test/test_bias_remover_nodelet.cpp)
  target_link_libraries(test_magnetometer_bias_remover_nodelet
    magnetometer_bias_remover_nodelet
    cras_cpp_common::cras_cpp_common)
  target_include_directories(test_magnetometer_bias_remover_nodelet PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

  ament_add_gtest(test_magnetometer_bias_remover test/test_bias_remover.cpp)
  target_link_libraries(test_magnetometer_bias_remover magnetometer_bias_remover)
  target_include_directories(test_magnetometer_bias_remover PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

  ament_add_gtest(test_magnetometer_bias_remover_filter test/test_message_filter.cpp)
  target_link_libraries(test_magnetometer_bias_remover_filter
    magnetometer_bias_remover_filter magnetometer_bias_remover)
  target_include_directories(test_magnetometer_bias_remover_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
endif()

ament_package()
