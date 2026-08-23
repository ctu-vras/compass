# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(compass_conversions)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(compass_interfaces REQUIRED)
find_package(cras_cpp_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(magnetic_model REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_export_dependencies(
  compass_interfaces
  cras_cpp_common
  geometry_msgs
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

include(CMakePushCheckState)

cmake_push_check_state(RESET)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
try_compile(SUBSCRIBER_BASE_IS_TEMPLATE
  ${CMAKE_BINARY_DIR}/is_template
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/message_filters_subscriber_base_is_template.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/dummy.c  # This dummy C file fixes build on Lyrical and newer
  CXX_STANDARD 20 LINK_LIBRARIES message_filters::message_filters)
cmake_pop_check_state()

if(SUBSCRIBER_BASE_IS_TEMPLATE)
  cmake_push_check_state(RESET)
  set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
  try_compile(HAS_NODE_INTERFACES
    ${CMAKE_BINARY_DIR}/node_interfaces
    SOURCES
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/message_filters_has_node_interfaces.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/dummy.c
    CXX_STANDARD 20 LINK_LIBRARIES message_filters::message_filters)
  cmake_pop_check_state()
else()
  set(HAS_NODE_INTERFACES TRUE)
endif()

# There have been quite some changes to the message_filters::SubscriberBase interface recently.
# Jazzy: USES_NODE_INTERFACES=0 BASE_IS_TEMPLATE=1
# Kilted: USES_NODE_INTERFACES=1 BASE_IS_TEMPLATE=1
# Newer: USES_NODE_INTERFACES=1 BASE_IS_TEMPLATE=0
add_library(_message_filters INTERFACE)
target_link_libraries(_message_filters INTERFACE message_filters::message_filters)
if(HAS_NODE_INTERFACES)
  target_compile_definitions(_message_filters INTERFACE MESSAGE_FILTERS_VERSION_SUBSCRIBER_USES_NODE_INTERFACES=1)
endif()
if(SUBSCRIBER_BASE_IS_TEMPLATE)
  target_compile_definitions(_message_filters INTERFACE MESSAGE_FILTERS_VERSION_SUBSCRIBER_BASE_IS_TEMPLATE=1)
endif()

cmake_push_check_state(RESET)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
try_compile(TF2_ROS_HAS_NODE_INTERFACES
  ${CMAKE_BINARY_DIR}/tf2_ros_node_interfaces
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/tf2_ros_has_node_interfaces.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/dummy.c
  CXX_STANDARD 20 LINK_LIBRARIES tf2_ros::tf2_ros)
cmake_pop_check_state()

add_library(_tf2_ros INTERFACE)
target_link_libraries(_tf2_ros INTERFACE tf2_ros::tf2_ros)
if(TF2_ROS_HAS_NODE_INTERFACES)
  target_compile_definitions(_tf2_ros INTERFACE TF2_ROS_HAS_NODE_INTERFACES=1)
endif()

add_library(${PROJECT_NAME}_defaults INTERFACE)
target_link_libraries(${PROJECT_NAME}_defaults INTERFACE cras_cpp_common::cras_defaults)
target_compile_features(${PROJECT_NAME}_defaults INTERFACE $<BUILD_INTERFACE:cxx_std_20>)
target_include_directories(${PROJECT_NAME}_defaults INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(compass_topic_names SHARED src/topic_names.cpp)
target_link_libraries(compass_topic_names
  PUBLIC ${compass_interfaces_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${PROJECT_NAME}_defaults
  PRIVATE cras_cpp_common::cras_cpp_common)

add_library(compass_converter SHARED src/compass_converter.cpp)
target_link_libraries(compass_converter
  PUBLIC compass_topic_names _message_filters ${PROJECT_NAME}_defaults
  PUBLIC ${compass_interfaces_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  PUBLIC cras_cpp_common::cras_expected rclcpp::rclcpp
  PRIVATE angles::angles cras_cpp_common::cras_cpp_common GeographicLib magnetic_model::magnetic_model
  PRIVATE tf2::tf2 tf2_geometry_msgs::tf2_geometry_msgs)

add_library(compass_message_filter SHARED src/message_filter.cpp)
target_link_libraries(compass_message_filter
  PUBLIC compass_converter _message_filters ${PROJECT_NAME}_defaults
  PUBLIC ${compass_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  PUBLIC rclcpp::rclcpp
  PRIVATE cras_cpp_common::cras_cpp_common
)

add_library(tf2_compass_msgs SHARED src/tf2_compass_msgs.cpp)
target_link_libraries(tf2_compass_msgs
  PUBLIC ${PROJECT_NAME}_defaults
  PUBLIC ${compass_interfaces_TARGETS} ${geometry_msgs_TARGETS}
  PUBLIC tf2::tf2
  PRIVATE angles::angles cras_cpp_common::cras_cpp_common rclcpp::rclcpp)

add_library(compass_transformer_nodelet_lib SHARED nodelets/compass_transformer.cpp)
target_link_libraries(compass_transformer_nodelet_lib
  PUBLIC compass_topic_names compass_converter compass_message_filter tf2_compass_msgs _message_filters
  PUBLIC ${PROJECT_NAME}_defaults
  PUBLIC ${compass_interfaces_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS}
  PUBLIC rclcpp::rclcpp _tf2_ros
  PRIVATE cras_cpp_common::cras_cpp_common  rclcpp_components::component)
rclcpp_components_register_node(compass_transformer_nodelet_lib
  PLUGIN "compass_conversions::CompassTransformerNodelet" EXECUTABLE compass_transformer)

pluginlib_export_plugin_description_file(rclcpp plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# All shared libraries exported by the project. Do not add component nodes here (they should not be exported).
# All PUBLIC and INTERFACE dependencies of these libraries have to be <depend> and they have to be covered by
# ament_export_dependencies() for each package they depend on.
set(EXPORTED_LIBS
  compass_topic_names
  compass_converter
  compass_message_filter
  tf2_compass_msgs
  _message_filters
  ${PROJECT_NAME}_defaults
)

# Libraries that should not be exported (i.e. external packages will not link to them). E.g. component nodes go here.
# Their dependencies are only <build_depend> and <export_depend> and they need only find_package().
set(NON_EXPORTED_LIBS
  compass_transformer_nodelet_lib
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

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(tf2_sensor_msgs REQUIRED)

  ament_add_gtest(test_compass_converter test/test_compass_converter.cpp)
  target_link_libraries(test_compass_converter
    angles::angles compass_converter compass_topic_names cras_cpp_common::cras_cpp_common ${PROJECT_NAME}_defaults)
  set_tests_properties(test_compass_converter PROPERTIES ENVIRONMENT "GTEST_CAPTURE=0")

  ament_add_ros_isolated_gtest(test_compass_transformer_nodelet test/test_compass_transformer_nodelet.cpp)
  target_link_libraries(test_compass_transformer_nodelet
    compass_converter compass_message_filter compass_topic_names compass_transformer_nodelet_lib tf2_compass_msgs
    angles::angles cras_cpp_common::cras_cpp_common tf2_geometry_msgs::tf2_geometry_msgs ${PROJECT_NAME}_defaults)

  ament_add_gtest(test_compass_message_filter test/test_message_filter.cpp)
  target_link_libraries(test_compass_message_filter
    compass_converter compass_message_filter compass_topic_names
    angles::angles cras_cpp_common::cras_cpp_common ${PROJECT_NAME}_defaults)

  ament_add_gtest(test_tf2_compass_msgs test/test_tf2_compass_msgs.cpp)
  target_link_libraries(test_tf2_compass_msgs
    tf2_compass_msgs
    angles::angles cras_cpp_common::cras_cpp_common ${PROJECT_NAME}_defaults)

  ament_add_gtest(test_compass_topic_names test/test_topic_names.cpp)
  target_link_libraries(test_compass_topic_names compass_topic_names ${PROJECT_NAME}_defaults)
endif()

ament_package()
