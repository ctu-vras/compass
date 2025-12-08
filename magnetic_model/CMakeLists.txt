# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(magnetic_model)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(angles REQUIRED)
find_package(cras_cpp_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

ament_export_dependencies(
  cras_cpp_common
  geometry_msgs
  rclcpp
  sensor_msgs
)

# LIBRARY
add_library(${PROJECT_NAME} SHARED
  src/magnetic_model_manager.cpp
  src/magnetic_model.cpp
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS}
  PUBLIC cras_cpp_common::cras_expected rclcpp::rclcpp
  PRIVATE ament_index_cpp::ament_index_cpp angles::angles cras_cpp_common::cras_cpp_common GeographicLib)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY share/data DESTINATION share/${PROJECT_NAME})

add_library(${PROJECT_NAME}_version INTERFACE)
ament_generate_version_header(${PROJECT_NAME}_version)

# Install exported libs
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_version
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} angles::angles cras_cpp_common::cras_string_utils)
  target_include_directories(test_${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  )
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE -DTEST_DATA_DIR="${CMAKE_SOURCE_DIR}/share/data/magnetic")
endif()

ament_package()
