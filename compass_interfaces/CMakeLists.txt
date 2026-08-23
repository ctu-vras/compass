# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(compass_interfaces)

find_package(ament_cmake REQUIRED)
find_package(cras_cpp_common REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Azimuth.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(cras_cpp_common rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_defaults INTERFACE)
target_link_libraries(${PROJECT_NAME}_defaults INTERFACE cras_cpp_common::cras_defaults)
target_compile_features(${PROJECT_NAME}_defaults INTERFACE $<BUILD_INTERFACE:cxx_std_20>)
target_include_directories(${PROJECT_NAME}_defaults INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(${PROJECT_NAME}_string_utils SHARED src/string_utils.cpp)
target_link_libraries(${PROJECT_NAME}_string_utils
  PUBLIC "${cpp_typesupport_target}" ${PROJECT_NAME}_defaults
  PRIVATE cras_cpp_common::cras_format cras_cpp_common::cras_string_utils)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/)

add_library(${PROJECT_NAME}_version INTERFACE)
ament_generate_version_header(${PROJECT_NAME}_version)

install(TARGETS ${PROJECT_NAME}_defaults ${PROJECT_NAME}_version ${PROJECT_NAME}_string_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_string_utils test/test_string_utils.cpp)
  target_link_libraries(test_string_utils
    ${PROJECT_NAME}_string_utils "${cpp_typesupport_target}" ${PROJECT_NAME}_defaults
    cras_cpp_common::cras_string_utils)
endif()

ament_package()
