# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(compass_interfaces)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(cras_cpp_common REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Azimuth.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_string_utils SHARED src/string_utils.cpp)
target_include_directories(${PROJECT_NAME}_string_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_string_utils
  PUBLIC "${cpp_typesupport_target}"
  PRIVATE cras_cpp_common::cras_string_utils)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}_string_utils EXPORT export_${PROJECT_NAME}
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
    ${PROJECT_NAME}_string_utils "${cpp_typesupport_target}" cras_cpp_common::cras_string_utils)
endif()

ament_package()
