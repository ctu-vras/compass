# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(magnetic_model)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  angles
  compass_msgs
  cras_cpp_common
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
)

# Ubuntu libgeographic-dev package installs into non-standard location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS compass_msgs cras_cpp_common geometry_msgs roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp src/${PROJECT_NAME}_manager.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} PRIVATE ${GeographicLib_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files

  file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
  file(GLOB_RECURSE ROSLINT_SRC src/*.cpp src/*.hpp src/*.h)
  file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC} ${ROSLINT_TEST})

  roslint_add_test()

  catkin_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE -DTEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data/magnetic")
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
