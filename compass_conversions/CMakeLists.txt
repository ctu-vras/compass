# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(compass_conversions)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  angles
  compass_msgs
  cras_cpp_common
  geometry_msgs
  magnetic_model
  message_filters
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  topic_tools
)

# Ubuntu libgeographic-dev package installs into non-standard location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES compass_converter compass_message_filter compass_topic_names magnetic_model tf2_compass_msgs
  CATKIN_DEPENDS compass_msgs cras_cpp_common geometry_msgs message_filters roscpp sensor_msgs std_msgs tf2 topic_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(compass_topic_names src/topic_names.cpp)
target_link_libraries(compass_topic_names ${catkin_LIBRARIES})

add_library(compass_converter src/compass_converter.cpp)
add_dependencies(compass_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(compass_converter PUBLIC ${catkin_LIBRARIES} PRIVATE compass_topic_names ${GeographicLib_LIBRARIES})

add_library(compass_message_filter src/message_filter.cpp)
target_link_libraries(compass_message_filter compass_converter)

add_library(tf2_compass_msgs src/tf2_compass_msgs.cpp)
target_link_libraries(tf2_compass_msgs ${catkin_LIBRARIES})

add_library(compass_transformer_nodelet nodelets/compass_transformer.cpp)
target_link_libraries(compass_transformer_nodelet compass_message_filter compass_topic_names tf2_compass_msgs ${catkin_LIBRARIES})
cras_node_from_nodelet(compass_transformer_nodelet ${PROJECT_NAME}::CompassTransformerNodelet OUTPUT_NAME compass_transformer ANONYMOUS)

install(TARGETS compass_converter compass_message_filter compass_topic_names compass_transformer_nodelet tf2_compass_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files

  file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h)
  file(GLOB_RECURSE ROSLINT_SRC src/*.cpp nodelets/*.cpp)
  file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC} ${ROSLINT_TEST})

  roslint_add_test()

  catkin_add_gtest(test_compass_converter test/test_compass_converter.cpp)
  target_link_libraries(test_compass_converter compass_converter)

  catkin_add_gtest(test_message_filter test/test_message_filter.cpp)
  target_link_libraries(test_message_filter compass_message_filter)

  catkin_add_gtest(test_tf2_compass_msgs test/test_tf2_compass_msgs.cpp)
  target_link_libraries(test_tf2_compass_msgs tf2_compass_msgs)

  catkin_add_gtest(test_compass_topic_names test/test_topic_names.cpp)
  target_link_libraries(test_compass_topic_names compass_topic_names)

  add_rostest_gtest(test_compass_transformer_nodelet test/test_compass_transformer_nodelet.test test/test_compass_transformer_nodelet.cpp)
  target_link_libraries(test_compass_transformer_nodelet ${catkin_LIBRARIES} compass_transformer_nodelet)
  if(CMAKE_VERSION VERSION_LESS "3.13.0")
    set_property(TARGET test_compass_transformer_nodelet APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-as-needed")
  else()
    #catkin_lint: ignore_once cmake_old
    target_link_options(test_compass_transformer_nodelet PUBLIC "LINKER:--no-as-needed")
  endif()
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
