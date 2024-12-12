# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(magnetometer_compass)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  angles
  compass_conversions
  compass_msgs
  cras_cpp_common
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS compass_msgs cras_cpp_common sensor_msgs tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_nodelet nodelets/${PROJECT_NAME}_nodelet.cpp)
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME} ${catkin_LIBRARIES})
cras_node_from_nodelet(${PROJECT_NAME}_nodelet ${PROJECT_NAME}::MagnetometerCompassNodelet OUTPUT_NAME ${PROJECT_NAME})

add_library(visualize_azimuth_nodelet nodelets/visualize_azimuth_nodelet.cpp)
add_dependencies(visualize_azimuth_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_azimuth_nodelet ${catkin_LIBRARIES})
cras_node_from_nodelet(visualize_azimuth_nodelet ${PROJECT_NAME}::VisualizeAzimuthNodelet OUTPUT_NAME visualize_azimuth)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet visualize_azimuth_nodelet
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
  find_package(roslaunch REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_FILES include/*.h src/*.cpp nodelets/*.cpp test/*.cpp)
  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_FILES})

  roslint_add_test()

  roslaunch_add_file_check(launch/compass.launch)

  catkin_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

  add_rostest_gtest(test_${PROJECT_NAME}_nodelet test/test_${PROJECT_NAME}_nodelet.test test/test_${PROJECT_NAME}_nodelet.cpp)
  target_link_libraries(test_${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME}_nodelet)
  roslaunch_add_file_check(test/test_${PROJECT_NAME}_nodelet.test USE_TEST_DEPENDENCIES)
  if(CMAKE_VERSION VERSION_LESS "3.13.0")
    set_property(TARGET test_${PROJECT_NAME}_nodelet APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-as-needed")
  else()
    #catkin_lint: ignore_once cmake_old
    target_link_options(test_${PROJECT_NAME}_nodelet PUBLIC "LINKER:--no-as-needed")
  endif()

  add_rostest_gtest(test_visualize_azimuth_nodelet test/test_visualize_azimuth_nodelet.test test/test_visualize_azimuth_nodelet.cpp)
  target_link_libraries(test_visualize_azimuth_nodelet ${catkin_LIBRARIES})
  roslaunch_add_file_check(test/test_visualize_azimuth_nodelet.test USE_TEST_DEPENDENCIES)
endif()
