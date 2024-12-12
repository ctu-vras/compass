# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(magnetometer_pipeline)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  geometry_msgs
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2_eigen
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES magnetometer_bias_remover magnetometer_bias_remover_filter
  CATKIN_DEPENDS cras_cpp_common message_filters roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(magnetometer_bias_remover src/bias_remover.cpp)
add_dependencies(magnetometer_bias_remover ${catkin_EXPORTED_TARGETS})
target_link_libraries(magnetometer_bias_remover ${catkin_LIBRARIES} Eigen3::Eigen)

add_library(magnetometer_bias_remover_filter src/message_filter.cpp)
add_dependencies(magnetometer_bias_remover_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(magnetometer_bias_remover_filter ${catkin_LIBRARIES} magnetometer_bias_remover)

add_library(magnetometer_bias_remover_nodelet nodelets/magnetometer_bias_remover_nodelet.cpp)
add_dependencies(magnetometer_bias_remover_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(magnetometer_bias_remover_nodelet ${catkin_LIBRARIES} magnetometer_bias_remover_filter)
cras_node_from_nodelet(magnetometer_bias_remover_nodelet
  ${PROJECT_NAME}::MagnetometerBiasRemoverNodelet OUTPUT_NAME magnetometer_bias_remover)

install(TARGETS magnetometer_bias_remover magnetometer_bias_remover_filter magnetometer_bias_remover_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/magnetometer_bias_observer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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

  # Run roslint on Python sources
  file(GLOB_RECURSE PYTHON_FILES nodes/magnetometer_bias_observer)
  roslint_python("${PYTHON_FILES}")

  roslint_add_test()

  roslaunch_add_file_check(launch/magnetometer_proc.launch)

  catkin_add_gtest(test_magnetometer_bias_remover test/test_bias_remover.cpp)
  target_link_libraries(test_magnetometer_bias_remover magnetometer_bias_remover)

  catkin_add_gtest(test_magnetometer_bias_remover_filter test/test_message_filter.cpp)
  target_link_libraries(test_magnetometer_bias_remover_filter magnetometer_bias_remover_filter)

  add_rostest_gtest(test_magnetometer_bias_remover_nodelet
    test/test_bias_remover_nodelet.test test/test_bias_remover_nodelet.cpp)
  target_link_libraries(test_magnetometer_bias_remover_nodelet ${catkin_LIBRARIES})
  roslaunch_add_file_check(test/test_bias_remover_nodelet.test USE_TEST_DEPENDENCIES)
endif()
