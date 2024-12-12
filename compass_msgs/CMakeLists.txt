# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(compass_msgs)

find_package(catkin REQUIRED COMPONENTS cras_cpp_common message_generation std_msgs)

add_message_files(
  FILES
  Azimuth.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_string_utils
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${${PROJECT_NAME}_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_string_utils src/string_utils.cpp)
add_dependencies(${PROJECT_NAME}_string_utils ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_string_utils ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_string_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

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

  catkin_add_gtest(test_string_utils test/test_string_utils.cpp)
  target_link_libraries(test_string_utils ${PROJECT_NAME}_string_utils)
endif()