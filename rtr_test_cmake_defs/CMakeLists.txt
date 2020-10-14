cmake_minimum_required(VERSION 3.0.2)
project(rtr_test_cmake_defs)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS cmake_functions-extras.cmake)

install(DIRECTORY cmake/functions
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)
