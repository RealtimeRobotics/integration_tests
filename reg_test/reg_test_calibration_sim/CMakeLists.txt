cmake_minimum_required(VERSION 3.0.2)
project(reg_test_calibration_sim)

set(CMAKE_CXX_STANDARD 14)
# dpkg assumes multiarch and cmake supports it:
# https://wiki.debian.org/Multiarch/Implementation#Dynamic_debian.2F.2A_files
# https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  rtr_core
  rtr_geometry
  rtr_utils
  rtr_planning
  rtr_cmake_defs
  rtr_control
  rtr_robots
  rtr_api
  rtr_appliance_app
  rtr_test_harness
  rtr_appliance
  rtr_perc_rapidsense
  rtr_perc_rapidsense_ros
  rtr_app_layer
  rtr_control_ros
  rtr_msgs
  rtr_ros_utils
)

# https://github.com/ros/cmake_modules
find_package(cmake_modules REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)

# https://stackoverflow.com/questions/17633513/cmake-cannot-find-openmp/28792611#28792611
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

################################################
## Declare ROS messages, services and actions ##
################################################

#catkin_python_setup()

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs
)

###################################
## Catkin Specific Configuration ##
###################################

catkin_package(
  INCLUDE_DIRS inc
  CATKIN_DEPENDS
    rospy
    rtr_core
    rtr_geometry
    rtr_utils
    rtr_planning
    rtr_control
    rtr_robots
    rtr_api
    rtr_appliance
    rtr_perc_rapidsense
    rtr_test_harness
    rtr_appliance_app
    rtr_perc_rapidsense_ros
    rtr_app_layer
    rtr_control_ros
    rtr_msgs
    rtr_ros_utils
  DEPENDS
    yaml-cpp
    Boost
    OpenCV
    Qt5
    OpenGL
)

###########
## Build ##
###########
rtr_get_perc_compile_options(RTR_COMPILE_OPTIONS GTEST_COMPILE_OPTIONS)

# because catkin should do this, but doesn't
set(PROJECT_VERSION ${${PROJECT_NAME}_VERSION})

# autoconf-style config header; because config- and build-time cmake variables
# are nice to have as preprocessor definitions
configure_file(config.h.in config.h @ONLY)

include_directories(inc ${CMAKE_CURRENT_BINARY_DIR})

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#############
## Install ##
#############


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(CalibrationSimTest test/cal_sim.test test/CalibrationSimTest.cpp)
  target_link_libraries(CalibrationSimTest ${catkin_LIBRARIES})
  set_property(TARGET CalibrationSimTest PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_compile_options(CalibrationSimTest PRIVATE ${GTEST_COMPILE_OPTIONS})
endif()


