cmake_minimum_required(VERSION 3.0.2)
project(integ_test_robot_observer)

set(CMAKE_CXX_STANDARD 14)
# dpkg assumes multiarch and cmake supports it:
# https://wiki.debian.org/Multiarch/Implementation#Dynamic_debian.2F.2A_files
# https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

find_package(catkin REQUIRED COMPONENTS
  rtr_cmake_defs
  rtr_control
  rtr_control_ros
  rtr_perc_spatial
  rtr_perc_sensors_ros
  rtr_perc_rapidsense_ros
  rtr_test_cmake_defs
  rtr_utils
)

# https://github.com/ros/cmake_modules
find_package(cmake_modules REQUIRED)

find_package(OpenGL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## Catkin Specific Configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    rtr_control
    rtr_control_ros
    rtr_perc_spatial
    rtr_perc_sensors_ros
    rtr_perc_rapidsense_ros
    rtr_utils
  DEPENDS
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

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############


#############
## Testing ##
#############

rs_add_gtest(RobotProxyTest)
rs_add_rostest(RosRobotObserverTest)
rs_add_rostest(RobotObserverManagerTest)
