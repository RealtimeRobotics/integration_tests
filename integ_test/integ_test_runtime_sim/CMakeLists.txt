cmake_minimum_required(VERSION 3.0.2)
project(integ_test_runtime_sim)

set(CMAKE_CXX_STANDARD 14)
# dpkg assumes multiarch and cmake supports it:
# https://wiki.debian.org/Multiarch/Implementation#Dynamic_debian.2F.2A_files
# https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

find_package(catkin REQUIRED COMPONENTS
  rtr_appliance
  rtr_cmake_defs
  rtr_perc_rapidsense_ros
  rtr_ros_utils
  rtr_test_cmake_defs
  rtr_test_harness
  rtr_utils
)

# https://github.com/ros/cmake_modules
find_package(cmake_modules REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 REQUIRED COMPONENTS Core)

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
  CATKIN_DEPENDS
    rtr_appliance
    rtr_perc_rapidsense_ros
    rtr_ros_utils
    rtr_test_harness
    rtr_utils
  DEPENDS
    Boost
    Qt5
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
  ${Boost_INCLUDE_DIRS}
)

#############
## Install ##
#############


#############
## Testing ##
#############

rs_add_rostest(RuntimeSimTest)
