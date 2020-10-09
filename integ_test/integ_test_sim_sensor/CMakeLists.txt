cmake_minimum_required(VERSION 3.0.2)
project(integ_test_sim_sensor)

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
  rtr_geometry
  rtr_perc_calibration
  rtr_perc_spatial
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

#catkin_python_setup()

#generate_messages(
#  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs
#)

###################################
## Catkin Specific Configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    rtr_geometry
    rtr_perc_calibration
    rtr_perc_spatial
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

function(rs_add_gtest TestName)
  catkin_add_gtest(${TestName}
    tests/${TestName}.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests
  )
  target_compile_options(${TestName} PRIVATE ${GTEST_COMPILE_OPTIONS})
  target_link_libraries(${TestName} ${catkin_LIBRARIES} ${ARGN})
  set_property(TARGET ${TestName} PROPERTY POSITION_INDEPENDENT_CODE ON)
endfunction()

rs_add_gtest(SimSensorTest)
rs_add_gtest(MeshImageGeneratorTest)
