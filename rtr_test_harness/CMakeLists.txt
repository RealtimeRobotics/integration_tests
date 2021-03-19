cmake_minimum_required(VERSION 3.0.2)
project(rtr_test_harness)

# dpkg assumes multiarch and cmake supports it:
# https://wiki.debian.org/Multiarch/Implementation#Dynamic_debian.2F.2A_files
# https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 14)

# see RAPID-1503:
add_definitions(-DTBB_DEPRECATED_FLOW_ENQUEUE)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  eigen_conversions
  image_transport
  message_generation
  matplotlib-cpp
  roscpp
  rtr_cmake_defs
  rtr_spatial_perception
  rtr_control_ros
  rtr_msgs
  rtr_appliance
  rtr_app_layer
  rtr_perc_rapidsense
  rtr_perc_rapidsense_ros
  rtr_perc_sensors_ros
  rtr_perc_sensors
  rtr_perc_spatial
  rtr_roadmapgen
  sensor_msgs
  std_msgs
  tbb2
  tf
  tf_conversions
  visualization_msgs
)

# https://github.com/ros/cmake_modules
find_package(cmake_modules REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Qt5 REQUIRED COMPONENTS Core Gui)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS inc
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    cv_bridge
    eigen_conversions
    image_transport
    message_runtime
    matplotlib-cpp
    roscpp
    rtr_spatial_perception
    rtr_control_ros
    rtr_msgs
    rtr_appliance
    rtr_app_layer
    rtr_perc_rapidsense
    rtr_perc_rapidsense_ros
    rtr_perc_sensors_ros
    rtr_perc_sensors
    rtr_perc_spatial
    rtr_roadmapgen
    sensor_msgs
    std_msgs
    tbb2
    tf
    tf_conversions
    visualization_msgs
  DEPENDS
    Boost
    Threads
    Qt5
)

###########
## Build ##
###########
rtr_get_perc_compile_options(RTR_COMPILE_OPTIONS GTEST_COMPILE_OPTIONS)

if (FALSE)
  # underscores in an executable basename look weird...
  string(REPLACE "_" "-" EXE_NAME "${PROJECT_NAME}")
else()
  # ...but some people like that.
  set(EXE_NAME "${PROJECT_NAME}")
endif()

# because catkin should do this, but doesn't
set(PROJECT_VERSION ${${PROJECT_NAME}_VERSION})

# autoconf-style config header; because config- and build-time cmake variables
# are nice to have as preprocessor definitions
configure_file(config.h.in config.h @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR} inc)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${boost_INCLUDE_DIR}   ${Qt5Core_INCLUDE_DIRS}
${Qt5Gui_INCLUDE_DIRS})

set(INC
  inc/${PROJECT_NAME}/ApplianceTestHelper.hpp
  inc/${PROJECT_NAME}/RapidSenseTestHarnessServer.hpp
  inc/${PROJECT_NAME}/RapidSenseTestConfigs.hpp
  inc/${PROJECT_NAME}/RapidSenseTest.hpp
  inc/${PROJECT_NAME}/RapidSenseTestHelper.hpp
)

set(SRC
  src/ApplianceTestHelper.cpp
  src/RapidSenseTestHarnessServer.cpp
  src/RapidSenseTest.cpp
  src/RapidSenseTestHelper.cpp
  src/RapidSenseTestConfigs.cpp
)

#set(RES resources/resources.qrc )
#qt5_add_resources(RES_RCC ${RES})

add_executable(run_rapidsense_tests src/run_rapidsense_tests.cpp ${INC} ${SRC})
target_link_libraries(run_rapidsense_tests ${catkin_LIBRARIES} Threads::Threads)
target_compile_options(run_rapidsense_tests PRIVATE ${RTR_COMPILE_OPTIONS})

add_executable(rapidsense_benchmarker src/rapidsense_benchmarker.cpp ${INC} ${SRC})
target_link_libraries(rapidsense_benchmarker ${catkin_LIBRARIES} Threads::Threads)
target_compile_options(rapidsense_benchmarker PRIVATE ${RTR_COMPILE_OPTIONS})

add_library(${PROJECT_NAME} ${INC} ${SRC})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME} rtr_test_harness_ros_generate_messages_cpp)
string(REGEX MATCH "^[0-9]+" ${PROJECT_NAME}_VERSION_MAJOR ${${PROJECT_NAME}_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION} SOVERSION ${${PROJECT_NAME}_VERSION_MAJOR})
target_link_libraries(${PROJECT_NAME}
  LINK_PUBLIC
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  Threads::Threads
)
target_compile_options(${PROJECT_NAME} PRIVATE ${RTR_COMPILE_OPTIONS})

#############
## Install ##
#############
# http://docs.ros.org/kinetic/api/catkin/html/howto/format2/building_libraries.html#installing
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  FILES ${INC}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# > type="nodetype"
# >     Node type. There must be a corresponding executable with the same name.
#
# -- https://wiki.ros.org/roslaunch/XML/node
#
# Yep, it has to be an executable, but that executable must be installed to the
# CATKIN_PACKAGE_BIN_DESTINATION. `roslaunch` only finds executables installed
# to the CATKIN_PACKAGE_BIN_DESTINATION and definitely NOT the
# CATKIN_GLOBAL_BIN_DESTINATION. Because of course it does.
#install(TARGETS ${EXE_NAME}  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
