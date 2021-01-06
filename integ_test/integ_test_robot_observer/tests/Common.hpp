#ifndef INTEG_TEST_INTEG_TEST_ROBOT_OBSERVER_TESTS_COMMON_HPP_
#define INTEG_TEST_INTEG_TEST_ROBOT_OBSERVER_TESTS_COMMON_HPP_

#include <sensor_msgs/JointState.h>

#include <rtr_control/RobotManagerInterface.hpp>
#include <rtr_control_ros/RobotStatus.h>
#include <rtr_control_ros/RosController.hpp>
#include <rtr_perc_rapidsense_ros/RosRobotConnection.hpp>
#include <rtr_perc_spatial/PerceptionTestUtils.hpp>
#include <rtr_utils/Environment.hpp>

sensor_msgs::JointState CreateJointStateMessage(const rtr::JointConfiguration& config) {
  sensor_msgs::JointState joint_msg;
  joint_msg.name = std::vector<std::string>(config.Size(), "");
  joint_msg.header.stamp = ros::Time::now();
  for (const auto& js : config.GetData()) {
    joint_msg.position.push_back(js);
  }
  joint_msg.velocity = std::vector<double>(config.Size(), 0.0);
  joint_msg.effort = std::vector<double>(config.Size(), 0.0);
  return joint_msg;
}

rtr_control_ros::RobotStatus CreateRobotStatusMessage(const rtr::RobotManagerInterface::State state,
                                                      const std::string& state_str) {
  rtr_control_ros::RobotStatus status_msg;
  status_msg.state = static_cast<int>(state);
  status_msg.state_str = state_str;
  return status_msg;
}

rtr::RosController::Ptr CreateRosController(const std::string& robot_type,
                                            const std::string& prefix) {
  rtr::RosController::Config config;
  rtr::perception::testutils::GetRobotPathInfo(robot_type, config.urdf_path, config.base_link,
                                               config.end_effector_link);
  EXPECT_TRUE(rtr::GetDefaultParamJSONString(robot_type, config.robot_params));
  config.robot_type = robot_type;
  config.ros_package_path = rtr::utils::ROS_PACKAGE_PATH;
  config.has_gripper = false;
  config.connection_type = rtr::RobotManagerInterface::ConnectionType::kInternalSim;
  config.stopping_speed_factor = 1.f;
  config.velocity_factors = rtr::Vec(6, 1.f);
  config.acceleration_factors = rtr::Vec(6, 1.f);
  config.jerk_factors = rtr::Vec(6, 1.f);
  config.topic_prefix = prefix;
  config.do_setup_rtr = false;
  return rtr::RosController::Create(config);
}

#endif  // INTEG_TEST_INTEG_TEST_ROBOT_OBSERVER_TESTS_COMMON_HPP
