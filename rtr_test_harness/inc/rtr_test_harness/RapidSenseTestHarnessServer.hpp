#ifndef RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETEST_HPP_
#define RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETEST_HPP_

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ros/node_handle.h>

#include <rtr_appliance/Appliance.hpp>
#include <rtr_appliance/ApplianceExternalInterfaceCodes.hpp>
#include <rtr_control_ros/FollowJointPathAction.h>
#include <rtr_msgs/DeconGroupInfo.h>
#include <rtr_perc_rapidsense_ros/GetSchemaMessage.h>
#include <rtr_perc_rapidsense_ros/RapidSenseDefs.hpp>
#include <rtr_perc_rapidsense_ros/SchemaMessageHelpers.hpp>

#include "rtr_perc_rapidsense_ros/RapidSenseServer.hpp"
#include "rtr_perc_rapidsense_ros/SensorSimulator.hpp"
#include "rtr_test_harness/RapidSenseTestConfigs.hpp"

namespace rtr {
namespace perception {

// Brings up Appliance and RapidSenseServer nodes.
class RapidSenseTestHarnessServer {
 public:
  RapidSenseTestHarnessServer();

  // @brief Setup applinace & rapidsense server
  bool SetUp(const std::string& app_dir);

  // @brief Setup applinace & rapidsense server with simulated sensors
  bool SetUpSim(const std::string& app_dir);

  // @brief Start appliance server
  bool StartServer(const std::string& app_dir);

  // @brief Start simulated sensors
  bool StartSensorSimulator();

  // @brief Check state of appliance & rapidsense
  bool CheckServerHealth();

  // @brief Cleanup appliance & rapidsense server
  void Teardown();

 private:
  ros::NodeHandle nh_;
  std::string appliance_dir_, rapidsense_dir_;
  // rtr::Appliance::Ptr appliance_;
  // rtr::perception::RapidSenseServer::Ptr rapidsense_;
  SensorSimulator::Ptr simulator_;
  ros::ServiceServer restart_sim_;
  ros::AsyncSpinner spinner_;
};

}  // namespace perception
}  // namespace rtr

#endif  // RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETEST_HPP_
