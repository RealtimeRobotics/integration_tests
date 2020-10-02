#ifndef RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETEST_HPP_
#define RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETEST_HPP_

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <ros/node_handle.h>

#include <rtr_appliance/ApplianceCommander.hpp>
#include <rtr_appliance/ApplianceExternalInterfaceCodes.hpp>
#include <rtr_control_ros/FollowJointPathAction.h>
#include <rtr_msgs/DeconGroupInfo.h>
#include <rtr_perc_rapidsense_ros/GetSchemaMessage.h>
#include <rtr_perc_rapidsense_ros/RapidSenseDefs.hpp>
#include <rtr_perc_rapidsense_ros/SchemaMessageHelpers.hpp>

#include "rtr_test_harness/RapidSenseTestConfigs.hpp"

namespace rtr {
namespace perception {

class RapidSenseFrontEndProxy;

class RapidSenseTest {
 public:
  RapidSenseTest();

  bool Init(const std::string& dir);

  bool Run(const bool use_live_data, const bool record_data);

  std::string GetParameterFilename() const;

  RapidSenseTestResult::Ptr GetResult() const;

 private:
  bool CheckRapidSenseServerState_();

  bool MoveToHubs_(std::vector<RapidSenseTestHubConfig>& hub_sequence);

  ros::NodeHandle nh_;

  std::string test_dir_;
  RapidSenseTestConfig config_;
  rtr::ApplianceCommander app_commander_;
  BenchmarkManager::Ptr benchmark_manager_;

  SpatialPerceptionProjectSchema rapidsense_config_;
  std::shared_ptr<RapidSenseFrontEndProxy> proxy_;
  VoxelRegionDescription region_desc_;
  std::vector<RobotObserver::Ptr> observers_;
  RapidSenseTestResult::Ptr test_result_;
};

// TODO: move these utilities into rtr_perc_rapidsense_ros
std::string GetActiveDeconGroup();
bool GetLoadedDeconGroup(rtr_msgs::DeconGroupInfo& loaded_group);
bool LoadRapidPlanProjects(std::vector<RobotObserver::Ptr>& observers,
                           VoxelRegionDescription& region_desc);
bool SetIgnoreVisionEnabledOnServer(const bool enable);

}  // namespace perception
}  // namespace rtr

#endif  // RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETEST_HPP_
