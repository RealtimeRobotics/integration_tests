#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/service.h>

#include "rtr_appliance/ApplianceExternalInterfaceCodes.hpp"
#include <rtr_msgs/DeconGroupInfo.h>
#include <rtr_utils/Logging.hpp>

const std::string kDefaultEulaSignature = "unittest";
const std::string kDefaultDeconGroupName = "integration_test_group";

namespace rtr {

//! @brief call ros service and check response
template <typename ActionType, typename GoalType>
bool CallRosAction(const std::string& action_name, const GoalType& goal) {
  actionlib::SimpleActionClient<ActionType> client(action_name, true);
  if (!client.waitForServer(ros::Duration(30.0))) {
    RTR_ERROR("Failed to call {}. Action server does not exist", action_name);
    return false;
  }

  actionlib::SimpleClientGoalState state = client.sendGoalAndWait(goal);
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED && state != actionlib::SimpleClientGoalState::ABORTED) {
    RTR_ERROR("Failed to call {}", action_name);
    return false;
  }
  RTR_DEBUG("Successfully called {}", action_name);
  return true;
}

//! @brief call ros service and check response
template <typename ActionType, typename GoalType, typename ResultType>
bool CallRosAction(const std::string& action_name, const GoalType& goal, ResultType& result) {
  actionlib::SimpleActionClient<ActionType> client(action_name, true);
  if (!client.waitForServer(ros::Duration(30.0))) {
    RTR_ERROR("Failed to call {}. Action server does not exist", action_name);
    return false;
  }

  actionlib::SimpleClientGoalState state = client.sendGoalAndWait(goal);
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED && state != actionlib::SimpleClientGoalState::ABORTED) {
    RTR_ERROR("Failed to call {}", action_name);
    return false;
  }
  result = *client.getResult();
  RTR_DEBUG("Successfully called {}", action_name);
  return true;
}

//! @brief call ros service and check response
template <typename ActionType, typename GoalType>
bool CallRosActionWithResultCheck(const std::string& action_name, const GoalType& goal) {
  actionlib::SimpleActionClient<ActionType> client(action_name, true);
  if (!client.waitForServer(ros::Duration(30.0))) {
    RTR_ERROR("Failed to call {}. Action server does not exist", action_name);
    return false;
  }

  actionlib::SimpleClientGoalState state = client.sendGoalAndWait(goal);
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED && state != actionlib::SimpleClientGoalState::ABORTED) {
    RTR_ERROR("Failed to call {}", action_name);
    return true;
  }
  RTR_DEBUG("Successfully called {}", action_name);
  return client.getResult()->result_code == Convert<int>(ExtCode::SUCCESS);
}

//! @brief call ros service and check response
template <typename MessageType>
bool CallRosService(ros::NodeHandle& nh, MessageType& srv, const std::string& serv) {
  // TODO: We need a timeout on watiForService, but uncertain of how this
  // can be done without hardcoding
  if (!ros::service::waitForService(serv)) {
    RTR_ERROR("No service {}", serv);
    return false;
  }

  RTR_DEBUG("Calling service {} with {}", serv, srv.request);
  ros::ServiceClient client = nh.serviceClient<MessageType>(serv);
  if (client.call(srv)) {
    RTR_DEBUG("Success with {}", srv.response);
    return true;
  } else {
    RTR_ERROR("Failed to call {}", serv);
  }
  return false;
}

// @brief Class to help setup the appliance, as would a rtr customer
class WebappCommander {
 public:
  WebappCommander(ros::NodeHandle& nh);

  // @brief Accept the rtr appliance web eula
  bool AcceptEula();

  // @brief Install the project zip to appliance
  bool InstallProject(const std::string& zip_path);

  // @brief Get a list of projects currently installed onto the appliance
  bool GetInstalledProjects(std::vector<std::string>& projects);

  // @brief Create a decon group, and add all installed projects to that group
  bool AddAllProjectsToDeconGroup(const std::string& dc_group_name);

  // @brief Set appliance deconfliction group to run with rapidsense enabled
  bool SetVisionEnabled(const std::string& dc_group_name, bool is_enabled);

  // @brief Load appliance with deconfliction group as per user-guide
  bool LoadGroup(const std::string& dc_group_name);

  // @brief Unload deconfliction group from the appliance
  bool UnloadGroup(const std::string& dc_group_name);

  /**
   * @brief take a json file and set that as the robot_params for the
   * appliance project via ros services
   */
  bool SetProjectRobotParam(const std::string& prj_name, const std::string& param_json_path);

  // @brief Get info about the deconfliction group loaded on the appliance
  bool GetLoadedDeconGroup(rtr_msgs::DeconGroupInfo& loaded_group);

  // @brief Clear entire appliance database
  bool ClearApplianceDatabase();

  // @brief Teleport to hub
  bool TeleportToHub(const std::string& robot_name, const std::string& hub_name);

 protected:
  ros::NodeHandle nh_;
};

}  // namespace rtr
