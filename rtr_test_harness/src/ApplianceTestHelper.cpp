#include "rtr_test_harness/ApplianceTestHelper.hpp"

#include <fstream>

#include <nlohmann-json/json.hpp>

#include <rtr_msgs/ClearApplianceFaultsAction.h>
#include <rtr_msgs/DeconGroupAction.h>
#include <rtr_msgs/DeleteProjectAction.h>
#include <rtr_msgs/EnterCalibrationMode.h>
#include <rtr_msgs/ExitCalibrationMode.h>
#include <rtr_msgs/GetAllResultCodes.h>
#include <rtr_msgs/GetApplianceErrors.h>
#include <rtr_msgs/GetApplianceState.h>
#include <rtr_msgs/GetEULAAccepted.h>
#include <rtr_msgs/GetGroupInfo.h>
#include <rtr_msgs/GetHubConfig.h>
#include <rtr_msgs/GetProjectList.h>
#include <rtr_msgs/GetProjectROSInfo.h>
#include <rtr_msgs/GetVisionEnabled.h>
#include <rtr_msgs/GroupProjectAction.h>
#include <rtr_msgs/InstallProjectAction.h>
#include <rtr_msgs/LoadedProjectInfo.h>
#include <rtr_msgs/SetEULAAcceptedAction.h>
#include <rtr_msgs/SetVisionEnabled.h>
#include <rtr_msgs/TeleportRobot.h>
#include <rtr_msgs/UpdateGroupAction.h>
#include <rtr_msgs/UpdateProjectAction.h>
#include <rtr_msgs/Version.h>
#include <rtr_utils/Logging.hpp>

namespace rtr {

ApplianceTestHelper::ApplianceTestHelper(ros::NodeHandle& nh)
    : ApplianceCommander("127.0.0.1"), nh_(nh) {
  rtr_msgs::SetEULAAcceptedGoal goal;
  goal.signature = "unittest";
  CallRosAction<rtr_msgs::SetEULAAcceptedAction>("/SetEULAAccepted", goal);
}

bool ApplianceTestHelper::InstallProject(const std::string& project_zip) {
  rtr_msgs::InstallProjectGoal goal;
  goal.zip_file_path = project_zip;
  if (!CallRosAction<rtr_msgs::InstallProjectAction>("/InstallNewProject", goal)) {
    return false;
  }
  return true;
}

bool ApplianceTestHelper::GetInstalledProjects(std::vector<std::string>& projects) {
  rtr_msgs::GetProjectList msg_prjs;
  if (!CallRosService<rtr_msgs::GetProjectList>(nh_, msg_prjs, "/GetProjectList")) {
    return false;
  }
  projects = std::vector<std::string>(msg_prjs.response.projects);

  return true;
}

bool ApplianceTestHelper::AddAllProjectsToDeconGroup(const std::string& dc_group_name) {
  // Create the decon group
  rtr_msgs::DeconGroupGoal grp_goal;
  grp_goal.group_name = dc_group_name;
  if (!CallRosAction<rtr_msgs::DeconGroupAction>("/CreateGroup", grp_goal)) {
    return false;
  }

  // Get all list of all projects that are installed
  std::vector<std::string> projects;
  if (!this->GetInstalledProjects(projects)) {
    return false;
  }

  // Add all installed projects to the decon group
  for (const auto& P : projects) {
    rtr_msgs::GroupProjectGoal msg_add_prj;
    msg_add_prj.group_name = dc_group_name;
    msg_add_prj.project_name = P;
    if (!CallRosAction<rtr_msgs::GroupProjectAction>("/AddProjectToGroup", msg_add_prj)) {
      return false;
    }
  }

  // Set voxel region verified
  rtr_msgs::UpdateGroupGoal update_grp;
  update_grp.group_name = dc_group_name;
  update_grp.json_data = "{\"voxel_region_verified\":1}";
  if (!CallRosActionWithResultCheck<rtr_msgs::UpdateGroupAction>("/UpdateGroup", update_grp)) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::SetVisionEnabled(const std::string& dc_group_name, bool is_enabled) {
  rtr_msgs::SetVisionEnabled srv;
  srv.request.group_name = dc_group_name;
  srv.request.enabled = is_enabled;
  if (!CallRosService<rtr_msgs::SetVisionEnabled>(nh_, srv, "/SetVisionEnabled")) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::LoadGroup(const std::string& dc_group_name) {
  rtr_msgs::DeconGroupGoal goal;
  goal.group_name = dc_group_name;
  if (!CallRosAction<rtr_msgs::DeconGroupAction>("/LoadGroup", goal)) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::UnloadGroup(const std::string& dc_group_name) {
  rtr_msgs::DeconGroupGoal goal;
  goal.group_name = dc_group_name;
  if (!CallRosAction<rtr_msgs::DeconGroupAction>("/UnloadGroup", goal)) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::SetProjectRobotParam(const std::string& prj_name,
                                               const std::string& param_json_path) {
  // take the robot_param json file and create a json obj
  std::ifstream ifs(param_json_path);
  nlohmann::json j = nlohmann::json::parse(ifs);

  // Update Project Info
  rtr_msgs::UpdateProjectGoal goal;
  goal.project_name = prj_name;
  goal.json_data = j.dump();
  if (!CallRosActionWithResultCheck<rtr_msgs::UpdateProjectAction>("/UpdateProject", goal)) {
    RTR_WARN("Could not update project {} with params: {}", prj_name, j.dump());
    return false;
  }

  return true;
}

bool ApplianceTestHelper::GetLoadedDeconGroup(rtr_msgs::DeconGroupInfo& loaded_group) {
  rtr_msgs::GetGroupInfo srv;
  if (!CallRosService<rtr_msgs::GetGroupInfo>(nh_, srv, "/GetDeconGroupInfo")) {
    return false;
  }

  for (const auto& group : srv.response.groups) {
    if (group.loaded) {
      loaded_group = group;
      return true;
    }
  }

  return false;
}

bool ApplianceTestHelper::ClearApplianceDatabase() {
  rtr_msgs::GetGroupInfo info_srv;
  if (!CallRosService<rtr_msgs::GetGroupInfo>(nh_, info_srv, "/GetDeconGroupInfo")) {
    return false;
  }

  for (const auto& group : info_srv.response.groups) {
    for (const auto& project : group.projects) {
      rtr_msgs::GroupProjectGoal remove_goal;
      remove_goal.group_name = group.GroupName;
      remove_goal.project_name = project;
      if (!CallRosActionWithResultCheck<rtr_msgs::GroupProjectAction>("/RemoveProjectFromGroup", remove_goal)) {
        return false;
      }
      rtr_msgs::DeleteProjectGoal delete_goal;
      delete_goal.project_name = project;
      if (!CallRosActionWithResultCheck<rtr_msgs::DeleteProjectAction>("/DeleteProject", delete_goal)) {
        return false;
      }
    }
    rtr_msgs::DeconGroupGoal delete_goal;
    delete_goal.group_name = group.GroupName;
    if (!CallRosActionWithResultCheck<rtr_msgs::DeconGroupAction>("/DeleteGroup", delete_goal)) {
      return false;
    }
  }
  return true;
}

bool ApplianceTestHelper::TeleportToHub(const std::string& robot_name,
                                        const std::string& hub_name) {
  ros::ServiceClient hub_config_service =
      nh_.serviceClient<rtr_msgs::GetHubConfig>("/GetHubConfig");
  rtr_msgs::GetHubConfig hub_config_srv;
  hub_config_srv.request.project_name = robot_name;
  hub_config_srv.request.hub_name = hub_name;
  rtr::JointConfiguration joint_config;

  if (!hub_config_service.call(hub_config_srv)) {
    RTR_ERROR("Failed to call /GetHubConfig service");
    return false;
  }
  joint_config = rtr::JointConfiguration(hub_config_srv.response.joint_config);

  // Teleport to Hub
  const std::string teleport_topic = fmt::format("/{}/teleport_robot", robot_name);
  ros::ServiceClient teleport_service = nh_.serviceClient<rtr_msgs::TeleportRobot>(teleport_topic);
  rtr_msgs::TeleportRobot teleport_srv;
  teleport_srv.request.joint_config = joint_config.GetData();

  return teleport_service.call(teleport_srv) && teleport_srv.response.is_success;
}

}  // namespace rtr
