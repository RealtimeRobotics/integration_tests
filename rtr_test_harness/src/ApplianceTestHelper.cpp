#include "rtr_test_harness/ApplianceTestHelper.hpp"

#include <fstream>

#include <nlohmann-json/json.hpp>

#include <rtr_msgs/ClearApplianceFaults.h>
#include <rtr_msgs/DeconGroup.h>
#include <rtr_msgs/DeleteProject.h>
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
#include <rtr_msgs/GroupProject.h>
#include <rtr_msgs/InstallProject.h>
#include <rtr_msgs/LoadedProjectInfo.h>
#include <rtr_msgs/SetEULAAccepted.h>
#include <rtr_msgs/SetVisionEnabled.h>
#include <rtr_msgs/TeleportRobot.h>
#include <rtr_msgs/UpdateGroup.h>
#include <rtr_msgs/UpdateProject.h>
#include <rtr_msgs/Version.h>

namespace rtr {

ApplianceTestHelper::ApplianceTestHelper(ros::NodeHandle& nh)
    : ApplianceCommander("127.0.0.1"), nh_(nh) {
  rtr_msgs::SetEULAAccepted srv;
  srv.request.signature = "unittest";
  CallRosService<rtr_msgs::SetEULAAccepted>(nh_, srv, "/SetEULAAccepted");
}

bool ApplianceTestHelper::InstallProject(const std::string &project_zip) {
  rtr_msgs::InstallProject srv;
  srv.request.zip_file_path = project_zip;
  if (!CallRosService<rtr_msgs::InstallProject>(nh_, srv, "/InstallProject")) {
    return false;
  }
  return true;
}

bool ApplianceTestHelper::GetInstalledProjects(
    std::vector<std::string> &projects) {
  rtr_msgs::GetProjectList msg_prjs;
  if (!CallRosService<rtr_msgs::GetProjectList>(nh_, msg_prjs,
                                                "/GetProjectList")) {
    return false;
  }
  projects = std::vector<std::string>(msg_prjs.response.projects);

  return true;
}

bool ApplianceTestHelper::AddAllProjectsToDeconGroup(
    const std::string &dc_group_name) {
  // Create the decon group
  rtr_msgs::DeconGroup msg_grp;
  msg_grp.request.group_name = dc_group_name;
  if (!CallRosService<rtr_msgs::DeconGroup>(nh_, msg_grp, "/CreateGroup")) {
    return false;
  }

  // Get all list of all projects that are installed
  std::vector<std::string> projects;
  if (!this->GetInstalledProjects(projects)) {
    return false;
  }

  // Add all installed projects to the decon group
  for (const auto &P : projects) {
    rtr_msgs::GroupProject msg_add_prj;
    msg_add_prj.request.group_name = dc_group_name;
    msg_add_prj.request.project_name = P;
    if (!CallRosService<rtr_msgs::GroupProject>(nh_, msg_add_prj,
                                                "/AddProjectToGroup")) {
      return false;
    }
  }

  // Set voxel region verified
  rtr_msgs::UpdateGroup update_grp;
  update_grp.request.group_name = dc_group_name;
  update_grp.request.json_data = "{\"voxel_region_verified\":1}";
  if (!CallRosService<rtr_msgs::UpdateGroup>(nh_, update_grp, "/UpdateGroup")
      && update_grp.response.result_code != Convert<int>(ExtCode::SUCCESS)) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::SetVisionEnabled(const std::string &dc_group_name,
                                           bool is_enabled) {
  rtr_msgs::SetVisionEnabled srv;
  srv.request.group_name = dc_group_name;
  srv.request.enabled = is_enabled;
  if (!CallRosService<rtr_msgs::SetVisionEnabled>(nh_, srv,
                                                  "/SetVisionEnabled")) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::LoadGroup(const std::string &dc_group_name) {
  rtr_msgs::DeconGroup srv;
  srv.request.group_name = dc_group_name;
  if (!CallRosService<rtr_msgs::DeconGroup>(nh_, srv, "/LoadGroup")) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::UnloadGroup(const std::string &dc_group_name) {
  rtr_msgs::DeconGroup srv;
  srv.request.group_name = dc_group_name;
  if (!CallRosService<rtr_msgs::DeconGroup>(nh_, srv, "/UnloadGroup")) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::SetProjectRobotParam(
    const std::string &prj_name, const std::string &param_json_path) {
  // take the robot_param json file and create a json obj
  std::ifstream ifs(param_json_path);
  nlohmann::json j = nlohmann::json::parse(ifs);

  // Update Project Info
  rtr_msgs::UpdateProject srv;
  srv.request.project_name = prj_name;
  srv.request.json_data = j.dump();
  if (!CallRosService<rtr_msgs::UpdateProject>(nh_, srv, "/UpdateProject") ||
      srv.response.result_code != 0) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::GetLoadedDeconGroup(
    rtr_msgs::DeconGroupInfo &loaded_group) {
  rtr_msgs::GetGroupInfo srv;
  if (!CallRosService<rtr_msgs::GetGroupInfo>(nh_, srv, "/GetDeconGroupInfo")) {
    return false;
  }

  for (const auto &group : srv.response.groups) {
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
      rtr_msgs::GroupProject remove_srv;
      remove_srv.request.group_name = group.GroupName;
      remove_srv.request.project_name = project;
      if (!CallRosService<rtr_msgs::GroupProject>(nh_, remove_srv, "/RemoveProjectFromGroup")
          || remove_srv.response.result_code != Convert<int>(ExtCode::SUCCESS)) {
        return false;
      }
      rtr_msgs::DeleteProject delete_srv;
      delete_srv.request.project_name = project;
      if (!CallRosService<rtr_msgs::DeleteProject>(nh_, delete_srv, "/DeleteProject")
          || delete_srv.response.result_code != Convert<int>(ExtCode::SUCCESS)) {
        return false;
      }
    }
    rtr_msgs::DeconGroup delete_srv;
    delete_srv.request.group_name = group.GroupName;
    if (!CallRosService<rtr_msgs::DeconGroup>(nh_, delete_srv, "/DeleteGroup")
        || delete_srv.response.result_code != Convert<int>(ExtCode::SUCCESS)) {
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

