#include "rtr_test_harness/ApplianceTestHelper.hpp"
//#include <rtr_app_layer/RapidPlanProject.hpp>
//#include <rtr_control_ros/RosController.hpp>
//#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
//#include <rtr_perc_rapidsense_ros/RosRobotConnection.hpp>
//#include <rtr_appliance/Appliance.hpp>
//#include <rtr_utils/Backtrace.hpp>
//#include <rtr_utils/Logging.hpp>

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

ApplianceTestHelper::ApplianceTestHelper(ros::NodeHandle& nh): nh_(nh) {

  rtr_msgs::SetEULAAccepted srv;
  srv.request.signature = "unittest";
  CallRosService<rtr_msgs::SetEULAAccepted>(nh_, srv, "/SetEULAAccepted");
}

bool ApplianceTestHelper::InstallProject(const std::string& project_zip) {

  rtr_msgs::InstallProject srv;
  srv.request.zip_file_path = project_zip;
  if(!CallRosService<rtr_msgs::InstallProject>(nh_, srv, "/InstallProject")) {
    return false;
  }
  return true;
}

bool GetInstalledProjects(std::vector<std::string>& projects) {

  rtr_msgs::GetProjectList msg_prjs;
  if(!CallRosService<rtr_msgs::GetProjectList>(nh_, msg_prjs, "/GetProjectList")) {
    return false;
  }
  projects = std::vector<std::string>(msg_prjs.response.projects);

  return true;
}

bool ApplianceTestHelper::AddAllProjectsToDeconGroup(const std::string& dc_group_name) {

  // Create the decon group
  rtr_msgs::DeconGroup msg_grp;
  msg_grp.request.group_name = dc_group_name;
  if(!CallRosService<rtr_msgs::DeconGroup>(nh_, msg_grp, "/CreateGroup")) {
    return false;
  }

  // Get all list of all projects that are installed
  std::vector<std::string> projects;
  if(!this->GetInstalledProjects(projects)) {
    return false;
  }

  // Add all installed projects to the decon group
  for(const auto& P : projects)
  {
    rtr_msgs::GroupProject msg_add_prj;
    msg_add_prj.request.group_name = dc_group_name;
    msg_add_prj.request.project_name = P;
    if(!CallRosService<rtr_msgs::GroupProject>(nh_, msg_add_prj, "/AddProjectToGroup")) {
      return false;
    }
  }

  return true;
}

bool ApplianceTestHelper::SetVisionEnabled(const std::string& dc_group_name,
                                           bool is_enabled) {
  rtr_msgs::SetVisionEnabled srv;
  srv.request.group_name = dc_group_name;
  srv.request.enabled = is_enabled;
  if(!CallRosService<rtr_msgs::SetVisionEnabled>(nh_, srv, "/SetVisionEnabled")) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::LoadGroup(const std::string& dc_group_name) {

  rtr_msgs::DeconGroup srv;
  srv.request.group_name = dc_group_name;
  if(!CallRosService<rtr_msgs::DeconGroup>(nh_, srv, "/LoadGroup")) {
    return false;
  }

  return true;
}

bool ApplianceTestHelper::UnloadGroup(const std::string& dc_group_name) {

  rtr_msgs::DeconGroup srv;
  srv.request.group_name = dc_group_name;
  if(!CallRosService<rtr_msgs::DeconGroup>(nh_, srv, "/UnloadGroup")) {
    return false;
  }

  return true;
}


// rtr_appliance will overwrite all robot_params with what is provided to the UpdateProject service.
// Thus the current robot param will need to be retrieved, modified and then sent back to the appliance
template<typename ParamType> bool ApplianceTestHelper::SetProjectRobotParam(
                                                const std::string& prj_name,
                                                const std::string& param_k,
                                                ParamType param_v) {
  // Get project info
  rtr_msgs::GetProjectROSInfo prj_info;
  prj_info.request.project_name = prj_name;
  if(!CallRosService<rtr_msgs::GetProjectROSInfo>(nh_, prj_info, "/GetProjectROSInfo")
      || !prj_info.response.valid) {
    return false;
  }
  std::string robot_params = prj_info.response.robot_params;

  // Modify retrieved robot_params
  nlohmann::json j = nlohmann::json::parse(robot_params);

  // if key does not exist, return
  if (j.find(param_k) == j.end()) {
    return false;
  }
  j["string_params"][param_k] = param_v;
  // Update Project Info
  rtr_msgs::UpdateProject srv;
  srv.request.project_name = prj_name;
  srv.request.json_data = j.dump();
  if(!CallRosService<rtr_msgs::UpdateProject>(nh_, srv, "/UpdateProject")
      || srv.response.result_code != 0) {
    return false;
  }
  return true;
}

bool SetAllProjectsToSimulated() {

  // Get all list of all projects that are installed
  std::vector<std::string> projects;
  if(!this->GetInstalledProjects(projects)) {
    return false;
  }

  // Set Connection Type to simulated (1)
  for (auto& p : projects) {
    if(!this->SetProjectRobotParam(p, "connection_type", 1)) {
      return false;
    }
  }
}

bool ApplianceTestHelper::GetLoadedDeconGroup(rtr_msgs::DeconGroupInfo& loaded_group) {

  rtr_msgs::GetGroupInfo srv;
  if(!CallRosService<rtr_msgs::GetGroupInfo>(nh_, srv, "/GetDeconGroupInfo")) {
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

} // namespce rtr
