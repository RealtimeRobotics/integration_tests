#include "rtr_test_harness/ApplianceTestHelper.hpp"

#include <fstream>

#include <nlohmann-json/json.hpp>

#include <rtr_utils/Logging.hpp>
#include <rtr_utils/ZipUtils.hpp>
#include <rtr_utils/Strings.hpp>
#include <rtr_utils/time/Timer.hpp>

namespace bfs = boost::filesystem;
namespace rtr {

ApplianceTestHelper::ApplianceTestHelper(ros::NodeHandle& nh)
    : app_cmdr_(kDefaultApplianceIP), webapp_cmdr_(nh), nh_(nh) {
}

bool ApplianceTestHelper::AddToolkitProject(const std::string& zip_path) {
  std::string extracted_path = "/tmp/" + FileName(zip_path);
  zip::Unzip(zip_path, extracted_path);

  rtr::RapidPlanProject::Ptr prj;
  if (!prj->Load(extracted_path)) {
    RTR_WARN("Toolkit project failed to load {}", zip_path);
  }

  if (toolkit_projects_.find(zip_path) == toolkit_projects_.end()) {
    toolkit_projects_.emplace(zip_path, prj);
  } else {
    RTR_WARN("Toolkit Project aready exists: {}", zip_path);
    return false;
  }
  return true;
}

bool ApplianceTestHelper::SetupFixture_LoadedProjects() {
  // Wait for appliance to start up before setting up
  WaitForApplianceServer();

  // Install all added projects to the appliance
  for (const auto& prj : toolkit_projects_) {
    if (!webapp_cmdr_.InstallProject(prj.first)) {
      RTR_ERROR("Failed to install project {}", prj.first);
      return false;
    }

    std::string robot_param_path = RemoveExtension(prj.first) + ".json";
    if (!webapp_cmdr_.SetProjectRobotParam(prj.second->GetName(), robot_param_path)) {
      RTR_ERROR("Failed to robot params {}", prj.first);
      return false;
    }
  }

  if (!webapp_cmdr_.AddAllProjectsToDeconGroup(kDefaultDeconGroupName)) {
    RTR_ERROR("Failed add projects to decon group");
    return false;
  }

  if (!webapp_cmdr_.LoadGroup(kDefaultDeconGroupName)) {
    RTR_ERROR("Failed load group");
    return false;
  }

  for (const auto& prj : toolkit_projects_) {
    if(ExtCode::SUCCESS !=  app_cmdr_.InitGroup(kDefaultDeconGroupName,
                                                prj.second->GetName(),
                                                prj.second->GetStateSpaceNames()[0]) ) {
      RTR_ERROR("Failed init group");
      return false;
    }
  }
  return true;
}

bool ApplianceTestHelper::WaitForApplianceServer(const std::string& app_dir) {
  boost::system::error_code ec;

  // wait for appliance to create directory structure
  rtr::Timer timer(true);
  while (timer.Elapsed() < 30.0 && (!bfs::exists(app_dir, ec) || ec != nullptr)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  if (!bfs::exists(app_dir)) {
    RTR_ERROR("Timed out waiting for Appliance to create directory structure");
    return false;
  }
  return true;
}

}  // namespace rtr
