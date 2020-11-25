#include "rtr_test_harness/RapidSenseTestHarnessServer.hpp"

#include <boost/filesystem.hpp>
#include <ros/topic.h>

#include <rtr_app_layer/RapidPlanProject.hpp>
#include <rtr_appliance/Appliance.hpp>
#include <rtr_control_ros/RosController.hpp>
#include <rtr_msgs/GetGroupInfo.h>
#include <rtr_msgs/GetProjectROSInfo.h>
#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
#include <rtr_perc_rapidsense_ros/RosRobotConnection.hpp>
#include <rtr_utils/Backtrace.hpp>
#include <rtr_utils/Logging.hpp>
#include <rtr_voxelize/VoxelizerFactory.hpp>

namespace bfs = boost::filesystem;

namespace rtr {
namespace perception {

RapidSenseTestHarnessServer::RapidSenseTestHarnessServer() : nh_("~"), spinner_(10) {}

bool RapidSenseTestHarnessServer::SetUp(const std::string& app_dir, const std::string&) {
  InitializeLogging("TestHarness", "args_logs_dir", "conf_logs_dir");
  const bfs::path appl_path = app_dir + "/appliance_data";
  boost::system::error_code ec;
  if (!bfs::exists(appl_path, ec) || ec != nullptr) {
    bfs::create_directory(appl_path, ec);
  }

  spinner_.start();

  simulator_ = SensorSimulator::MakePtr(nh_);
  boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)> callback =
      [this](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) -> bool {
    this->simulator_->Shutdown();
    res.success = this->simulator_->Init(true);
    return true;
  };
  restart_sim_ = nh_.advertiseService("/restart_sim", callback);

  if (!ros::topic::waitForMessage<std_msgs::String>("/appliance_state", ros::Duration(30))) {
    RTR_ERROR("Timed out waiting for appliance");
    return false;
  }

  if (!ros::topic::waitForMessage<rtr_msgs::SchemaMessage>("/rapidsense/health",
                                                           ros::Duration(30))) {
    RTR_ERROR("Timed out waiting for RapidSense server");
    return false;
  }

  return true;
}

#if 0
  bool RapidSenseTestHarnessServer::SetupDeconflictionGroup(const std::string& project, const std::string& DC_group) {

    rtr::Code code;
    code = appliance_->InstallProject(project);
    if (code == Code::OK) {
    RTR_INFO("Installed project");
    } else {
    RTR_ERROR("Error Unzipping File: {}", project);
    return false;
    }

    std::vector<std::string> proj={project};
    code = appliance_->CreateDeconflictionGroup(DC_group, proj);

    code = appliance_->LoadProject(DC_group);
    if (code == Code::OK) {
    RTR_INFO("Loaded DC group");
    } else {
    RTR_ERROR("Unable to load DC group");
    return false;
    }
    return true;
  }
#endif

void RapidSenseTestHarnessServer::Teardown() {
  simulator_->Shutdown();
}

}  // namespace perception
}  // namespace rtr
