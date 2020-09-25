#include "rtr_test_harness/RapidSenseTestHarnessServer.hpp"

#include <rtr_app_layer/RapidPlanProject.hpp>
#include <rtr_control_ros/RosController.hpp>
#include <rtr_msgs/GetGroupInfo.h>
#include <rtr_msgs/GetProjectROSInfo.h>
#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
#include <rtr_perc_rapidsense_ros/RosRobotConnection.hpp>
#include <rtr_appliance/Appliance.hpp>
#include <rtr_utils/Backtrace.hpp>
#include <rtr_utils/Logging.hpp>
#include <rtr_voxelize/VoxelizerFactory.hpp>

//using ExtCodeSeqPair = rtr::ApplianceCommander::ExtCodeSeqPair;

namespace rtr {
namespace perception {

RapidSenseTestHarnessServer::RapidSenseTestHarnessServer() : nh_("~"), spinner_(10) {

}

bool RapidSenseTestHarnessServer::SetUp(const std::string& , const std::string& ) {

  InitializeLogging("TestHarness", "args_logs_dir", "conf_logs_dir");
#if 0
  // Setup the Appliance application  
  auto factory = VoxelizerFactory::GetInstance();
  factory->LoadPlugin("opencl", "librtr_voxelize_plugin_opencl.so.1");

  // Load the configuration stuff
  ApplianceConfig appl_config_opt;
    
  // Create the directory if it doesn't exist
  const bfs::path appl_path = app_dir + "/appliance_data";
  boost::system::error_code ec;
  if (!bfs::exists(appl_path, ec) || ec != nullptr) {
    bfs::create_directory(appl_path, ec);
  }

  appliance_ = Appliance::Create(appl_path.string(), appl_config_opt);
  if (appliance_ == nullptr) {
    RTR_ERROR("Error initializing appliance!");
    return false;
  }
#endif
  spinner_.start();

#if 0
 // TBD: CPU Affinity is no longer being valued in bringing both together. So this
 // may become an issue.

  RapidSenseServerInit("RapidSenseServer", "args_logs_dir", "conf_logs_dir");
  rapidsense_ = RapidSenseServer::Create(nh_, rs_dir);
  if (!rapidsense_->GetStatus().IsSuccess()) {
    RTR_ERROR("RapidSenseServer failed to initialized. Shutting down");
    return false;
  }
#endif

  simulator_ = SensorSimulator::MakePtr(nh_);
  boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)> callback =
      [this](std_srvs::Trigger::Request&,
             std_srvs::Trigger::Response& res) -> bool {
    //this->simulator_->Shutdown();
    res.success = this->simulator_->Init(true);
    return true;
  };
  restart_sim_ = nh_.advertiseService("/restart_sim", callback);

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
      //appliance_.reset();
      //rapidsense_->Shutdown();
      simulator_->Shutdown();
  }

}
}