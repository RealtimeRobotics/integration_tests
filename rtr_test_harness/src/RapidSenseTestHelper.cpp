#include "rtr_test_harness/RapidSenseTestHelper.hpp"

#include <ros/package.h>

#include <rtr_perc_rapidsense_ros/RapidSenseStatus.hpp>
#include <rtr_perc_rapidsense_ros/SchemaMessageHelpers.hpp>
#include <rtr_utils/Logging.hpp>
#include <rtr_utils/Strings.hpp>

#include "rtr_perc_rapidsense_ros/Record.hpp"

namespace bfs = boost::filesystem;
namespace rtr {
namespace perception {

RapidSenseTestHelper::RapidSenseTestHelper(ros::NodeHandle& nh) : ApplianceTestHelper{nh} {}

RapidSenseTestHelper::~RapidSenseTestHelper() {
  simulator_->Shutdown();
}

bool RapidSenseTestHelper::SetupFixture_SimulatedSensors() {
  std::string project_dir = DirName(toolkit_projects_.front().first);
  for (auto& project : toolkit_projects_) {
    if (project_dir != DirName(project.first)) {
      RTR_ERROR("all toolkit projects must be in the same directory");
      return false;
    }
  }

  std::string rapidsense_data = project_dir + "/rapidsense_data";
  if (bfs::exists(rapidsense_data)) {
    RTR_WARN("Rapidsense data exists, copying to active rapidsense dir");
    std::string rapidsense_data_directory =
        fmt::format("{}/{}/", kDefaultRapidsenseDirectory, kDefaultDeconGroupName);
    CopyFolder(rapidsense_data, kDefaultRapidsenseDirectory);
  }

  if (!SetupFixture_LoadedProjects()) {
    RTR_ERROR("Failed to setup appliance");
    return false;
  }

  if (!StartSensorSimulator()) {
    RTR_ERROR("Failed to start sensor simulator");
    return false;
  }

  if (CheckRapidSenseServerState(RapidSenseState::CONFIGURE)) {
    RTR_ERROR("Rapidsense server is not in configure mode");
    return false;
  }
  return true;
}

bool RapidSenseTestHelper::GetRapidSenseServerConfig(SpatialPerceptionProjectSchema& config) {
  ros::ServiceClient get_config_client =
      nh_.serviceClient<rtr_msgs::GetSchemaMessage>(RS::Topic("get_configuration"));
  if (!ros::service::waitForService(get_config_client.getService(), ros::Duration(10.0))) {
    RTR_ERROR("Timed out waiting for configuration from RapidSenseServer");
    return false;
  }

  rtr_msgs::GetSchemaMessage srv;
  if (!get_config_client.call(srv) || !FromSchemaMessageResponse(srv.response, config)) {
    RTR_ERROR("Failed to get configuration from RapidSenseServer");
    return false;
  }

  return true;
}

bool RapidSenseTestHelper::CheckRapidSenseServerConfig(RapidSenseTestConfig& config) {
  SpatialPerceptionProjectSchema rapidsense_config_;
  if (!this->GetRapidSenseServerConfig(rapidsense_config_)) {
    return false;
  }

  for (auto& stream : rapidsense_config_.streams) {
    if (stream.enable_robotfilter != config.test_robot_filter) {
      return false;
    }
  }

  return true;
}

bool RapidSenseTestHelper::SetRapidSenseServerConfig(RapidSenseTestConfig& config) {
  SpatialPerceptionProjectSchema rapidsense_config_;
  if (!this->GetRapidSenseServerConfig(rapidsense_config_)) {
    return false;
  }

  for (auto& stream : rapidsense_config_.streams) {
    if (stream.enable_robotfilter != config.test_robot_filter) {
      stream.enable_robotfilter = config.test_robot_filter;
    }
  }

  return true;
}

bool RapidSenseTestHelper::GetRapidSenseServerHealth(RapidSenseHealth& health) {
  rtr_msgs::SchemaMessage::ConstPtr health_msg =
      ros::topic::waitForMessage<rtr_msgs::SchemaMessage>(RS::Topic("health"), ros::Duration(10.0));
  if (!health_msg) {
    RTR_ERROR("Timed out waiting for health state from RapidSenseServer");
    return false;
  }

  try {
    health = FromSchemaMessage<RapidSenseHealth>(*health_msg);
  } catch (std::exception& e) {
    RTR_ERROR("Cannot parse RapidSenseHealth from schema message: [{}]", health_msg->data);
    return false;
  }

  return true;
}

bool RapidSenseTestHelper::CheckRapidSenseServerState(RapidSenseState state) {
  RapidSenseHealth health_;
  if (!this->GetRapidSenseServerHealth(health_)) {
    return false;
  }

  return health_.current_status.state == state;
}

bool RapidSenseTestHelper::StartSensorSimulator() {
  simulator_ = SensorSimulator::MakePtr(nh_);
  boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)> callback =
      [this](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) -> bool {
    this->simulator_->Shutdown();
    res.success = this->simulator_->Init(true);
    return true;
  };
  restart_sim_ = nh_.advertiseService("/restart_sim", callback);
  return true;
}

}  // namespace perception
}  // namespace rtr
