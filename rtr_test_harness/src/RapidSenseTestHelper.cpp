#include "rtr_test_harness/RapidSenseTestHelper.hpp"

#include <ros/package.h>

#include <rtr_perc_rapidsense_ros/RapidSenseStatus.hpp>
#include <rtr_perc_rapidsense_ros/SchemaMessageHelpers.hpp>
#include <rtr_utils/Logging.hpp>

namespace rtr {
namespace perception {

RapidSenseTestHelper::RapidSenseTestHelper(ros::NodeHandle& nh) : ApplianceTestHelper{nh} {}

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

bool RapidSenseTestHelper::CheckRapidSenseServerState(RapidSenseState& state) {
  RapidSenseHealth health_;
  if (!this->GetRapidSenseServerHealth(health_)) {
    return false;
  }

  return health_.current_status.state == state;
}

}  // namespace perception
}  // namespace rtr
