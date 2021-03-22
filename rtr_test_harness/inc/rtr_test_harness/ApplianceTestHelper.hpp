#include <set>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <gtest/gtest.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/service.h>

#include <rtr_appliance/ApplianceCommander.hpp>
#include <rtr_msgs/DeconGroupInfo.h>
#include <rtr_test_harness/WebappCommander.hpp>
#include <rtr_utils/Logging.hpp>
#include <rtr_app_layer/RapidPlanProject.hpp>

const std::string kDefaultApplianceIP = "127.0.0.1";

namespace rtr {

// @brief Class to help setup the appliance, as would a rtr customer
class ApplianceTestHelper {
 public:
  ApplianceTestHelper(ros::NodeHandle& nh);

  // @brief Add Rapidplan project to be used with test
  bool AddToolkitProject(const std::string& zip_path);

  // @brief Add Rapidplan project to be used with test
  bool SetupDefault();

  ApplianceCommander app_cmdr_;
  WebappCommander webapp_cmdr_;

 protected:
  ros::NodeHandle nh_;
  std::map<std::string, RapidPlanProject::Ptr> toolkit_projects_;
};

}  // namespace rtr
