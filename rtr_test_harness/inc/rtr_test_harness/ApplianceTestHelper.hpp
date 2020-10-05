#include <set>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/service.h>
#include <ros/node_handle.h>
//#include <QApplication>
#include <gtest/gtest.h>

//#include <rtr_perc_sensors/SensorCalibrationData.hpp>
#include <rtr_utils/Logging.hpp>

//#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
//#include <rtr_appliance/Appliance.hpp>
//#include "rtr_test_harness/RapidSenseTestHarnessServer.hpp"
#include <rtr_msgs/DeconGroupInfo.h>

namespace rtr {

template<typename MessageType> bool CallRosService(ros::NodeHandle& nh,
                                                   MessageType &srv,
                                                   const std::string& serv) {
  // TODO: We need a timeout on watiForService, but uncertain of how this
  // can be done without hardcoding
  if(!ros::service::waitForService(serv)){
    RTR_ERROR("No service {}", serv);
    return false;
  }

  RTR_DEBUG("Calling service {} with {}", serv, srv.request);
  ros::ServiceClient client = nh.serviceClient<MessageType>(serv);
  if (client.call(srv)) {
    RTR_DEBUG("Success with {}", srv.response);
    return true;
  }
  else
  {
    RTR_ERROR("Failed to call {}", serv);
  }
  return false;
}

class ApplianceTestHelper {
public:
  ApplianceTestHelper(ros::NodeHandle& nh);

  bool InstallProject(const std::string& project_zip);

  bool GetInstalledProjects(std::vector<std::string>& projects);

  bool AddAllProjectsToDeconGroup(const std::string& dc_group_name);

  bool SetVisionEnabled(const std::string& dc_group_name, bool is_enabled);

  bool LoadGroup(const std::string& dc_group_name);

  bool UnloadGroup(const std::string& dc_group_name);

  bool SetProjectRobotParam(const std::string& prj_name,
                            const std::string& param_json_path);

  bool GetLoadedDeconGroup(rtr_msgs::DeconGroupInfo& loaded_group);

protected:
  ros::NodeHandle nh_;
};

} // rtr
