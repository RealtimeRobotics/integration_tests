#include <set>
#include <string>
#include <vector>

#include <ros/service.h>
#include <QApplication>

#include <rtr_msgs/GetHubConfig.h>
#include <rtr_msgs/TeleportRobot.h>
#include <rtr_perc_sensors/SensorCalibrationData.hpp>
#include <rtr_utils/Logging.hpp>
#include <gtest/gtest.h>

#include "rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp"
#include "rtr_appliance/Appliance.hpp"
#include "rtr_test_harness/RapidSenseTestHarnessServer.hpp"

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
#include <rtr_msgs/UpdateGroup.h>
#include <rtr_msgs/UpdateProject.h>
#include <rtr_msgs/Version.h>

namespace rtr {
namespace perception {

template<typename MessageType> bool CallRosService(ros::NodeHandle& nh, MessageType &srv, const std::string& serv) {
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
 
class RapidSenseTestHelper {
public:
    RapidSenseTestHelper(ros::NodeHandle& nh);

    bool CreateAndSetupProject(const std::string& dc_group_name,
                               const std::string& project_zip);

protected:
  ros::NodeHandle nh_; 
};

}
}
