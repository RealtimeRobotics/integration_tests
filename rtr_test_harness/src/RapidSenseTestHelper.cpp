#include "rtr_test_harness/RapidSenseTestHelper.hpp"

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

#include "ros/ros.h"

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

RapidSenseTestHelper::RapidSenseTestHelper(ros::NodeHandle& nh): nh_(nh) {
    
    rtr_msgs::SetEULAAccepted srv;
    srv.request.signature = "unittest";
    CallRosService<rtr_msgs::SetEULAAccepted>(nh_, srv, "/SetEULAAccepted");
}
    
bool RapidSenseTestHelper::CreateAndSetupProject(const std::string& dc_group_name,
                               const std::string& project_zip) { 
    {
        rtr_msgs::DeconGroup srv;
        srv.request.group_name = dc_group_name;
        if(!CallRosService<rtr_msgs::DeconGroup>(nh_, srv, "/CreateGroup")){
            return false;
        }
    }
    {
        rtr_msgs::InstallProject srv;
        srv.request.zip_file_path = project_zip;
        if(!CallRosService<rtr_msgs::InstallProject>(nh_, srv, "/InstallProject")){
            return false;
        }
    }
    std::vector<std::string> projects;
    {
        rtr_msgs::GetProjectList srv;
        if(!CallRosService<rtr_msgs::GetProjectList>(nh_, srv, "/GetProjectList")){
            return false;
        }
        projects = std::vector<std::string>(srv.response.projects);
    }
    for(const auto& P : projects)
    {
        rtr_msgs::GroupProject srv;
        srv.request.group_name = dc_group_name;
        srv.request.project_name = P;
        if(!CallRosService<rtr_msgs::GroupProject>(nh_, srv, "/AddProjectToGroup")){
            return false;
        }

        rtr_msgs::UpdateProject srv1;
        srv1.request.project_name = P;
        srv1.request.json_data = "{\"connection_type\": 1}";
        if(!CallRosService<rtr_msgs::UpdateProject>(nh_, srv1, "/UpdateProject")){
            return false;
        }
    }
    {
        rtr_msgs::SetVisionEnabled srv;
        srv.request.group_name = dc_group_name;
        srv.request.enabled = true;
        if(!CallRosService<rtr_msgs::SetVisionEnabled>(nh_, srv, "/SetVisionEnabled")){
            return false;
        }
    }
    {
        rtr_msgs::DeconGroup srv;
        srv.request.group_name = dc_group_name;
        if(!CallRosService<rtr_msgs::DeconGroup>(nh_, srv, "/LoadGroup")){
            return false;
        }
    }
    return true;
  }

}
}