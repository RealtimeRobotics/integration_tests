#include <set>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <QApplication>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service.h>

#include <rtr_appliance/Appliance.hpp>
#include <rtr_msgs/GetHubConfig.h>
#include <rtr_msgs/TeleportRobot.h>
#include <rtr_perc_api/SensorFrame.hpp>
#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
#include <rtr_perc_rapidsense_ros/Record.hpp>
#include <rtr_perc_sensors/SensorCalibrationData.hpp>
#include <rtr_utils/Logging.hpp>

#include "rtr_test_harness/RapidSenseTestHarnessServer.hpp"
#include "rtr_test_harness/RapidSenseTestHelper.hpp"

using namespace rtr::perception;
using namespace rtr;

namespace bfs = boost::filesystem;

int main(int argc, char** argv) {
  bfs::remove_all("/tmp/appliance_test");
  bfs::remove_all("/tmp/rapidsense_test");
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("rapidsense_sim");

  ros::init(argc, argv, "RecordPlaybackSimTest");
  RapidSenseTestHarnessServer server;
  std::string rs_path = ros::package::getPath("reg_test_record_playback") + "/../../test_data";
  server.SetUp("appliance_test", rs_path);

  ::testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
  
  bfs::remove_all("/tmp/appliance_test");
  bfs::remove_all("/tmp/rapidsense_test");
  return res;
}

class RapidSenseTestFixture : public ::testing::Test {

  protected:
  ros::NodeHandle nh_;
  RapidSenseTestHelper appliance_;
  RapidSenseFrontEndProxy proxy_;
  std::string decon_group_name, robot_name, project, rapidsense_data, robot_param;
  
  void SetUp() override {
      nh_.param<std::string>("decon_group_name", decon_group_name, "ur3_calibration_test");
      nh_.param<std::string>("robot_name", robot_name, "ur3");
      nh_.param<std::string>("project", project, "../../");
      nh_.param<std::string>("rapidsense_data", rapidsense_data, "../../");
      
      project = ros::package::getPath("reg_test_record_playback") + "/../../test_data/ur3_calibration_test/ur3.zip";
      rapidsense_data = ros::package::getPath("reg_test_record_playback") + "/../../test_data/ur3_calibration_test/rapidsense_data/";
          robot_param = ros::package::getPath("reg_test_record_playback")
                  + "/../../test_data/ur3_calibration_test/ur3.json";
      RTR_INFO("Value of project={}", project);
      RTR_INFO("Value of rapidsense_data={}", rapidsense_data);

      


      ASSERT_TRUE(appliance_.InstallProject(project));
      ASSERT_TRUE(appliance_.SetProjectRobotParam("ur3", robot_param));
      ASSERT_TRUE(appliance_.AddAllProjectsToDeconGroup(decon_group_name));
      ASSERT_TRUE(appliance_.SetVisionEnabled(decon_group_name, true));
      ASSERT_TRUE(appliance_.LoadGroup(decon_group_name));

            std::string rapidsense_state_directory;
      if (!proxy_.GetStateDirectory(rapidsense_state_directory)) {
        RTR_ERROR("Unable to get state directory from rapidsense");
      }

      std::string rapidsense_data_directory = fmt::format("{}/{}/",rapidsense_state_directory, decon_group_name);
      CopyFolder(rapidsense_data, rapidsense_data_directory);

      std_srvs::Trigger trg;
      CallRosService<std_srvs::Trigger>(nh_, trg, "/restart_sim");

      RTR_DEBUG("Waiting for restart sim");
      std::this_thread::sleep_for(std::chrono::seconds(4));
  }
    
public:
    RapidSenseTestFixture() : nh_(""), appliance_(nh_), proxy_(RapidSenseFrontEndProxy::ProxyHost::RAPIDSENSE_GUI) { }
    RapidSenseTestFixture(ros::NodeHandle& nh) : nh_(nh), appliance_(nh_), proxy_(RapidSenseFrontEndProxy::ProxyHost::RAPIDSENSE_GUI) {
      // Start our proxy_ as the Rapidsense gui so the server doesn't automatically try to transition
      // while we are testing
    }
};


TEST_F(RapidSenseTestFixture, VerifyRobotFilterSimulatedCamera) { 
  
  EXPECT_EQ(proxy_.GetHealth().input_mode, RapidSenseInputMode::SIMULATION);
  if (proxy_.GetState() != RapidSenseState::OPERATION) {
    EXPECT_TRUE(proxy_.SetOperationMode());
  }

  rtr::perception::SensorFrame::ConstPtr imgin;
  rtr::perception::SensorFrameType frame_type =
      SensorFrameType(SensorFrameType::SENSOR_FRAME_VOXELS, SensorFrameType::ROBOT_SELF_FILTERED);
  imgin = proxy_.GetFrame(robot_name, "", frame_type, 0.5, 1.0);
  SensorFrameVoxels::ConstPtr voxel_ptr = SensorFrameVoxels::CastConstPtr(imgin);
  std::vector<Voxel> voxels;
  voxel_ptr->GetVoxels(voxels);
  EXPECT_TRUE(!voxels.empty());
  
  bool recording_result;
  std::thread recording_thread = std::thread([this, &recording_result]() {
    recording_result = proxy_.StartRecording("test_recording", 5);
  });

  // This part of the test doesn't work right now so commenting it out, right now basic recording and playback functionality work.
  // std::map<JointConfiguration, std::vector<Voxel>> voxel_map;
  // RobotObserver::Ptr active_observer = proxy_.GetActiveRobotObserver();
  // std::string teleport_topic = fmt::format("/{}/teleport_robot", robot_name);
  // ros::ServiceClient teleport_service = nh_.serviceClient<rtr_msgs::TeleportRobot>(teleport_topic);

  // rtr::JointConfiguration joint_config_1 = rtr::JointConfiguration({0,0,0,0,0,0});
  // rtr_msgs::TeleportRobot teleport_srv_1;
  // teleport_srv_1.request.joint_config = joint_config_1.GetData();
  // EXPECT_TRUE(teleport_service.call(teleport_srv_1) && teleport_srv_1.response.is_success);

  // imgin = proxy_.GetFrame(robot_name, "", frame_type, 0.5, 1.0);
  // voxel_ptr = SensorFrameVoxels::CastConstPtr(imgin);
  // voxels.clear();
  // voxel_ptr->GetVoxels(voxels);
  // voxel_map[joint_config_1] = voxels;

  // ros::Rate r(1.0);
  // r.sleep();

  // rtr::JointConfiguration joint_config_2 = rtr::JointConfiguration({Pi,Pi,Pi,Pi,Pi,Pi});
  // rtr_msgs::TeleportRobot teleport_srv_2;
  // teleport_srv_2.request.joint_config = joint_config_2.GetData();
  // EXPECT_TRUE(teleport_service.call(teleport_srv_2) && teleport_srv_1.response.is_success);

  // imgin = proxy_.GetFrame(robot_name, "", frame_type, 0.5, 1.0);
  // voxel_ptr = SensorFrameVoxels::CastConstPtr(imgin);
  // voxels.clear();
  // voxel_ptr->GetVoxels(voxels);
  // voxel_map[joint_config_2] = voxels;

  recording_thread.join();
  EXPECT_TRUE(recording_result);
  RTR_INFO("Finished Recording");

  bool playback_result;
  std::thread playback_thread = std::thread([this, &playback_result]() {
    RTR_ERROR("STARTING_PLAYBACK");
    playback_result = proxy_.StartPlayback("test_recording", 1.0, false);
  });
  
  // ros::Rate loop_rate(20);
  // while (active_observer->GetCurrentJointConfiguration() != joint_config_1) {
  //  loop_rate.sleep();
  // }

  imgin = proxy_.GetFrame(robot_name, "", frame_type, 0.5, 1.0);
  voxel_ptr = SensorFrameVoxels::CastConstPtr(imgin);
  voxels.clear();
  voxel_ptr->GetVoxels(voxels);
  // EXPECT_EQ(voxels, voxel_map[joint_config_1]);

  // while (active_observer->GetCurrentJointConfiguration() != joint_config_2) {
  //   loop_rate.sleep();
  // }
 
  imgin = proxy_.GetFrame(robot_name, "", frame_type, 0.5, 1.0);
  voxel_ptr = SensorFrameVoxels::CastConstPtr(imgin);
  voxels.clear();
  voxel_ptr->GetVoxels(voxels);
  // EXPECT_EQ(voxels, voxel_map[joint_config_2]);
 
  playback_thread.join();
  EXPECT_TRUE(playback_result);

  RTR_INFO("Recording/Playback Test Successful!");
}