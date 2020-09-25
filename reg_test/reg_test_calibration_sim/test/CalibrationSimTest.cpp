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
#include "rtr_test_harness/RapidSenseTestHelper.hpp"

using namespace rtr::perception;
using namespace rtr;

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("rapidsense_sim");

  ros::init(argc, argv, "CalibrationSimTest");
  ::testing::InitGoogleTest(&argc, argv);

  RapidSenseTestHarnessServer server;
  server.SetUp("appliance_dir", "~/.rapidsense");

  RUN_ALL_TESTS();
  ros::waitForShutdown();
  server.Teardown();
}

class CalibrationTestFixture : public ::testing::Test {

  protected:
  ros::NodeHandle nh_;
  RapidSenseFrontEndProxy proxy;
  RapidSenseTestHelper appliance_;
  std::string robot_name, flange_frame, hub_name, project;
  
  void SetUp() override {
      nh_.param<std::string>("robot_name", robot_name, "ur3");
      nh_.param<std::string>("flange_frame", flange_frame, "ur3/flange");
      nh_.param<std::string>("hub_name", hub_name, "h_3");
      nh_.param<std::string>("project", project, "../../");

      project = ros::package::getPath(reg_test_calibration_sim) + "/../../test_data/ur3_calibration_test/ur3-obstacle.zip";
      //"/home/krishna/workspaces/master/src/rapidsense_test/reg_test/reg_test_calibration_sim/../../test_data/ur3_calibration_test/ur3-obstacle.zip"; //ros::package::getPath("reg_test_calibration_sim") +
      RTR_INFO("Value of project={}", project);
      
      appliance_.CreateAndSetupProject("unit_test_decon_group", project);
      std_srvs::Trigger trg;
      CallRosService<std_srvs::Trigger>(nh_, trg, "/restart_sim");

      RTR_DEBUG("Waiting for restart sim");
      std::this_thread::sleep_for(std::chrono::seconds(4));
  }

  void TearDown() override {

  }
    
public:
    CalibrationTestFixture() : nh_(""), appliance_(nh_), proxy(RapidSenseFrontEndProxy::ProxyHost::RAPIDSENSE_GUI) {}
    CalibrationTestFixture(ros::NodeHandle& nh) : nh_(nh), appliance_(nh_), proxy(RapidSenseFrontEndProxy::ProxyHost::RAPIDSENSE_GUI) {
      // Start our proxy as the Rapidsense gui so the server doesn't automatically try to transition
      // while we are testing
    }
};


TEST_F(CalibrationTestFixture, VerifyCailbrationWorkflowWithPreviousLoc) { 
  
  EXPECT_EQ(proxy.GetHealth().input_mode, RapidSenseInputMode::SIMULATION);
  EXPECT_EQ(proxy.GetState(), RapidSenseState::CONFIGURE);
  // Configure the Robot for Calibration
  // Set Active Observer (arg)
  std::string active_observer = robot_name;
  EXPECT_TRUE(proxy.SetActiveRobotObserver(active_observer));
  
  // Set Flange Frame (arg)
  EXPECT_TRUE(proxy.SetFlangeFrame(flange_frame));
  
  // Attach Fiducial
  EXPECT_TRUE(proxy.AttachFiducialToActiveObserver());
  
  // Get Hubs (arg)
  const std::string hub_config_topic = "/GetHubConfig";
  ros::ServiceClient hub_config_service =
      nh_.serviceClient<rtr_msgs::GetHubConfig>(hub_config_topic);
  rtr_msgs::GetHubConfig hub_config_srv;
  hub_config_srv.request.project_name = active_observer;
  hub_config_srv.request.hub_name = hub_name;
  rtr::JointConfiguration joint_config;

  EXPECT_TRUE(hub_config_service.call(hub_config_srv));
  joint_config = rtr::JointConfiguration(hub_config_srv.response.joint_config);
  
  // Teleport to Hub
  std::string teleport_topic = fmt::format("/{}/teleport_robot", active_observer);
  ros::ServiceClient teleport_service = nh_.serviceClient<rtr_msgs::TeleportRobot>(teleport_topic);
  rtr_msgs::TeleportRobot teleport_srv;
  teleport_srv.request.joint_config = joint_config.GetData();
  EXPECT_TRUE(teleport_service.call(teleport_srv) && teleport_srv.response.is_success);
  
  std::set<std::string> cameras_set = proxy.GetConnectedSensors();
  std::vector<std::string> uids(cameras_set.begin(), cameras_set.end());

  AllSensorData pre_calibration_data = proxy.GetCalibration();
  for (const std::string& uid : uids) {
    EXPECT_TRUE(proxy.EnableCalibrator(uid));
  }
  std::vector<std::string> failed_uids;
  EXPECT_TRUE(proxy.UseExistingCalibrationForInitGuess(uids, failed_uids));
  
  EXPECT_TRUE(failed_uids.empty());
  EXPECT_TRUE(proxy.SetCalibrateMode());
  EXPECT_TRUE(!proxy.CalibrateAllCameras());
  EXPECT_TRUE(!proxy.SetConfigureMode());
  AllSensorData post_calibration_data = proxy.GetCalibration();
  EXPECT_TRUE(pre_calibration_data.FuzzyEquals(post_calibration_data, .025f));
  
  RTR_INFO("Calibration Test Successful!");
}