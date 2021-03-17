#include <set>
#include <string>
#include <vector>

#include <QApplication>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/service.h>

#include <rtr_msgs/GetHubConfig.h>
#include <rtr_msgs/TeleportRobot.h>
#include <rtr_perc_sensors/SensorCalibrationData.hpp>
#include <rtr_utils/Logging.hpp>

#include "rtr_appliance/Appliance.hpp"
#include "rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp"
#include "rtr_perc_rapidsense_ros/Record.hpp"
#include "rtr_test_harness/RapidSenseTestHarnessServer.hpp"
#include "rtr_test_harness/RapidSenseTestHelper.hpp"

using namespace rtr::perception;
using namespace rtr;
namespace bfs = boost::filesystem;

const std::string appliance_dir = "/tmp/appliance_test";
const std::string rapidsense_dir = "/tmp/rapidsense_test";

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("rapidsense_calibration");

  ros::init(argc, argv, "CalibrationTest");
  RapidSenseTestHarnessServer server;
  if (!server.SetUp(appliance_dir)) {
    RTR_ERROR("Failed to setup test server");
    return EXIT_FAILURE;
  }

  ::testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();

  server.Teardown();
  // TODO Do not hard code. The appliance/rapidsense may end up using different
  // directories during runtime.
  bfs::remove_all(appliance_dir);
  bfs::remove_all(rapidsense_dir);
  return res;
}

class CalibrationTestFixture : public ::testing::Test {
 protected:
  ros::NodeHandle nh_;
  RapidSenseFrontEndProxy proxy_;
  RapidSenseTestHelper appliance_;
  std::string decon_group_name, robot_name, flange_frame, hub_name, project, rapidsense_data,
      robot_param;

  void SetUp() override {
    nh_.param<std::string>("decon_group_name", decon_group_name, "alphabot_calibration_test");
    nh_.param<std::string>("robot_name", robot_name, "alphabot");
    nh_.param<std::string>("flange_frame", flange_frame, "alphabot_rtr_flange");
    nh_.param<std::string>("hub_name", hub_name, "h3");
    nh_.param<std::string>("project", project, "../../");
    nh_.param<std::string>("rapidsense_data", rapidsense_data, "../../");

    std::string pkg_path = ros::package::getPath("reg_test_calibration");
    project = pkg_path + "/../../test_data/alphabot_calibration_test/alphabot.zip";
    robot_param = pkg_path + "/../../test_data/alphabot_calibration_test/alphabot.json";
    rapidsense_data = pkg_path + "/../../test_data/alphabot_calibration_test/rapidsense_data/";
    RTR_INFO("Value of project={}", project);
    RTR_INFO("Value of rapidsense_data={}", rapidsense_data);

    std::string rapidsense_state_directory;
    if (!proxy_.GetStateDirectory(rapidsense_state_directory)) {
      RTR_ERROR("Unable to get state directory from rapidsense");
    }

    std::string rapidsense_data_directory =
        fmt::format("{}/{}/", rapidsense_state_directory, decon_group_name);
    RTR_WARN("Copying files from {} to {}", rapidsense_data, rapidsense_data_directory);
    CopyFolder(rapidsense_data, rapidsense_data_directory);

    ASSERT_TRUE(appliance_.InstallProject(project));
    RTR_INFO("Project installed");
    ASSERT_TRUE(appliance_.SetProjectRobotParam("alphabot", robot_param));
    RTR_INFO("Robot params updated");
    ASSERT_TRUE(appliance_.AddAllProjectsToDeconGroup(decon_group_name));
    ASSERT_TRUE(appliance_.SetVisionEnabled(decon_group_name, true));
    ASSERT_TRUE(appliance_.LoadGroup(decon_group_name));
    RTR_INFO("Deconfliction group loaded");
  }

  void TearDown() override {
    RTR_INFO("TEST TEARDOWN");

    std::string rapidsense_state_directory;
    if (!proxy_.GetStateDirectory(rapidsense_state_directory)) {
      RTR_ERROR("Unable to get state directory from rapidsense");
    }

    // If the proxy cannot get the state directory, that means it rapidsense
    // will use the default path in /var/lib/rtr_spatial_perception. That
    // will still need to be removed for proper teardown
    std::string rapidsense_data_directory =
        fmt::format("{}/{}/", rapidsense_state_directory, decon_group_name);
    bfs::remove_all(rapidsense_data_directory);
  }

 public:
  CalibrationTestFixture()
      : nh_(""), proxy_(RapidSenseFrontEndProxy::ProxyHost::RAPIDSENSE_GUI), appliance_(nh_) {}
};

TEST_F(CalibrationTestFixture, VerifyCailbrationWorkflowWithPreviousLoc) {
  // Clear RobotManager and update robot observers
  proxy_.RefreshAllRobots();

  // Check system is LIVE and go into CONFIG
  EXPECT_EQ(proxy_.GetHealth().input_mode, RapidSenseInputMode::LIVE);
  if (proxy_.GetState() != RapidSenseState::CONFIGURE) {
    ASSERT_TRUE(proxy_.SetConfigureMode());
  }

  // Configure the Robot for Calibration
  // Set Active Observer (arg)
  std::string active_observer = robot_name;
  ASSERT_TRUE(proxy_.SetActiveRobotObserver(active_observer));

  // Set Flange Frame (arg)
  ASSERT_TRUE(proxy_.SetFlangeFrame(flange_frame));

  // Attach Fiducial
  ASSERT_TRUE(proxy_.AttachFiducialToActiveObserver());

  // Get Connected Sensors
  std::set<std::string> cameras_set = proxy_.GetConnectedSensors();
  std::vector<std::string> uids(cameras_set.begin(), cameras_set.end());

  AllSensorData pre_calibration_data = proxy_.GetCalibration();

  // CalibrationManager - get sensor instance from SensorManager
  for (const std::string& uid : uids) {
    ASSERT_TRUE(proxy_.EnableCalibrator(uid));
  }

  // CalibrationManager - Set init guess from extrinsics data
  std::vector<std::string> failed_uids;
  ASSERT_TRUE(proxy_.UseExistingCalibrationForInitGuess(uids, failed_uids));
  ASSERT_TRUE(failed_uids.empty());
  // If failed_uids exists besides the contents of test data, then
  // unanticipiated sensors may be connected to the workstation running this
  // test. Disconnect those sensors from the workstation.

  // Set to calibration mode and run calibration
  ASSERT_TRUE(proxy_.SetCalibrateMode());
  RTR_INFO("Set to calibration mode");
  ASSERT_TRUE(proxy_.CalibrateAllCameras());
  RTR_INFO("Calibrated all cameras");

  // Set to CONFIG mode and check that pre and post calibration data matches.
  // There are many commonly occuring scenarios that produce high deltas.
  // Consider disabling.
  EXPECT_TRUE(proxy_.SetConfigureMode());
  AllSensorData post_calibration_data = proxy_.GetCalibration();
  EXPECT_TRUE(pre_calibration_data.FuzzyEquals(post_calibration_data, .04f));
  RTR_INFO("Calibration Test Successful!");
}
