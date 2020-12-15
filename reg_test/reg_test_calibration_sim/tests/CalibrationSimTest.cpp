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
  bfs::remove_all(appliance_dir);
  bfs::remove_all(rapidsense_dir);
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("rapidsense_sim");

  ros::init(argc, argv, "CalibrationSimTest");
  RapidSenseTestHarnessServer server;
  std::string rs_path = ros::package::getPath("reg_test_calibration_sim") + "/../../test_data";
  if (!server.SetUpSim(rs_path)) {
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
    RTR_INFO("TEST SETUP");
    nh_.param<std::string>("decon_group_name", decon_group_name, "ur3_calibration_test");
    nh_.param<std::string>("robot_name", robot_name, "ur3");
    nh_.param<std::string>("flange_frame", flange_frame, "ur3/flange");
    nh_.param<std::string>("hub_name", hub_name, "h_3");
    nh_.param<std::string>("project", project, "../../");
    nh_.param<std::string>("rapidsense_data", rapidsense_data, "../../");

    project = ros::package::getPath("reg_test_calibration_sim")
              + "/../../test_data/ur3_calibration_test/ur3_november_25.zip";
    rapidsense_data = ros::package::getPath("reg_test_calibration_sim")
                      + "/../../test_data/ur3_calibration_test/rapidsense_data/";
    robot_param = ros::package::getPath("reg_test_calibration_sim")
                  + "/../../test_data/ur3_calibration_test/ur3.json";
    RTR_INFO("Value of project={}", project);
    RTR_INFO("Value of rapidsense_data={}", rapidsense_data);

    std::string rapidsense_state_directory;
    if (!proxy_.GetStateDirectory(rapidsense_state_directory)) {
      RTR_ERROR("Unable to get state directory from rapidsense");
    }

    std::string rapidsense_data_directory =
        fmt::format("{}/{}/", rapidsense_state_directory, decon_group_name);
    CopyFolder(rapidsense_data, rapidsense_data_directory);

    ASSERT_TRUE(appliance_.ClearApplianceDatabase());
    ASSERT_TRUE(appliance_.InstallProject(project));
    ASSERT_TRUE(appliance_.SetProjectRobotParam("ur3", robot_param));
    ASSERT_TRUE(appliance_.AddAllProjectsToDeconGroup(decon_group_name));
    ASSERT_TRUE(appliance_.SetVisionEnabled(decon_group_name, true));
    ASSERT_TRUE(appliance_.LoadGroup(decon_group_name));

    std_srvs::Trigger trg;
    CallRosService<std_srvs::Trigger>(nh_, trg, "/restart_sim");

    RTR_DEBUG("Waiting for restart sim");
    std::this_thread::sleep_for(std::chrono::seconds(4));
  }

  void TearDown() override {
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
  proxy_.RefreshAllRobots();

  EXPECT_EQ(proxy_.GetHealth().input_mode, RapidSenseInputMode::SIMULATION);
  if (proxy_.GetState() != RapidSenseState::CONFIGURE) {
    EXPECT_TRUE(proxy_.SetConfigureMode());
  }

  // Configure the Robot for Calibration
  // Set Active Observer (arg)
  std::string active_observer = robot_name;
  EXPECT_TRUE(proxy_.SetActiveRobotObserver(active_observer));

  // Set Flange Frame (arg)
  EXPECT_TRUE(proxy_.SetFlangeFrame(flange_frame));

  // Attach Fiducial
  EXPECT_TRUE(proxy_.AttachFiducialToActiveObserver());

  // Teleport to hub
  EXPECT_TRUE(appliance_.TeleportToHub(active_observer, hub_name));

  std::set<std::string> cameras_set = proxy_.GetConnectedSensors();
  std::vector<std::string> uids(cameras_set.begin(), cameras_set.end());

  AllSensorData pre_calibration_data = proxy_.GetCalibration();
  for (const std::string& uid : uids) {
    EXPECT_TRUE(proxy_.EnableCalibrator(uid));
  }
  std::vector<std::string> failed_uids;
  EXPECT_TRUE(proxy_.UseExistingCalibrationForInitGuess(uids, failed_uids));

  EXPECT_TRUE(failed_uids.empty());
  EXPECT_TRUE(proxy_.SetCalibrateMode());
  EXPECT_TRUE(proxy_.CalibrateAllCameras());
  EXPECT_TRUE(proxy_.SetConfigureMode());
  AllSensorData post_calibration_data = proxy_.GetCalibration();
  EXPECT_TRUE(pre_calibration_data.FuzzyEquals(post_calibration_data, .025f));

  RTR_INFO("Calibration Test Successful!");
}
