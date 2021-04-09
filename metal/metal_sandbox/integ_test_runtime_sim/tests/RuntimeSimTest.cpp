#include <set>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/service.h>

#include <rtr_appliance/Appliance.hpp>
#include <rtr_msgs/GetHubConfig.h>
#include <rtr_msgs/TeleportRobot.h>
#include <rtr_perc_api/SensorFrame.hpp>
#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
#include <rtr_perc_rapidsense_ros/Record.hpp>
#include <rtr_perc_sensors/SensorCalibrationData.hpp>
#include <rtr_test_harness/RapidSenseTestHarnessServer.hpp>
#include <rtr_test_harness/RapidSenseTestHelper.hpp>
#include <rtr_utils/Logging.hpp>

using namespace rtr::perception;
using namespace rtr;
namespace bfs = boost::filesystem;

const std::string appliance_dir = "/tmp/appliance_test";
const std::string rapidsense_dir = "/tmp/rapidsense_test";

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("rapidsense_runtime_sim");

  ros::init(argc, argv, "RuntimeSimTest");
  RapidSenseTestHarnessServer server;

  ::testing::InitGoogleTest(&argc, argv);
  if (!server.SetUpSim(appliance_dir)) {
    RTR_ERROR("Failed to setup test server");
    return EXIT_FAILURE;
  }
  int res = RUN_ALL_TESTS();

  server.Teardown();
  bfs::remove_all(appliance_dir);
  bfs::remove_all(rapidsense_dir);
  return res;
}

class RuntimeSimTest : public ::testing::Test {
 protected:
  ros::NodeHandle nh_;
  RapidSenseFrontEndProxy proxy_;
  RapidSenseTestHelper appliance_;
  std::string decon_group_name_, robot_name_, flange_frame_;
  std::string default_state_space_, other_state_space_;
  std::string start_hub_name_, target_hub_name_;

  void SetUp() override {
    // get test parameters
    nh_.param<std::string>("decon_group_name", decon_group_name_, "ur3_runtime_test");
    nh_.param<std::string>("robot_name", robot_name_, "ur3");
    nh_.param<std::string>("default_state_space", default_state_space_, "default_state");
    nh_.param<std::string>("other_state_space", other_state_space_, "default_state_2");
    nh_.param<std::string>("flange_frame", flange_frame_, "ur3_rtr_flange");
    nh_.param<std::string>("start_hub_name", start_hub_name_, "point1");
    nh_.param<std::string>("target_hub_name", target_hub_name_, "point2");

    std::string project, rapidsense_data;
    nh_.param<std::string>("project", project,
                           ros::package::getPath("integ_test_runtime_sim") +
                               "/../../test_data/ur3_runtime_test/"
                               "ur3_runtime_test_november_25.zip");
    nh_.param<std::string>("rapidsense_data", rapidsense_data,
                           ros::package::getPath("integ_test_runtime_sim")
                               + "/../../test_data/ur3_runtime_test/rapidsense_data");

    // get state directory and copy calibration and config data into directory
    std::string rapidsense_state_directory;
    if (!proxy_.GetStateDirectory(rapidsense_state_directory)) {
      RTR_ERROR("Unable to get state directory from rapidsense");
    }

    std::string rapidsense_data_directory =
        fmt::format("{}/{}/", rapidsense_state_directory, decon_group_name_);
    CopyFolder(rapidsense_data, rapidsense_data_directory);

    // set up appliance data
    ASSERT_TRUE(appliance_.ClearApplianceDatabase());
    ASSERT_TRUE(appliance_.InstallProject(project));
    const std::string robot_param = ros::package::getPath("integ_test_runtime_sim")
                                    + "/../../test_data/ur3_runtime_test/ur3.json";
    ASSERT_TRUE(appliance_.SetProjectRobotParam("ur3", robot_param));
    ASSERT_TRUE(appliance_.AddAllProjectsToDeconGroup(decon_group_name_));
    ASSERT_TRUE(appliance_.SetVisionEnabled(decon_group_name_, true));
    ASSERT_TRUE(appliance_.LoadGroup(decon_group_name_));
    ASSERT_TRUE(appliance_.InitGroup(decon_group_name_, "ur3", "default_state"));

    // start simulated sensors and wait for voxel data to be published
    std_srvs::Trigger trg;
    CallRosService<std_srvs::Trigger>(nh_, trg, "/restart_sim");

    const SensorFrameType ft(perception::SensorFrameType::SENSOR_FRAME_VOXELS,
                             perception::SensorFrameType::ROBOT_SELF_FILTERED);
    while (!proxy_.GetFrame("voxel_stream", "", ft)) {
      RTR_WARN("Waiting for RapidSense server to begin publishing voxels");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RTR_INFO("Voxels are being published. Setup complete");
  }

  void TearDown() override {}

 public:
  RuntimeSimTest() : nh_(""), appliance_(nh_) {}
};

TEST_F(RuntimeSimTest, TestRuntimeMoveToHub) {
  if (getenv("DISABLE_RS_FLAKYTESTS")) {
    RTR_WARN("Skipping flaky test RuntimeSimTest::TestRuntimeMoveToHub");
    return;
  }

  // Verify setup
  EXPECT_TRUE(proxy_.IsServerConnected());
  proxy_.RefreshAllRobots();
  EXPECT_EQ(proxy_.GetHealth().input_mode, RapidSenseInputMode::SIMULATION);
  if (proxy_.GetState() != RapidSenseState::OPERATION) {
    EXPECT_TRUE(proxy_.SetOperationMode());
  }

  EXPECT_TRUE(appliance_.TeleportToHub(robot_name_, start_hub_name_));

  //// Test initializing appliance
  EXPECT_EQ(appliance_.SetInterruptBehavior(robot_name_, 15, 5), ExtCode::SUCCESS);
  EXPECT_TRUE(appliance_.InitGroup(decon_group_name_, robot_name_, default_state_space_));
  EXPECT_EQ(appliance_.BeginOperationMode(), ExtCode::SUCCESS);

  // record first config
  const std::vector<RobotObserver::Ptr> observers = proxy_.GetObservers();
  ASSERT_FALSE(observers.empty());
  RobotObserver::Ptr observer = observers.front();
  JointConfiguration first_config = observer->GetCurrentJointConfiguration();

  //// Test move to hub
  ApplianceCommander::ExtCodeSeqPair res_pair =
      appliance_.MoveToHub(robot_name_, default_state_space_, target_hub_name_, 0.8f, 0.05);
  EXPECT_EQ(res_pair.first, ExtCode::SUCCESS);
  EXPECT_EQ(appliance_.WaitForMove(res_pair.second), ExtCode::SUCCESS);

  // double check that we actually moved
  JointConfiguration curr_config = observer->GetCurrentJointConfiguration();
  EXPECT_FALSE(first_config.FuzzyEquals(curr_config, 0.01));

  // move back to start configuration
  res_pair = appliance_.MoveToHub(robot_name_, default_state_space_, start_hub_name_, 0.8f, 0.05);
  EXPECT_EQ(res_pair.first, ExtCode::SUCCESS);
  EXPECT_EQ(appliance_.WaitForMove(res_pair.second), ExtCode::SUCCESS);

  // double check that we actually moved
  curr_config = observer->GetCurrentJointConfiguration();
  EXPECT_TRUE(first_config.FuzzyEquals(curr_config, 0.01));

  //// Test state space changes
  SensorFrameType ft(SensorFrameType::SENSOR_FRAME_VOXELS, SensorFrameType::ROBOT_SELF_FILTERED);
  SensorFrame::ConstPtr frame = proxy_.GetFrame("voxel_stream", "", ft, 0.5, 0.5);
  EXPECT_TRUE(frame);
  if (frame) {
    std::map<std::string, std::string> state_spaces =
        SensorFrameVoxels::CastConstPtr(frame)->GetStateSpaces();
    EXPECT_TRUE(state_spaces.count(observer->GetName()));
    EXPECT_EQ(state_spaces[observer->GetName()],
              observer->GetStateSpaceUUIDFromName(default_state_space_));
  }
  EXPECT_EQ(appliance_.ChangeWorkspace(robot_name_, other_state_space_), ExtCode::SUCCESS);
  frame = proxy_.GetFrame("voxel_stream", "", ft, 0.5, 0.5);
  EXPECT_TRUE(frame);
  if (frame) {
    std::map<std::string, std::string> state_spaces =
        SensorFrameVoxels::CastConstPtr(frame)->GetStateSpaces();
    EXPECT_TRUE(state_spaces.count(observer->GetName()));
    EXPECT_EQ(state_spaces[observer->GetName()],
              observer->GetStateSpaceUUIDFromName(other_state_space_));
  }

  // shutdown
  EXPECT_EQ(appliance_.EndOperationMode(), ExtCode::SUCCESS);
  EXPECT_EQ(appliance_.TerminateGroup(decon_group_name_), ExtCode::SUCCESS);
}
