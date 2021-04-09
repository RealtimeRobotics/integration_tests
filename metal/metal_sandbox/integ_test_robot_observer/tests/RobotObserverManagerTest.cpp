#include <set>
#include <string>
#include <vector>

#include <QApplication>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/service.h>

#include <rtr_appliance/Appliance.hpp>
#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
#include <rtr_perc_rapidsense_ros/Record.hpp>
#include <rtr_perc_rapidsense_ros/RobotObserverManager.hpp>
#include <rtr_test_harness/RapidSenseTestHelper.hpp>
#include <rtr_utils/Logging.hpp>

#include "Common.hpp"

using namespace rtr::perception;
using namespace rtr;
namespace bfs = boost::filesystem;

const std::string appliance_dir = "/tmp/appliance_test";
const std::string rapidsense_dir = "/tmp/rapidsense_test";

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("robot_observer_manager");

  ros::init(argc, argv, "RobotObserverManagerTest");

  ::testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();

  server.Teardown();

  bfs::remove_all(appliance_dir);
  bfs::remove_all(rapidsense_dir);
  return res;
}

class RobotObserverManagerTestFixture : public ::testing::Test {
 protected:
 protected:
  ros::NodeHandle nh_;
  RapidSenseFrontEndProxy proxy_;
  RapidSenseTestHelper appliance_;
  std::string decon_group_name_, robot_name_, flange_frame_;
  std::string default_state_space_, other_state_space_;
  std::string start_hub_name_, target_hub_name_;
  const std::vector<std::string> project_names_{"otto", "reggie"};

  void SetUp() override {
    appliance_.WaitForApplianceServer();
    appliance_.StartSensorSimulator();
    appliance_.CheckRapidSenseServerState(RapidSenseState::IDLE);

    // get test parameters
    std::string rapidsense_data;
    const std::string package_path =
        ros::package::getPath("integ_test_robot_observer") + "/../../test_data/rocket_power/";
    nh_.param<std::string>("decon_group_name", decon_group_name_, "rocket_power");

    nh_.param<std::string>("rapidsense_data", rapidsense_data,
                           ros::package::getPath("integ_test_robot_observer")
                               + "/../../test_data/rocket_power/rapidsense_data");

    // get state directory and copy calibration and config data into directory
    std::string rapidsense_state_directory;
    if (!proxy_.GetStateDirectory(rapidsense_state_directory)) {
      RTR_ERROR("Unable to get state directory from rapidsense");
    }

    std::string rapidsense_data_directory =
        fmt::format("{}/{}/", rapidsense_state_directory, decon_group_name_);
    CopyFolder(rapidsense_data, rapidsense_data_directory);

    ASSERT_TRUE(appliance_.ClearApplianceDatabase());
    for (auto& project : project_names_) {
      const std::string project_path = package_path + project + ".zip";
      ASSERT_TRUE(appliance_.InstallProject(project_path));
      const std::string robot_param = package_path + project + ".json";
      ASSERT_TRUE(appliance_.SetProjectRobotParam(project, robot_param));
    }
    ASSERT_TRUE(appliance_.AddAllProjectsToDeconGroup(decon_group_name_));
    ASSERT_TRUE(appliance_.SetVisionEnabled(decon_group_name_, true));
    ASSERT_TRUE(appliance_.LoadGroup(decon_group_name_));
    for (auto& project : project_names_) {
      ASSERT_TRUE(appliance_.InitGroup(decon_group_name_, project, "default_state"));
    }

    std_srvs::Trigger trg;
    CallRosService<std_srvs::Trigger>(nh_, trg, "/restart_sim");
  }

  void TearDown() override {}

 public:
  RobotObserverManagerTestFixture()
      : nh_(""), proxy_(RapidSenseFrontEndProxy::ProxyHost::RAPIDSENSE_GUI), appliance_(nh_) {}
};

TEST_F(RobotObserverManagerTestFixture, CheckRobotObserverManager) {
  // Clear RobotManager and update robot observers
  proxy_.RefreshAllRobots();
  if (proxy_.GetState() != RapidSenseState::OPERATION) {
    EXPECT_TRUE(proxy_.SetOperationMode());
  }

  RobotObserverManager::Ptr manager;
  proxy_.GetRobotObserverManager(manager);
  EXPECT_EQ(manager->GetRobotMap().size(), 2u);
  EXPECT_TRUE(manager->SetActiveRobotObserver("otto"));
  proxy_.SetRobotObserverManager(manager);
  EXPECT_EQ(appliance_.BeginOperationMode(), ExtCode::SUCCESS);

  EXPECT_EQ(manager->GetRobotObservers().size(), 2u);
  EXPECT_TRUE(manager->AllRobotsOk());
  EXPECT_TRUE(manager->GetFaultedRobots().empty());
  RuntimeRobotProjects projects_out = manager->GetRuntimeRobotProjects();
  EXPECT_EQ(projects_out.group_name, "");
  EXPECT_EQ(projects_out.robots.size(), 2u);
  EXPECT_EQ(manager->GetRobotObservers().front()->GetName(), "otto");

  EXPECT_TRUE(manager->SetActiveRobotObserver("reggie"));
  EXPECT_EQ(manager->GetActiveRobotObserver()->GetName(), "reggie");

  //// Test the clear and update
  manager->Clear();
  EXPECT_TRUE(manager->GetRobotMap().empty());
  EXPECT_TRUE(manager->UpdateRobots(projects_out));
  EXPECT_EQ(manager->GetRobotObservers().size(), 2u);
  auto robot_map = manager->GetRobotMap();
  for (const auto& project : project_names_) {
    ASSERT_TRUE(robot_map.count(project));
  }

  //// Test state space functionality
  const RobotObserver::Ptr otto = robot_map["otto"];
  const std::string state_space_id = otto->GetStateSpaceUUIDFromName("state2");
  EXPECT_TRUE(manager->SetStateSpace("otto", state_space_id));
  EXPECT_EQ("state2", otto->GetStateSpaceName());

  //// Test flange frame
  const std::vector<std::string> flange_frames = manager->GetPossibleFlangeFrames();
  EXPECT_TRUE(flange_frames.empty());
  const std::string default_flange = manager->ResetActiveObserverToDefaultFlangeFrame();
  for (const auto& frame : flange_frames) {
    if (frame != default_flange) {
      EXPECT_TRUE(manager->SetFlangeFrameOnActiveObserver(frame));
      EXPECT_EQ(frame, manager->GetFlangeFrameOnActiveObserver());
    }
  }
  manager->ResetActiveObserverToDefaultFlangeFrame();
  EXPECT_EQ(default_flange, manager->GetFlangeFrameOnActiveObserver());

  //// Test set and get active robot observer
  EXPECT_TRUE(manager->SetActiveRobotObserver("otto"));
  EXPECT_EQ(manager->GetActiveRobotObserver()->GetName(), "otto");

  //// Test robot proxy - bare minimum. Separate test for everything
  RobotProxy::Ptr robot_proxy = manager->GenerateRobotProxy();
  ASSERT_NE(robot_proxy, nullptr);

  //// Test robot tatus and callback
  const RobotObserver::Ptr reggie = robot_map["reggie"];
  EXPECT_EQ(reggie->GetRobotStatus(), 3);
  EXPECT_EQ(otto->GetRobotStatus(), 3);
  rtr_control_ros::RobotStatus status_msg =
      CreateRobotStatusMessage(robot_manager::State::kDisconnecting, "");
  manager->AddRobotStatusChangedCallback(
      [&](const int old_status, const int current_status, const std::string&) {
        EXPECT_EQ(old_status, 3);
        EXPECT_EQ(current_status, status_msg.state);
      });

  //  Check correct faulted robots
  EXPECT_TRUE(manager->AllRobotsOk());
  std::set<std::string> faulted_robots = manager->GetFaultedRobots();
  EXPECT_TRUE(faulted_robots.empty());
}
