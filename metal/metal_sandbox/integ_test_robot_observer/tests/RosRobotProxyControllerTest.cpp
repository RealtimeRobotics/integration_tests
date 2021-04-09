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
  QCoreApplication::setApplicationName("robotproxy_controller");

  ros::init(argc, argv, "RosRobotProxyControllerTest");
  RapidSenseTestHarnessServer server;
  if (!server.SetUpSim(appliance_dir)) {
    RTR_ERROR("Failed to setup test server");
    return EXIT_FAILURE;
  }

  ::testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();

  server.Teardown();

  bfs::remove_all(appliance_dir);
  bfs::remove_all(rapidsense_dir);
  return res;
}

class RosRobotProxyControllerTest : public ::testing::Test {
 protected:
 protected:
  ros::NodeHandle nh_;
  RapidSenseFrontEndProxy proxy_;
  RapidSenseTestHelper appliance_;
  std::string decon_group_name_, robot_name_, flange_frame_;
  std::string default_state_space_, other_state_space_;
  std::string start_hub_name_, target_hub_name_;

  void SetUp() override {
    // get test parameters
    nh_.param<std::string>("decon_group_name", decon_group_name_, "ur3_proxycontroller_test");
    nh_.param<std::string>("robot_name", robot_name_, "ur3");
    nh_.param<std::string>("default_state_space", default_state_space_, "default_state");
    nh_.param<std::string>("other_state_space", other_state_space_, "default_state_2");
    nh_.param<std::string>("flange_frame", flange_frame_, "ur3_rtr_flange");
    nh_.param<std::string>("start_hub_name", start_hub_name_, "point1");
    nh_.param<std::string>("target_hub_name", target_hub_name_, "point2");

    std::string project, rapidsense_data;
    nh_.param<std::string>("project", project,
                           ros::package::getPath("integ_test_robot_observer") +
                               "/../../test_data/ur3_proxycontroller_test/"
                               "ur3_proxycontroller_november_25.zip");
    nh_.param<std::string>("rapidsense_data", rapidsense_data,
                           ros::package::getPath("integ_test_robot_observer")
                               + "/../../test_data/ur3_proxycontroller_test/rapidsense_data");

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
    const std::string robot_param = ros::package::getPath("integ_test_robot_observer")
                                    + "/../../test_data/ur3_proxycontroller_test/ur3.json";
    ASSERT_TRUE(appliance_.SetProjectRobotParam("ur3", robot_param));
    ASSERT_TRUE(appliance_.AddAllProjectsToDeconGroup(decon_group_name_));
    ASSERT_TRUE(appliance_.SetVisionEnabled(decon_group_name_, true));
    ASSERT_TRUE(appliance_.LoadGroup(decon_group_name_));
    ASSERT_TRUE(appliance_.InitGroup(decon_group_name_, "ur3", "default_state"));

    // start simulated sensors and wait for voxel data to be published
    std_srvs::Trigger trg;
    CallRosService<std_srvs::Trigger>(nh_, trg, "/restart_sim");
  }

  void TearDown() override {}

 public:
  RosRobotProxyControllerTest()
      : nh_(""), proxy_(RapidSenseFrontEndProxy::ProxyHost::RAPIDSENSE_GUI), appliance_(nh_) {}
};

TEST_F(RosRobotProxyControllerTest, VerifyRobotProxyController) {
  // Clear RobotManager and update robot observers
  proxy_.RefreshAllRobots();
  if (proxy_.GetState() != RapidSenseState::OPERATION) {
    EXPECT_TRUE(proxy_.SetOperationMode());
  }
  RobotObserverManager::Ptr manager;
  proxy_.GetRobotObserverManager(manager);
  EXPECT_TRUE(manager->SetActiveRobotObserver("ur3"));
  EXPECT_TRUE(manager->SetFlangeFrameOnActiveObserver("ur3_rtr_flange"));
  RobotProxy::Ptr robot_proxy = manager->GenerateRobotProxy();

  ASSERT_NE(robot_proxy, nullptr);
  EXPECT_EQ(appliance_.BeginOperationMode(), ExtCode::SUCCESS);
  EXPECT_TRUE(robot_proxy->AcquireControl());

  if (proxy_.GetState() != RapidSenseState::CONFIGURE) {
    ASSERT_TRUE(proxy_.SetConfigureMode());
  }
  ASSERT_TRUE(proxy_.SetCalibrateMode());
  std::vector<JointConfiguration> joints({Vec(6, 0.f), Vec(6, 1.5f)});
  EXPECT_TRUE(robot_proxy->SendJointPath(joints));
  EXPECT_TRUE(joints.back().FuzzyEquals(robot_proxy->GetCurrentJointConfiguration()));
  joints = std::vector<JointConfiguration>({Vec(6, 1.5f), Vec(6, 0.f)});
  EXPECT_TRUE(robot_proxy->SendJointPath(joints));
  EXPECT_TRUE(joints.back().FuzzyEquals(robot_proxy->GetCurrentJointConfiguration()));
  EXPECT_TRUE(robot_proxy->ReleaseControl());

  robot_proxy->Shutdown();
  robot_proxy.reset();
  RTR_INFO("RobotProxyControllerTest Successful!");
}
