// Copyright (C) 2019 Realtime Robotics

#include <gtest/gtest.h>
#include <ros/package.h>

#include <rtr_perc_rapidsense_ros/RobotObserverManager.hpp>
#include <rtr_utils/ZipUtils.hpp>

#include "Common.hpp"

using namespace rtr;
using namespace rtr::perception;
namespace bfs = boost::filesystem;

const std::string PROJECT_PATH = "/tmp/RTRProjects";

RapidSenseRobotConfig CreateRobotConfig(const std::string& safe_name,
                                        const std::string& package_path) {
  RapidSenseRobotConfig config;
  config.name = safe_name;
  config.safe_name = safe_name;
  config.project_path = fmt::format("{}/{}", package_path, safe_name);
  RosController::TopicNames names = RosController::GetTopicNames(safe_name);
  config.joint_topic = names.joint_states;
  config.status_topic = names.robot_status;
  config.joint_path_topic = names.follow_joint_path;
  config.setup_rtr_topic = names.setup_rtr;
  config.acquire_control_topic = names.acquire_control;
  config.release_control_topic = names.release_control;
  return config;
}

void CompareRobotConfigs(const RapidSenseRobotConfig& a, const RapidSenseRobotConfig& b) {
  EXPECT_EQ(a.name, b.name);
  EXPECT_EQ(a.safe_name, b.safe_name);
  EXPECT_EQ(a.project_path, b.project_path);
  EXPECT_EQ(a.joint_topic, b.joint_topic);
  EXPECT_EQ(a.status_topic, b.status_topic);
  EXPECT_EQ(a.setup_rtr_topic, b.setup_rtr_topic);
  EXPECT_EQ(a.acquire_control_topic, b.acquire_control_topic);
  EXPECT_EQ(a.release_control_topic, b.release_control_topic);
}

TEST(RobotObserverManager, APICalls) {
  if (bfs::exists(PROJECT_PATH)) {
    bfs::remove_all(PROJECT_PATH);
  }
  ASSERT_TRUE(bfs::create_directories(PROJECT_PATH));

  // Setup ROS
  ros::NodeHandle nh("");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Set up projects
  const std::vector<std::string> project_names({"otto", "reggie", "squid", "twister"});
  const std::string package_path =
      ros::package::getPath("integ_test_robot_observer") + "/../../test_data/rocket_power";
  std::map<std::string, RapidSenseRobotConfig> config_map;
  for (const auto& name : project_names) {
    ASSERT_TRUE(zip::Unzip(fmt::format("{}/{}.zip", package_path, name), PROJECT_PATH));
    config_map[name] = CreateRobotConfig(name, PROJECT_PATH);
  }

  RuntimeRobotProjects projects;
  projects.group_name = "group_name";
  projects.robots.push_back(config_map.at("otto"));

  RobotObserverManager manager(nh);
  EXPECT_TRUE(manager.GetRobotMap().empty());

  // Create publishers
  std::map<std::string, ros::Publisher> joint_pubs, status_pubs;
  for (const auto& config_pair : config_map) {
    joint_pubs[config_pair.first] =
        nh.advertise<sensor_msgs::JointState>(config_pair.second.joint_topic, 1);
    status_pubs[config_pair.first] =
        nh.advertise<rtr_control_ros::RobotStatus>(config_pair.second.status_topic, 1);
  }

  // Allow the RobotObservers to initialize
  std::atomic_bool cancel(false);
  std::thread publish_thread([&]() {
    while (!cancel) {
      {
        for (const auto& pub_pair : joint_pubs) {
          const std::string& robot = pub_pair.first;
          joint_pubs[robot].publish(CreateJointStateMessage(Vec(6, 0.f)));
          status_pubs[robot].publish(
              CreateRobotStatusMessage(robot_manager::State::kConnected, ""));
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  //// Test single robot is added properly
  EXPECT_TRUE(manager.UpdateRobots(projects));
  EXPECT_EQ(manager.GetRobotObservers().size(), 1u);
  EXPECT_TRUE(manager.AllRobotsOk());
  EXPECT_TRUE(manager.GetFaultedRobots().empty());
  RuntimeRobotProjects projects_out = manager.GetRuntimeRobotProjects();
  EXPECT_EQ(projects.group_name, projects_out.group_name);
  EXPECT_EQ(projects.robots.size(), projects_out.robots.size());
  CompareRobotConfigs(projects.robots.front(), projects_out.robots.front());
  EXPECT_EQ(projects.robots.front().name, manager.GetRobotObservers().front()->GetName());

  //// Test that the update works correctly
  projects.robots.push_back(config_map["reggie"]);
  EXPECT_TRUE(manager.UpdateRobots(projects));
  EXPECT_EQ(manager.GetRobotObservers().size(), 2u);
  RobotObserverManager::RobotMap robot_map = manager.GetRobotMap();
  for (const auto& robot_config : projects.robots) {
    EXPECT_TRUE(robot_map.count(robot_config.name));
  }

  //// Test the clear and update
  manager.Clear();
  EXPECT_TRUE(manager.GetRobotMap().empty());
  projects.robots.push_back(config_map["squid"]);
  projects.robots.push_back(config_map["twister"]);
  EXPECT_TRUE(manager.UpdateRobots(projects));
  EXPECT_EQ(manager.GetRobotObservers().size(), 4u);
  robot_map = manager.GetRobotMap();
  for (const auto& robot_config : projects.robots) {
    ASSERT_TRUE(robot_map.count(robot_config.name));
  }

  //// Test state space functionality
  const RobotObserver::Ptr otto = robot_map["otto"];
  const std::string state_space_id = otto->GetStateSpaceUUIDFromName("state2");
  EXPECT_TRUE(manager.SetStateSpace("otto", state_space_id));
  EXPECT_EQ("state2", otto->GetStateSpaceName());

  //// Test set and get active robot observer
  EXPECT_TRUE(manager.SetActiveRobotObserver("otto"));
  EXPECT_EQ(manager.GetActiveRobotObserver()->GetName(), "otto");

  //// Test flange frame
  const std::vector<std::string> flange_frames = manager.GetPossibleFlangeFrames();
  EXPECT_FALSE(flange_frames.empty());
  const std::string default_flange = manager.ResetActiveObserverToDefaultFlangeFrame();
  for (const auto& frame : flange_frames) {
    if (frame != default_flange) {
      EXPECT_TRUE(manager.SetFlangeFrameOnActiveObserver(frame));
      EXPECT_EQ(frame, manager.GetFlangeFrameOnActiveObserver());
    }
  }
  manager.ResetActiveObserverToDefaultFlangeFrame();
  EXPECT_EQ(default_flange, manager.GetFlangeFrameOnActiveObserver());

  //// Test fiducial attachment
  EXPECT_TRUE(manager.AttachFiducialToActiveObserver());
  Robot::LinkMap link_map = otto->GetRobot()->GetLinkMapConst();
  EXPECT_TRUE(link_map.count(FIDUCIAL_FRAME));
  EXPECT_TRUE(link_map.count(FIDUCIAL_ORIGIN_FRAME));

  // Kill the initializer thread, so we can set different robot statuses
  cancel = true;
  if (publish_thread.joinable()) {
    publish_thread.join();
  }

  //// Test robot proxy creation (with a basic call to the proxy - not retesting full proxy)
  RosController::Ptr controller = CreateRosController(testutils::UR10_MODEL_NAME, "otto");
  RobotProxy::Ptr proxy = manager.GenerateRobotProxy();
  EXPECT_TRUE(proxy->InitCollisionChecker());
  EXPECT_TRUE(proxy->IsNodeInCollision(Vec(6, 0.f)));
  EXPECT_FALSE(proxy->IsNodeInCollision(Vec({0.f, 0.f, -2.25f, 0.f, 0.f, 0.f})));
  controller->Shutdown();

  //// Test robot status functionality
  rtr_control_ros::RobotStatus status_msg =
      CreateRobotStatusMessage(robot_manager::State::kConnected, "");
  const RobotObserver::Ptr reggie = robot_map["reggie"];
  const RobotObserver::Ptr twister = robot_map["twister"];
  const RobotObserver::Ptr squid = robot_map["squid"];
  for (const auto& status_pair : status_pubs) {
    status_pair.second.publish(status_msg);
  }
  while (reggie->GetRobotStatus() != status_msg.state || squid->GetRobotStatus() != status_msg.state
         || otto->GetRobotStatus() != status_msg.state
         || twister->GetRobotStatus() != status_msg.state) {
    ros::spinOnce();
  }

  // add and test callback
  status_msg = CreateRobotStatusMessage(robot_manager::State::kDisconnecting, "");
  manager.AddRobotStatusChangedCallback(
      [&](const int old_status, const int current_status, const std::string&) {
        EXPECT_EQ(old_status, static_cast<int>(robot_manager::State::kConnected));
        EXPECT_EQ(current_status, status_msg.state);
      });

  // set error on robots
  status_pubs["reggie"].publish(status_msg);
  status_pubs["twister"].publish(status_msg);
  while (reggie->GetRobotStatus() != status_msg.state
         || twister->GetRobotStatus() != status_msg.state) {
    ros::spinOnce();
  }

  // check correct faulted robots
  EXPECT_FALSE(manager.AllRobotsOk());
  std::set<std::string> faulted_robots = manager.GetFaultedRobots();
  EXPECT_TRUE(faulted_robots.count("reggie"));
  EXPECT_TRUE(faulted_robots.count("twister"));

  bfs::remove_all(PROJECT_PATH);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "RobotObserverManagerTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
