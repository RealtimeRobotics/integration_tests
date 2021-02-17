// Copyright (C) 2019 Realtime Robotics

#include <QApplication>

#include <gtest/gtest.h>

#include <rtr_control_ros/RosController.hpp>
#include <rtr_perc_rapidsense_ros/RosRobotConnection.hpp>
#include <rtr_perc_spatial/PerceptionTestUtils.hpp>
#include <rtr_utils/Environment.hpp>

#include "Common.hpp"

using namespace rtr;
using namespace rtr::perception;

RapidSenseRobotConfig CreateRobotConfig(const RapidPlanProject::Ptr prj) {
  RapidSenseRobotConfig config;
  config.name = prj->GetName();
  config.safe_name = GetROSSafeTopicPrefix(prj->GetName(), prj->GetId());
  RosController::TopicNames names = RosController::GetTopicNames(config.safe_name);
  config.joint_topic = names.joint_states;
  config.status_topic = names.robot_status;
  config.joint_path_topic = names.follow_joint_path;
  config.setup_rtr_topic = names.setup_rtr;
  config.acquire_control_topic = names.acquire_control;
  config.release_control_topic = names.release_control;
  return config;
}

// fake joint states and status so observer can initialize (assumes observer pointer is already
// created)
void InitRosRobotObserver(ros::NodeHandle& nh, const RapidPlanProject::Ptr project,
                          const RobotObserver::Ptr observer, const RapidSenseRobotConfig& config) {
  // Create publishers
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>(config.joint_topic, 1);
  ros::Publisher status_pub = nh.advertise<rtr_control_ros::RobotStatus>(config.status_topic, 1);

  // Allow the RobotObserver to initialize
  std::atomic_bool cancel(false);
  std::thread publish_thread([&]() {
    while (!cancel) {
      joint_pub.publish(CreateJointStateMessage(Vec(6, 0.f)));
      status_pub.publish(CreateRobotStatusMessage(robot_manager::State::kConnected, ""));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  // Create RosRobotObserver
  RobotConnection::Ptr connection = RosRobotConnection::MakePtr(nh, config);
  ASSERT_TRUE(observer->Init(project, connection, false, 5));

  // Kill the initializer thread
  cancel = true;
  if (publish_thread.joinable()) {
    publish_thread.join();
  }
}

TEST(RosRobotObserverTest, ROSTopics) {
  // Create project
  ros::NodeHandle nh("");
  const std::string robot_type = testutils::MELCO_MODEL_NAME;
  RapidPlanProject::Ptr project = testutils::CreateRapidPlanProject(robot_type);
  RapidSenseRobotConfig config = CreateRobotConfig(project);

  // Create RosRobotObserver
  RobotObserver::Ptr observer = RobotObserver::MakePtr();
  InitRosRobotObserver(nh, project, observer, config);

  //// Test joint states
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>(config.joint_topic, 1);

  // refresh spinner until at current joint config
  JointConfiguration test_config(6, 1.f);
  joint_pub.publish(CreateJointStateMessage(test_config));
  while (!observer->GetCurrentJointConfiguration().FuzzyEquals(test_config)) {
    ros::spinOnce();
  }

  JointConfiguration previous_config = test_config;
  for (const auto& config : testutils::MELCO_JOINTS) {
    // publish joint states
    sensor_msgs::JointState joint_msg = CreateJointStateMessage(config);
    joint_pub.publish(joint_msg);

    // check that the are received and returned properly
    JointConfiguration current_config =
        observer->GetJointConfiguration(RosTimeToSystemTime(joint_msg.header.stamp));
    while (current_config.FuzzyEquals(previous_config)) {
      current_config = observer->GetJointConfiguration(RosTimeToSystemTime(joint_msg.header.stamp));
      ros::spinOnce();
    }
    EXPECT_TRUE(current_config.FuzzyEquals(config));
    previous_config = current_config;
  }

  //// Test robot status
  ros::Publisher status_pub = nh.advertise<rtr_control_ros::RobotStatus>(config.status_topic, 1);

  // refresh spinner until at current joint config
  rtr_control_ros::RobotStatus status_msg1 =
      CreateRobotStatusMessage(robot_manager::State::kInitialized, "");
  status_pub.publish(status_msg1);
  while (observer->GetRobotStatus() != status_msg1.state) {
    ros::spinOnce();
  }

  // publish robot status
  rtr_control_ros::RobotStatus status_msg2 =
      CreateRobotStatusMessage(robot_manager::State::kConnected, "");
  status_pub.publish(status_msg2);

  // check that they are received and returned properly
  while (observer->GetRobotStatus() == status_msg1.state) {
    ros::spinOnce();
  }
  EXPECT_EQ(status_msg2.state, observer->GetRobotStatus());

  // check that the status callback functionality works
  rtr_control_ros::RobotStatus status_msg3 =
      CreateRobotStatusMessage(robot_manager::State::kDisconnecting, "state string");
  observer->AddRobotStatusChangedCallback(
      [&](const int old_status, const int current_status, const std::string& state_str) {
        EXPECT_EQ(old_status, status_msg2.state);
        EXPECT_EQ(current_status, status_msg3.state);
        EXPECT_TRUE(state_str.find(status_msg3.state_str) != std::string::npos);
      });
  status_pub.publish(status_msg3);
  while (observer->GetRobotStatus() == status_msg2.state) {
    ros::spinOnce();
  }

  // check that the status callback is removed
  observer->RemoveRobotStatusChangedCallback();
  rtr_control_ros::RobotStatus status_msg4 =
      CreateRobotStatusMessage(robot_manager::State::kInitialized, "");
  status_pub.publish(status_msg4);

  // check that the status is still received correctly
  while (observer->GetRobotStatus() == status_msg3.state) {
    ros::spinOnce();
  }
  EXPECT_EQ(status_msg4.state, observer->GetRobotStatus());
}

TEST(RosRobotProxyTest, RobotControllerInteractions) {
  // Create project
  ros::NodeHandle nh("");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  const std::string robot_type = testutils::MELCO_MODEL_NAME;
  RapidPlanProject::Ptr project = testutils::CreateRapidPlanProject(robot_type);
  RapidSenseRobotConfig config = CreateRobotConfig(project);

  // Create RosController
  RosController::Ptr controller = CreateRosController(robot_type, config.safe_name);
  ASSERT_TRUE(controller);
  EXPECT_EQ(controller->GetDetailedState().state, robot_manager::State::kConnected);

  // Create RosRobotProxy
  RobotProxy::Ptr proxy = RobotProxy::MakePtr();
  InitRosRobotObserver(nh, project, proxy, config);

  //// Test SetupRTR
  EXPECT_EQ(controller->GetDetailedState().state, robot_manager::State::kConnected);

  //// Test AcquireControl
  EXPECT_TRUE(proxy->AcquireControl());
  // TODO: Robot controller does not actually register as active (inner robot
  // interface will be active). Still waiting to hear back from robotics team on
  // this, but it seems like they're in the middle of a refactor
  // EXPECT_EQ(controller->GetDetailedState().state,
  // robot_manager::State::ACTIVE);
  EXPECT_FALSE(controller->HasError());

  //// Test moving the robot
  std::vector<JointConfiguration> joints({Vec(6, 0.f), Vec(6, 1.5f)});
  EXPECT_TRUE(proxy->SendJointPath(joints));
  EXPECT_TRUE(joints.back().FuzzyEquals(proxy->GetCurrentJointConfiguration()));

  // check halt behavior - note that this test could flake if the halt_thread
  // gets starved
  if (!getenv("DISABLE_RS_FLAKYTESTS")) {
    joints = std::vector<JointConfiguration>({Vec(6, 1.5f), Vec(6, 0.f)});
    std::atomic_bool cancelled(false);
    std::thread halt_thread([&proxy, &cancelled]() {
      while (proxy->GetCurrentJointConfiguration()[0] > 1.f) {
        if (cancelled) {
          EXPECT_FALSE(cancelled);
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      proxy->Halt();
    });
    EXPECT_TRUE(proxy->SendJointPath(joints));
    cancelled = true;  // make sure halt_thread cannot get stuck
    if (halt_thread.joinable()) {
      halt_thread.join();
    }
    EXPECT_FALSE(joints.back().FuzzyEquals(proxy->GetCurrentJointConfiguration()));
  } else {
    RTR_INFO("Skipping flaky test RosRobotObserver::Halt");
  }

  //// Test ReleaseControl
  EXPECT_TRUE(proxy->ReleaseControl());
  EXPECT_FALSE(controller->HasError());
  // TODO: Robot controller does not return true because it expects to be in
  // HANDOFF CONNECTED). Still waiting to hear back from robotics team on this,
  // but it seems like they're in the middle of a refactor

  proxy->Shutdown();
  proxy.reset();
  controller->Shutdown();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "RosRobotObserverTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
