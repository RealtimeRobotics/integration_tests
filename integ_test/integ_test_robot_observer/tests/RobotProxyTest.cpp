// Copyright (C) 2019 Realtime Robotics

#include <QApplication>

#include <gtest/gtest.h>

#include <rtr_perc_spatial/PerceptionTestUtils.hpp>
#include <rtr_perc_spatial/RobotProxy.hpp>

using namespace rtr;
using namespace rtr::perception;

// the other RobotProxy tests are in rtr_perc_spatial, but this one relies on IK
// and cannot be run with rapidplan-ci tests (flaky if not provided enough
// resources)
TEST(RobotProxy, CalibrationIKCalls) {
  if (getenv("DISABLE_RS_FLAKYTESTS")) {
    RTR_INFO("Skipping flaky test RobotProxy::CalibrationIKCalls");
    return;
  }

  RobotProxy::Ptr proxy = testutils::CreateRobotProxy(testutils::UR3_MODEL_NAME);
  ASSERT_TRUE(proxy);

  std::vector<std::string> tool_links =
      proxy->GetRobot()->GetPossibleToolLinks(proxy->GetRootFrame());
  for (const auto& link : tool_links) {
    proxy->SetFlangeFrame(link);
    if (link.find("rtr_flange") != std::string::npos) {
      break;
    }
  }
  const std::string flange_frame = proxy->GetFlangeFrame();

  //// Test IK
  // check that IKManager can be created properly
  EXPECT_TRUE(proxy->InitCollisionChecker());
  EXPECT_TRUE(proxy->GetIKManager());

  JointConfiguration config;
  // check a pose in reach
  Pose test_pose(Mat3::Identity(), Vec3(0.25f, 0.25f, 0.25f));
  IKManager::Ptr ik_manager = proxy->GetIKManager();
  EXPECT_TRUE(
      ik_manager->Solve(test_pose, JointConfiguration({0.f, 0.f, 0.f, 0.f, 0.f, 0.f}), config));
  proxy->SetCurrentJointConfiguration(config);
  EXPECT_TRUE(test_pose.FuzzyEquals(proxy->GetLinkPose(flange_frame)));

  // check a pose out of reach
  test_pose = Pose(Mat3::Identity(), Vec3(2.5f, 2.5f, 2.5f));
  EXPECT_FALSE(
      ik_manager->Solve(test_pose, JointConfiguration({0.f, 0.f, 0.f, 0.f, 0.f, 0.f}), config));
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
