// Copyright (C) 2019 Realtime Robotics

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

#include <rtr_perc_spatial/MeshRobotMaskGenerator.hpp>
#include <rtr_perc_spatial/PerceptionTestUtils.hpp>
#include <rtr_perc_spatial/RobotFilter.hpp>
#include <rtr_perc_spatial/RobotMaskGenerator.hpp>

using namespace rtr;
using namespace rtr::perception;

void RunMaskTest(const RobotMaskGenerator::Type type, const std::string& robot_model,
                 const int fill_tolerance, const int match_tolerance,
                 const std::vector<JointConfiguration>& configs) {
  // Set up robot and environment
  RobotObserver::Ptr observer = testutils::CreateRobotObserver(robot_model);
  ASSERT_TRUE(observer);
  RobotMaskGenerator::MeshMap mesh_map;
  ASSERT_TRUE(testutils::CreateMeshMap(observer, mesh_map));
  VoxelRegionDescription vrd = testutils::CreateVoxelRegionDescription();

  // Limit resources unit test can take up
  tbb::task_arena arena;
  arena.initialize(std::max(1, int(0.25 * arena.max_concurrency())));

  // Create mask generator
  RobotMaskGenerator::Ptr generator =
      RobotFilter::CreateRobotMaskGenerator(type, mesh_map, observer, vrd, arena);
  ASSERT_TRUE(generator);
  generator->Init();

  EXPECT_TRUE(testutils::RobotFilterAccuracyTest(observer, mesh_map, vrd, generator, fill_tolerance,
                                                 match_tolerance, configs));
  generator->Shutdown();
}

// The other RobotMaskGenerator tests are kept in rtr_perc_spatial, but the OpenCL ones require
// resources which are not guaranteed to the rapidplan-ci stage
TEST(OpenCLRobotMaskGenerator, URTest) {
  const int fill_tolerance = 0;
  // Some surface details different between OpenCL and Collision
  const int match_tolerance = 1;
  RunMaskTest(RobotMaskGenerator::OPENCL, testutils::UR5_MODEL_NAME, fill_tolerance,
              match_tolerance, testutils::UR5_JOINTS);
}

TEST(OpenCLRobotMaskGenerator, FanucTest) {
  const int fill_tolerance = 0;
  // Some surface details different between OpenCL and Collision
  const int match_tolerance = 1;
  RunMaskTest(RobotMaskGenerator::OPENCL, testutils::FANUC_MODEL_NAME, fill_tolerance,
              match_tolerance, testutils::FANUC_JOINTS);
}

TEST(OpenCLRobotMaskGenerator, MelcoTest) {
  const int fill_tolerance = 0;
  // Some surface details different between OpenCL and Collision
  const int match_tolerance = 1;
  RunMaskTest(RobotMaskGenerator::OPENCL, testutils::MELCO_MODEL_NAME, fill_tolerance,
              match_tolerance, testutils::MELCO_JOINTS);
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
