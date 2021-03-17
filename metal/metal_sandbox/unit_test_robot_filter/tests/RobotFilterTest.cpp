#include <gtest/gtest.h>

#include "rtr_perc_spatial/PerceptionTestUtils.hpp"
#include "rtr_perc_spatial/RobotClusterFilter.hpp"
#include "rtr_perc_spatial/RobotFilter.hpp"
#include "rtr_perc_spatial/RobotMaskGenerator.hpp"

using namespace rtr;
using namespace rtr::perception;

const float INNER_DILATION = 0.03f;
const float OUTER_DILATION = 0.15f;

SensorFrameVoxels::ConstPtr GenerateVoxelFrame(const RobotObserver::Ptr observer,
                                               const VoxelRegionDescription& vrd) {
  // set up generated voxel data
  RobotMaskGenerator::MeshMap gt_mesh_map;
  EXPECT_TRUE(testutils::CreateMeshMap(observer, gt_mesh_map));
  std::shared_ptr<CollisionVoxelizer> voxelizer;
  std::vector<std::string> links;
  EXPECT_TRUE(testutils::CreateCollisionVoxelizer(gt_mesh_map, voxelizer, links));

  // generate voxel data
  Array3D<bool> grid;
  EXPECT_TRUE(testutils::Voxelize(observer, voxelizer, links, vrd, grid));
  SensorMetadata meta("", SensorFrameType::SENSOR_FRAME_VOXELS);
  return SensorFrameVoxels::MakePtr(grid, meta);
}

TEST(RobotGridFilter, Filtering) {
  // set up robot observer for filter
  RobotObserver::Ptr observer = testutils::CreateRobotObserver(testutils::UR5_MODEL_NAME);
  Pose offset(Mat3::Identity(), Vec3(-0.5f, -0.5f, -0.1f));
  VoxelRegionDescription vrd(Vec3(64, 64, 64), Vec3(0.5f, 0.5f, .5f), offset.Inverse());
  RobotMaskGenerator::MeshMap filter_mesh_map;
  ASSERT_TRUE(testutils::CreateMeshMap(observer, OUTER_DILATION, OUTER_DILATION, filter_mesh_map));

  // set up filter
  tbb::task_arena arena;
  JointConfiguration config({0.5 * M_PI, -0.5 * M_PI, 0.5 * M_PI, 0.f, M_PI, 0.f});
  observer->SetCurrentJointConfiguration(config);
  RobotGridFilter filter(filter_mesh_map, observer, vrd, RobotMaskGenerator::Type::GRID, arena);

  // generate voxel data
  SensorFrameVoxels::ConstPtr voxel_frame = GenerateVoxelFrame(observer, vrd);

  // double check validity of generated voxel data
  std::vector<Voxel> voxels;
  voxel_frame->GetVoxels(voxels);
  EXPECT_FALSE(voxels.empty());

  //// Test basic filtering
  voxels.clear();
  SensorFrameVoxels::ConstPtr output =
      SensorFrameVoxels::CastConstPtr(filter.GetFilteredNow(voxel_frame));
  output->GetVoxels(voxels);
  EXPECT_TRUE(voxels.empty());
  EXPECT_TRUE(output->GetStateSpaces().count(observer->GetName()));
  if (output->GetStateSpaces().count(observer->GetName())) {
    EXPECT_EQ(output->GetStateSpaces().at(observer->GetName()), (observer->GetStateSpace()));
  }

  //// Test that GetFiltered also filters correctly
  if (!getenv("RS_DISABLE_FLAKYTESTS")) {
    // wait for mask to render
    filter.SetFiltering(true);
    filter.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    voxels.clear();
    output = SensorFrameVoxels::CastConstPtr(filter.GetFiltered(voxel_frame));
    output->GetVoxels(voxels);
    EXPECT_TRUE(voxels.empty());
    EXPECT_TRUE(output->GetStateSpaces().count(observer->GetName()));
    if (output->GetStateSpaces().count(observer->GetName())) {
      EXPECT_EQ(output->GetStateSpaces().at(observer->GetName()), (observer->GetStateSpace()));
    }
  } else {
    RTR_INFO("Skipping timing dependent test in RobotClusterFilter::GetFiltered");
  }
}

TEST(RobotClusterFilter, Filtering) {
  // set up robot observer for filter
  RobotObserver::Ptr observer = testutils::CreateRobotObserver(testutils::UR5_MODEL_NAME);
  Pose offset(Mat3::Identity(), Vec3(-0.25f, -0.375f, -0.1f));
  VoxelRegionDescription vrd(Vec3(64, 64, 64), Vec3(0.25f, 0.375f, .5f), offset.Inverse());
  RobotMaskGenerator::MeshMap filter_mesh_map;
  ASSERT_TRUE(testutils::CreateMeshMap(observer, INNER_DILATION, OUTER_DILATION, filter_mesh_map));

  // set up filter
  tbb::task_arena arena;
  JointConfiguration config({0.5 * M_PI, -0.5 * M_PI, 0.5 * M_PI, 0.f, M_PI, 0.f});
  observer->SetCurrentJointConfiguration(config);
  RobotClusterFilter filter(filter_mesh_map, observer, vrd, true, RobotMaskGenerator::Type::GRID,
                            arena);

  // generate voxel data
  SensorFrameVoxels::ConstPtr voxel_frame = GenerateVoxelFrame(observer, vrd);

  // double check validity of generated voxel data
  std::vector<Voxel> voxels;
  voxel_frame->GetVoxels(voxels, VoxelCluster::Label::OBSTACLE);
  std::size_t num_original_voxels = voxels.size();
  EXPECT_FALSE(voxels.empty());

  //// Test basic filtering
  voxels.clear();
  SensorFrameVoxels::ConstPtr output =
      SensorFrameVoxels::CastConstPtr(filter.GetFilteredNow(voxel_frame));
  output->GetVoxels(voxels, VoxelCluster::Label::OBSTACLE);
  EXPECT_TRUE(voxels.empty());
  EXPECT_TRUE(output->GetStateSpaces().count(observer->GetName()));
  if (output->GetStateSpaces().count(observer->GetName())) {
    EXPECT_EQ(output->GetStateSpaces().at(observer->GetName()), (observer->GetStateSpace()));
  }

  //// Test that noisy clusters are filtered out
  voxels.clear();
  SensorFrameVoxels::ConstPtr noisy_frame = testutils::AddNoiseNearRobot(voxel_frame, 3, 5);
  noisy_frame->GetVoxels(voxels);
  EXPECT_GT(voxels.size(), num_original_voxels);
  std::size_t num_noisy_voxels = voxels.size() - num_original_voxels;
  voxels.clear();
  output = SensorFrameVoxels::CastConstPtr(filter.GetFilteredNow(noisy_frame));
  output->GetVoxels(voxels, VoxelCluster::Label::OBSTACLE);
  output->GetVoxels(voxels, VoxelCluster::Label::OCCLUSION);
  EXPECT_TRUE(voxels.empty());

  //// Test noisy clusters with filter_self_occlusion off
  voxels.clear();
  RobotClusterFilter occupied_filter(filter_mesh_map, observer, vrd, false,
                                     RobotMaskGenerator::Type::GRID, arena);
  output = SensorFrameVoxels::CastConstPtr(occupied_filter.GetFilteredNow(noisy_frame));
  output->GetVoxels(voxels, VoxelCluster::Label::OBSTACLE);
  EXPECT_TRUE(voxels.empty());
  output->GetVoxels(voxels, VoxelCluster::Label::OCCLUSION);
  EXPECT_FALSE(voxels.empty());
  EXPECT_LT(voxels.size(), num_noisy_voxels);

  //// Test that large clusters remain in the grid
  const int line_height = 5;
  SensorFrameVoxels::ConstPtr cluster_frame =
      testutils::AddDiagonalToVoxelFrame(voxel_frame, line_height);
  cluster_frame->GetVoxels(voxels);
  EXPECT_GT(voxels.size(), num_original_voxels);
  voxels.clear();
  output = SensorFrameVoxels::CastConstPtr(filter.GetFilteredNow(cluster_frame));
  const SensorFrameVoxels::VoxelClusters& clusters = output->GetClusters();
  int output_count = 0;
  for (const auto& cluster_pair : clusters) {
    if (cluster_pair.second.GetLabel() == VoxelCluster::Label::OBSTACLE
        || cluster_pair.second.GetLabel() == VoxelCluster::Label::OCCLUSION) {
      output_count++;
    }
  }
  EXPECT_EQ(output_count, 2);
  Array3D<bool> grid;
  output->GetOccGrid(grid);
  EXPECT_TRUE(grid(0, 0, line_height));

  //// Test that GetFiltered also filters correctly
  if (!getenv("RS_DISABLE_FLAKYTESTS")) {
    // wait for mask to render
    filter.SetFiltering(true);
    filter.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    voxels.clear();
    output = SensorFrameVoxels::CastConstPtr(filter.GetFiltered(voxel_frame));
    output->GetVoxels(voxels);
    EXPECT_TRUE(voxels.empty());
    EXPECT_TRUE(output->GetStateSpaces().count(observer->GetName()));
    if (output->GetStateSpaces().count(observer->GetName())) {
      EXPECT_EQ(output->GetStateSpaces().at(observer->GetName()), (observer->GetStateSpace()));
    }
  } else {
    RTR_INFO("Skipping timing dependent test in RobotClusterFilter::GetFiltered");
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
