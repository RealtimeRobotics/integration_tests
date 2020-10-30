// Copyright (C) 2019 Realtime Robotics

#include <QApplication>

#include <gtest/gtest.h>

#include <rtr_perc_spatial/OcclusionMapGenerator.hpp>
#include <rtr_perc_spatial/PerceptionTestUtils.hpp>
#include <rtr_utils/PackagePath.hpp>

using namespace rtr;
using namespace rtr::perception;

const std::string OG_PATH = "data/87791cc3-7698-406a-906b-6adccdfb0abd.og";
const std::string CONFIG_PATH = "data/config.xml";
const std::string DILATED_MESH_PATH = "data/dilated_mesh.bin";
const std::string CALIBRATION_FOLDER = "data/calibration";

//// Test operation
TEST(OcclusionMapGenerator, SimulationRunTest) {
  RTR_INFO("Starting OcclusionMapGenerator Simulation Run Test");
  OGFileReader og_reader(OG_PATH);
  EXPECT_TRUE(og_reader.IsValid());

  RobotObserver::Ptr robot =
      testutils::CreateRobotObserver(testutils::UR3_MODEL_NAME);
  RobotMaskGenerator::MeshMap mesh_map;
  EXPECT_NO_THROW(rtr::LoadFile(DILATED_MESH_PATH, mesh_map,
                                rtr::Format::BOOST_BINARY,
                                rtr::Encoding::BINARY_RAW));

  std::string robot_name = robot->GetName();
  std::map<std::string, OGFileReader> ogs = {{robot_name, og_reader}};
  std::map<std::string, RobotObserver::Ptr> robots = {{robot_name, robot}};
  std::map<std::string, RobotMaskGenerator::MeshMap> mesh_maps = {
      {robot_name, mesh_map}};

  SpatialPerceptionProjectSchema config;
  EXPECT_TRUE(config.Deserialize(CONFIG_PATH));
  config.sensors_folder =
      rtr::utils::RelativeToAbsolutePath(CALIBRATION_FOLDER);

  OcclusionMapGenerator::Ptr map_gen = nullptr;

  EXPECT_NO_THROW(
      map_gen = OcclusionMapGenerator::MakePtr(ogs, config, robots, mesh_maps));

  EXPECT_NE(map_gen, nullptr);

  EXPECT_NO_THROW(map_gen->Init());

  VoxelRegionDescription vrd = map_gen->GetVoxelRegionDes();

  EXPECT_EQ(vrd.num_voxels[0], config.streams[0].num_voxels[0]);
  EXPECT_EQ(vrd.num_voxels[1], config.streams[0].num_voxels[1]);
  EXPECT_EQ(vrd.num_voxels[2], config.streams[0].num_voxels[2]);

  EXPECT_NO_THROW(map_gen->GeneratePoses(100));

  auto fn = [](SensorFrame::ConstPtr, std::string, float) {};

  EXPECT_NO_THROW(map_gen->RunSimulation(fn));

  OcclusionData::Ptr data = map_gen->GetData();

  EXPECT_NE(data, nullptr);

  EXPECT_EQ(data->GetNumTotalPoses(), 100);

  rtr::Array3D<int> output;
  data->GetHeatMapAllVoxel(output);

  // check for presense of some data
  int count = 0;
  for (int x = 0; x < 64; x++) {
    for (int y = 0; y < 64; y++) {
      for (int z = 0; z < 64; z++) {
        int val = output(x, y, z);
        if (val) {
          EXPECT_LE(val, 100);
          count++;
        }
      }
    }
  }

  EXPECT_GT(count, 0);
  RTR_INFO("Ending OcclusionMapGenerator Simulation Run Test");
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
