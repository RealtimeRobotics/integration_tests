#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rtr_perc_api/SensorFrame.hpp>
#include <rtr_perc_api/SensorMetadata.hpp>

#include "rtr_perc_spatial/PerceptionTestUtils.hpp"
#include "rtr_perc_spatial/RobotClusterFilter.hpp"
#include "rtr_perc_spatial/RobotFilter.hpp"
#include "rtr_perc_spatial/RobotMaskGenerator.hpp"
using namespace rtr;
using namespace rtr::perception;
using namespace std;

bool matIsEqual(const cv::Mat mat1, const cv::Mat mat2) {
  if (mat1.empty() && mat2.empty()) {
    return true;
  }

  if (mat1.cols != mat2.cols || mat1.rows != mat2.rows || mat1.type() != mat2.type()
      || abs(cv::norm(mat1, mat2, cv::NORM_L1)) > pow(10, -6)) {
    return false;
  }
  return true;
}
TEST(SensorFrameTest, BasicSensorFrame) {
  SensorTime time_point = std::chrono::high_resolution_clock::now();
  SensorMetadata meta("TestMFG", "TestModel", "SubMod", "Serial",
                      SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_IR), time_point);
  SensorFrame::Ptr frame = SensorFrame::MakePtr(meta, 10);
  SensorExtrinsics::Ptr extrinsics = SensorExtrinsics::MakePtr(
      meta, Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), "world");
  frame->setExtrinsics(extrinsics);

  EXPECT_EQ(frame->getMetadata(),
            SensorMetadata("TestMFG", "TestModel", "SubMod", "Serial",
                           SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_IR), time_point));
  EXPECT_EQ(frame->GetFrameNumber(), 10);
  EXPECT_EQ(*frame->getExtrinsics(),
            SensorExtrinsics(
                SensorMetadata("TestMFG", "TestModel", "SubMod", "Serial",
                               SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_IR), time_point),
                Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), "world"));
}

TEST(SensorFrameTest, SensorFrameImage) {
  SensorTime time_point = std::chrono::high_resolution_clock::now();
  SensorMetadata meta("TestMFG", "TestModel", "SubMod", "Serial",
                      SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_IR), time_point);
  cv::Mat image = cv::Mat(400, 200, CV_8UC3, 5);
  SensorFrameImage::Ptr frame_img = SensorFrameImage::MakePtr(image, meta, 10);
  SensorIntrinsics2D::ConstPtr intrinsics = SensorIntrinsics2D::MakePtr(
      meta, 200, 210, 205, 90, 400, 200, SensorIntrinsics2D::DistortionModel::DISTORTION_NONE,
      cv::Mat::zeros(5, 1, CV_64FC1));
  SensorExtrinsics::Ptr extrinsics = SensorExtrinsics::MakePtr(
      meta, Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), "world");

  frame_img->setIntrinsics(intrinsics);
  frame_img->setExtrinsics(extrinsics);
  SensorFrameImage::DistortionMap dist_map_;
  frame_img->GetDistortionMaps(dist_map_);

  EXPECT_EQ(frame_img->getMetadata(),
            SensorMetadata("TestMFG", "TestModel", "SubMod", "Serial",
                           SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_IR), time_point));
  EXPECT_EQ(frame_img->GetFrameNumber(), 10);
  EXPECT_EQ(*frame_img->getExtrinsics(),
            SensorExtrinsics(
                SensorMetadata("TestMFG", "TestModel", "SubMod", "Serial",
                               SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_IR), time_point),
                Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), "world"));
  EXPECT_EQ(*frame_img->getIntrinsics(),
            SensorIntrinsics2D(
                SensorMetadata("TestMFG", "TestModel", "SubMod", "Serial",
                               SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_IR), time_point),
                200, 210, 205, 90, 400, 200, SensorIntrinsics2D::DistortionModel::DISTORTION_NONE,
                cv::Mat::zeros(5, 1, CV_64FC1)));
  EXPECT_TRUE(matIsEqual(frame_img->image(), cv::Mat(400, 200, CV_8UC3, 5)));
  EXPECT_FALSE(dist_map_.first.empty());
  EXPECT_FALSE(dist_map_.second.empty());
}

TEST(SensorFrameTest, SensorFrameDepthImage) {
  SensorTime time_point = std::chrono::high_resolution_clock::now();
  SensorMetadata meta("TestMFG", "TestModel", "SubMod", "Serial",
                      SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH), time_point);
  cv::Mat image = cv::Mat(400, 200, CV_64FC1, 5);
  SensorFrameDepthImage::Ptr frame_depth_img = SensorFrameDepthImage::MakePtr(image, 0.5, meta, 10);
  SensorIntrinsics2D::ConstPtr intrinsics = SensorIntrinsics2D::MakePtr(
      meta, 200, 210, 205, 90, 400, 200, SensorIntrinsics2D::DistortionModel::DISTORTION_NONE,
      cv::Mat::zeros(5, 1, CV_64FC1));
  SensorExtrinsics::Ptr extrinsics = SensorExtrinsics::MakePtr(
      meta, Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), "world");

  frame_depth_img->setIntrinsics(intrinsics);
  frame_depth_img->setExtrinsics(extrinsics);

  EXPECT_EQ(frame_depth_img->getMetadata(),
            SensorMetadata("TestMFG", "TestModel", "SubMod", "Serial",
                           SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH), time_point));
  EXPECT_EQ(frame_depth_img->GetFrameNumber(), 10);
  EXPECT_EQ(
      *frame_depth_img->getExtrinsics(),
      SensorExtrinsics(
          SensorMetadata("TestMFG", "TestModel", "SubMod", "Serial",
                         SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH), time_point),
          Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), "world"));
  EXPECT_EQ(
      *frame_depth_img->getIntrinsics(),
      SensorIntrinsics2D(
          SensorMetadata("TestMFG", "TestModel", "SubMod", "Serial",
                         SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH), time_point),
          200, 210, 205, 90, 400, 200, SensorIntrinsics2D::DistortionModel::DISTORTION_NONE,
          cv::Mat::zeros(5, 1, CV_64FC1)));
  EXPECT_TRUE(matIsEqual(frame_depth_img->image(), cv::Mat(400, 200, CV_16UC1, 5)));
  EXPECT_EQ(frame_depth_img->GetDepthValues().rows, 400);
  EXPECT_EQ(frame_depth_img->GetDepthValues().cols, 200);
  EXPECT_EQ(frame_depth_img->GetDepthValues().type(), 5);
}

TEST(SensorFrameTest, SensorFameVoxelsAndRobotFiltering) {
  RobotObserver::Ptr observer = testutils::CreateRobotObserver(testutils::UR5_MODEL_NAME);
  Pose offset(Mat3::Identity(), Vec3(-0.25f, -0.375f, -0.1f));
  VoxelRegionDescription vrd(Vec3(64, 64, 64), Vec3(0.25f, 0.375f, .5f), offset.Inverse());
  RobotMaskGenerator::MeshMap filter_mesh_map;
  ASSERT_TRUE(testutils::CreateMeshMap(observer, 0.03f, 0.15f, filter_mesh_map));

  tbb::task_arena arena;
  JointConfiguration config({0.5 * M_PI, -0.5 * M_PI, 0.5 * M_PI, 0.f, M_PI, 0.f});
  observer->SetCurrentJointConfiguration(config);
  RobotClusterFilter filter(filter_mesh_map, observer, vrd, true, RobotMaskGenerator::Type::GRID,
                            arena);

  RobotMaskGenerator::MeshMap gt_mesh_map;
  EXPECT_TRUE(testutils::CreateMeshMap(observer, gt_mesh_map));
  std::shared_ptr<CollisionVoxelizer> voxelizer;
  std::vector<std::string> links;
  EXPECT_TRUE(testutils::CreateCollisionVoxelizer(gt_mesh_map, voxelizer, links));

  Array3D<bool> grid;
  EXPECT_TRUE(testutils::Voxelize(observer, voxelizer, links, vrd, grid));
  SensorMetadata meta("", SensorFrameType::SENSOR_FRAME_VOXELS);
  SensorFrameVoxels::ConstPtr voxel_frame = SensorFrameVoxels::MakePtr(grid, meta);

  std::vector<Voxel> voxels;
  voxel_frame->GetVoxels(voxels, VoxelCluster::Label::OBSTACLE);
  EXPECT_FALSE(voxels.empty());

  //// Test basic filtering
  voxels.clear();
  SensorFrameVoxels::ConstPtr output =
      SensorFrameVoxels::CastConstPtr(filter.GetFilteredNow(voxel_frame));
  output->GetVoxels(voxels, VoxelCluster::Label::OBSTACLE);
  EXPECT_TRUE(voxels.empty());
  EXPECT_EQ(observer->GetStateSpace(), output->GetStateSpace());
  EXPECT_EQ(output->GetMaxClusterId(), 0);
}

TEST(SensorFrameTest, SensorFrameHeatMap) {
  Array3D<float> grid(5, 5, 5, 0.1);
  grid(0, 2, 4) = 0.5;
  grid(1, 3, 2) = 0.4;
  SensorFrameHeatMap frame_heat(grid, 0.1, 0.6);

  EXPECT_EQ(frame_heat.GetResolution()[0], 5);
  EXPECT_EQ(frame_heat.GetResolution()[1], 5);
  EXPECT_EQ(frame_heat.GetResolution()[2], 5);
  EXPECT_FLOAT_EQ(frame_heat.GetMinPercentage(), 0.1);
  EXPECT_FLOAT_EQ(frame_heat.GetMaxPercentage(), 0.6);
  EXPECT_EQ(frame_heat.GetOcclusionVolumePercent(), 100);
  EXPECT_EQ(frame_heat.GetOccGrid().NumItems(), 125);
}

TEST(SensorFrameTest, SensorFrameRGBDImage) {
  SensorTime time_point = std::chrono::high_resolution_clock::now();
  SensorMetadata meta1("TestMFG", "TestModel", "SubMod", "Serial",
                       SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_IR), time_point);
  cv::Mat image1 = cv::Mat(400, 200, CV_8UC3, 5);
  SensorFrameImage::Ptr frame_img = SensorFrameImage::MakePtr(image1, meta1, 10);
  SensorMetadata meta2("TestMFG", "TestModel", "SubMod", "Serial",
                       SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH), time_point);
  cv::Mat image2 = cv::Mat(400, 200, CV_64FC1, 5);
  SensorFrameDepthImage::Ptr frame_depth_img =
      SensorFrameDepthImage::MakePtr(image2, 0.5, meta2, 10);
  SensorMetadata meta("TestMFG", "TestModel", "SubMod", "Serial",
                      SensorFrameType(SensorFrameType::SENSOR_FRAME_IMAGE_RGB_D), time_point);
  SensorFrameRGBDImage::Ptr frame_rgbd_img =
      SensorFrameRGBDImage::MakePtr(frame_img, frame_depth_img, meta, 10);

  EXPECT_TRUE(matIsEqual(frame_rgbd_img->GetRGB(), cv::Mat(400, 200, 16, 5)));
  EXPECT_TRUE(matIsEqual(frame_rgbd_img->GetDepth(), cv::Mat(400, 200, 2, 5)));
  EXPECT_FLOAT_EQ(frame_rgbd_img->GetScale(), 0.5);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  RTR_INFO("Running sensor frame tests...");
  return RUN_ALL_TESTS();
}
