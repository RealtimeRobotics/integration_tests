// Copyright (C) 2019 Realtime Robotics

#include <QApplication>

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rtr_geometry/MeshPrimitives.hpp>
#include <rtr_perc_api/SensorCalibration.hpp>

#include "rtr_perc_spatial/ColorMeshImageGenerator.hpp"
#include "rtr_perc_spatial/DepthMeshImageGenerator.hpp"
#include "rtr_perc_spatial/PerceptionTestUtils.hpp"

using namespace rtr;
using namespace rtr::perception;

const int INTERSECT_TOLERANCE = 10;

// Add mesh to MeshImageGenerator and render
SensorFrame::ConstPtr AddMeshAndRender(const MeshImageGenerator::Ptr generator,
                                       const std::string &mesh_name,
                                       const TriMesh::Ptr mesh, const Mat3 &R,
                                       const Vec3 &t,
                                       std::map<std::string, Pose> &pose_map,
                                       std::vector<TriMesh::Ptr> &meshes,
                                       std::vector<Pose> &poses) {
  EXPECT_TRUE(generator->AddMesh(mesh_name, {mesh}));
  EXPECT_TRUE(generator->AddColor(mesh_name, {Vec4(0.5, 0.5, 0.5, 1.0)}));
  pose_map[mesh_name] = Pose(R, t);
  SensorFrame::ConstPtr frame = generator->Render(pose_map);
  meshes.push_back(mesh);
  poses.push_back(pose_map[mesh_name]);
  return frame;
}

// Set up MeshImageGenerator with correct initial parameters
MeshImageGenerator::Ptr
SetupMeshImageGenerator(const SensorIntrinsics2D::ConstPtr intr,
                        const SensorExtrinsics::ConstPtr extr,
                        const float near_clip, const float far_clip,
                        const float scale) {
  const SensorFrameType ft = intr->getMetadata().getFrameType();

  MeshImageGenerator::Ptr generator;
  if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
    generator = DepthMeshImageGenerator::MakePtr();
  } else {
    generator = ColorMeshImageGenerator::MakePtr();
  }

  EXPECT_TRUE(
      generator->Init(intr->width(), intr->height(), near_clip, far_clip, ft));
  generator->SetCameraModel(intr, extr);
  if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
    DepthMeshImageGenerator::CastPtr(generator)->SetScale(scale);
  }

  return generator;
}

// Create MeshImageGenerator, add mesh primitives and check accuracy
void TestMeshPrimitiveAccuracy(const SensorIntrinsics2D::ConstPtr intr,
                               const SensorExtrinsics::ConstPtr extr,
                               const float near_clip, const float far_clip,
                               const float scale) {
  const SensorFrameType ft = intr->getMetadata().getFrameType();
  MeshImageGenerator::Ptr generator =
      SetupMeshImageGenerator(intr, extr, near_clip, far_clip, scale);

  std::map<std::string, Pose> pose_map;
  std::vector<TriMesh::Ptr> meshes;
  std::vector<Pose> poses;

  // add cone in the center
  TriMesh::Ptr cone = TriMesh::MakePtr();
  ConeTriMesh(0.1, 0.25, 30, *cone);
  SensorFrame::ConstPtr frame = AddMeshAndRender(
      generator, "cone", cone, Mat3(0.0, 0.8509035, 0.0, 0.525322),
      Vec3(0.0, 0.0, 1.0), pose_map, meshes, poses);
  if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
    EXPECT_TRUE(testutils::TestDepthImageAccuracy(
        SensorFrameDepthImage::CastConstPtr(frame), far_clip, meshes, poses,
        INTERSECT_TOLERANCE));
  } else {
    EXPECT_TRUE(
        testutils::TestColorImageAccuracy(SensorFrameImage::CastConstPtr(frame),
                                          meshes, poses, INTERSECT_TOLERANCE));
  }

  // add cylinder across the screen in front of cone
  TriMesh::Ptr cylinder = TriMesh::MakePtr();
  CylinderTriMesh(0.05, 1.5, 30, *cylinder);
  frame = AddMeshAndRender(generator, "cylinder", cylinder,
                           Mat3(0.0, 0.0, 0.8509035, 0.525322),
                           Vec3(0.1, -0.1, 0.8), pose_map, meshes, poses);
  if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
    EXPECT_TRUE(testutils::TestDepthImageAccuracy(
        SensorFrameDepthImage::CastConstPtr(frame), far_clip, meshes, poses,
        INTERSECT_TOLERANCE));
  } else {
    EXPECT_TRUE(
        testutils::TestColorImageAccuracy(SensorFrameImage::CastConstPtr(frame),
                                          meshes, poses, INTERSECT_TOLERANCE));
  }

  // add torus off to the bottom right
  TriMesh::Ptr torus = TriMesh::MakePtr();
  TorusTriMesh(0.2, 0.1, 30, 30, *torus);
  frame = AddMeshAndRender(generator, "torus", torus,
                           Mat3(0, -0.9589243, 0, 0.2836622),
                           Vec3(-0.4, -0.2, 0.8), pose_map, meshes, poses);
  if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
    EXPECT_TRUE(testutils::TestDepthImageAccuracy(
        SensorFrameDepthImage::CastConstPtr(frame), far_clip, meshes, poses,
        INTERSECT_TOLERANCE));
  } else {
    EXPECT_TRUE(
        testutils::TestColorImageAccuracy(SensorFrameImage::CastConstPtr(frame),
                                          meshes, poses, INTERSECT_TOLERANCE));
  }

  // add cut off sphere in the bottom left corner
  TriMesh::Ptr sphere = TriMesh::MakePtr();
  SphereTriMesh(0.3, 30, 30, *sphere);
  frame =
      AddMeshAndRender(generator, "sphere", sphere, Mat3(0.0, 0.0, 0.0, 1.0),
                       Vec3(0.6, -0.5, 0.55), pose_map, meshes, poses);
  if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
    EXPECT_TRUE(testutils::TestDepthImageAccuracy(
        SensorFrameDepthImage::CastConstPtr(frame), far_clip, meshes, poses,
        INTERSECT_TOLERANCE));
  } else {
    EXPECT_TRUE(
        testutils::TestColorImageAccuracy(SensorFrameImage::CastConstPtr(frame),
                                          meshes, poses, INTERSECT_TOLERANCE));
  }

  // add pyramid to the bottom behind torus
  TriMesh::Ptr pyramid = TriMesh::MakePtr();
  PyramidTriMesh(0.4, 0.2, 0.3, *pyramid);
  frame = AddMeshAndRender(generator, "pyramid", pyramid,
                           Mat3(-0.4182878, -0.6193761, 0.4255434, 0.5102169),
                           Vec3(0.0, -0.3, 0.9), pose_map, meshes, poses);
  if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
    EXPECT_TRUE(testutils::TestDepthImageAccuracy(
        SensorFrameDepthImage::CastConstPtr(frame), far_clip, meshes, poses,
        INTERSECT_TOLERANCE));
  } else {
    EXPECT_TRUE(
        testutils::TestColorImageAccuracy(SensorFrameImage::CastConstPtr(frame),
                                          meshes, poses, INTERSECT_TOLERANCE));
  }

  // add star in the top left corner
  TriMesh::Ptr star = TriMesh::MakePtr();
  StarTriMesh(9, 0.25, 0.35, 0.15, *star);
  frame = AddMeshAndRender(generator, "star", star,
                           Mat3(-0.6276726, 0.3968189, -0.661156, -0.1069327),
                           Vec3(0.6, 0.4, 1.4), pose_map, meshes, poses);
  if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
    EXPECT_TRUE(testutils::TestDepthImageAccuracy(
        SensorFrameDepthImage::CastConstPtr(frame), far_clip, meshes, poses,
        INTERSECT_TOLERANCE));
  } else {
    EXPECT_TRUE(
        testutils::TestColorImageAccuracy(SensorFrameImage::CastConstPtr(frame),
                                          meshes, poses, INTERSECT_TOLERANCE));
  }
}

// Create MeshImageGenerator, add robot, add mesh primitives and check accuracy
void TestRobotAccuracy(const SensorIntrinsics2D::ConstPtr intr,
                       const SensorExtrinsics::ConstPtr extr,
                       const float near_clip, const float far_clip,
                       const float scale, const std::string &model_name,
                       const std::vector<JointConfiguration> &configs) {
  const SensorFrameType ft = intr->getMetadata().getFrameType();
  MeshImageGenerator::Ptr generator =
      SetupMeshImageGenerator(intr, extr, near_clip, far_clip, scale);
  RobotObserver::Ptr observer = testutils::CreateRobotObserver(model_name);
  ASSERT_TRUE(observer);
  RobotMaskGenerator::MeshMap mesh_map;
  ASSERT_TRUE(testutils::CreateMeshMap(observer, mesh_map));
  generator->AddRobot(observer, mesh_map);

  // add cylinder
  TriMesh::Ptr cylinder = TriMesh::MakePtr();
  CylinderTriMesh(0.1, 0.25, 30, *cylinder);
  EXPECT_TRUE(generator->AddMesh("cylinder", {cylinder}));
  EXPECT_TRUE(generator->AddColor("cylinder", {Vec4(0.5, 0.5, 0.5, 1.0)}));

  // add torus
  TriMesh::Ptr torus = TriMesh::MakePtr();
  TorusTriMesh(0.125, 0.05, 30, 30, *torus);
  EXPECT_TRUE(generator->AddMesh("torus", {torus}));
  EXPECT_TRUE(generator->AddColor("torus", {Vec4(0.5, 0.5, 0.5, 1.0)}));

  // render and test the robot with different joint states
  for (const auto &config : configs) {
    observer->SetCurrentJointConfiguration(config);
    RobotObserver::LinkPoseMap pose_map = observer->GetLinkPoses();

    pose_map["cylinder"] =
        Pose(Mat3(0.0, 0.0, 0.8509035, 0.525322), Vec3(0.5, 0.5, 0.25));
    pose_map["torus"] =
        Pose(Mat3(0, -0.9589243, 0, 0.2836622), Vec3(-0.6, -0.6, 0.3));

    SensorFrame::ConstPtr frame = generator->Render(pose_map);

    std::vector<TriMesh::Ptr> meshes;
    std::vector<Pose> poses;
    for (const auto &mesh_pair : mesh_map.filter_meshes) {
      for (const auto &mesh : mesh_pair.second) {
        meshes.push_back(mesh);
        poses.push_back(pose_map.at(mesh_pair.first));
      }
    }
    meshes.push_back(torus);
    meshes.push_back(cylinder);
    poses.push_back(pose_map["torus"]);
    poses.push_back(pose_map["cylinder"]);

    if (ft == SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH) {
      EXPECT_TRUE(testutils::TestDepthImageAccuracy(
          SensorFrameDepthImage::CastConstPtr(frame), far_clip, meshes, poses,
          INTERSECT_TOLERANCE));
    } else {
      EXPECT_TRUE(testutils::TestColorImageAccuracy(
          SensorFrameImage::CastConstPtr(frame), meshes, poses,
          INTERSECT_TOLERANCE));
    }
  }
}

//// Test depth accuracy of basic mesh primitives with different clips and
/// scales
TEST(DepthMeshImageGenerator, MeshDepthAccuracy) {
  SensorMetadata meta("", SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH);
  SensorIntrinsics2D::Ptr intr = SensorIntrinsics2D::MakePtr(
      meta, 200, 210, 205, 90, 400, 200,
      SensorIntrinsics2D::DistortionModel::DISTORTION_NONE,
      cv::Mat::zeros(5, 1, CV_64FC1));
  SensorExtrinsics::Ptr extr =
      SensorExtrinsics::MakePtr(meta, Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0),
                                Eigen::Vector3d(0.0, 0.0, 0.0), "world");

  // scale and clip of D435 / D455
  TestMeshPrimitiveAccuracy(intr, extr, 0.1, 10.0, 0.001);

  // scale and clip of L515
  TestMeshPrimitiveAccuracy(intr, extr, 0.1, 10.0, 0.00025);

  // different clips with different amounts of depth precision
  TestMeshPrimitiveAccuracy(intr, extr, 0.15, 5.0, 0.0075);
  TestMeshPrimitiveAccuracy(intr, extr, 0.2, 4.0, 0.0005);
  TestMeshPrimitiveAccuracy(intr, extr, 0.25, 3.0, 0.00025);
}

//// Test depth accuracy of robot model + mesh primitives
// Note that this is only a simple robot model test, SimSensorTest provides more
// extensive testing
TEST(DepthMeshImageGenerator, RobotModelDepthAccuracy) {
  SensorMetadata meta("", SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH);
  SensorIntrinsics2D::Ptr intr = SensorIntrinsics2D::MakePtr(
      meta, 205, 190, 205, 110, 420, 200,
      SensorIntrinsics2D::DistortionModel::DISTORTION_NONE,
      cv::Mat::zeros(5, 1, CV_64FC1));
  SensorExtrinsics::Ptr extr = SensorExtrinsics::MakePtr(
      meta, Eigen::Quaterniond(-0.2233364, 0.8835876, 0.1601474, 0.3791393),
      Eigen::Vector3d(-0.75, -0.75, 0.75), "world");

  // UR3 test with scale and clip of D435 / D455
  TestRobotAccuracy(intr, extr, 0.1, 10.0, 0.001, testutils::UR3_MODEL_NAME,
                    testutils::UR5_JOINTS);

  // UR3 test with scale and clip of L515
  TestRobotAccuracy(intr, extr, 0.1, 10.0, 0.00025, testutils::UR3_MODEL_NAME,
                    testutils::UR5_JOINTS);
}

//// Test ray intersection accuracy of basic primitives with different clips and
/// scales
TEST(ColorMeshImageGenerator, MeshMaskAccuracy) {
  SensorMetadata meta("", SensorFrameType::SENSOR_FRAME_IMAGE_RGB);
  SensorIntrinsics2D::Ptr intr = SensorIntrinsics2D::MakePtr(
      meta, 225, 215, 195, 155, 400, 300,
      SensorIntrinsics2D::DistortionModel::DISTORTION_NONE,
      cv::Mat::zeros(5, 1, CV_64FC1));
  SensorExtrinsics::Ptr extr =
      SensorExtrinsics::MakePtr(meta, Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0),
                                Eigen::Vector3d(0.0, 0.0, 0.0), "world");

  // test with different clips
  TestMeshPrimitiveAccuracy(intr, extr, 0.1, 10.0, 0.0);
  TestMeshPrimitiveAccuracy(intr, extr, 0.15, 5.0, 0.0);
  TestMeshPrimitiveAccuracy(intr, extr, 0.2, 4.0, 0.0);
  TestMeshPrimitiveAccuracy(intr, extr, 0.25, 3.0, 0.0);
}

//// Test ray intersection of robot model + mesh primitives
TEST(ColorMeshImageGenerator, RobotModelMaskAccuracy) {
  SensorMetadata meta("", SensorFrameType::SENSOR_FRAME_IMAGE_RGB);
  SensorIntrinsics2D::Ptr intr = SensorIntrinsics2D::MakePtr(
      meta, 215, 101, 170, 100, 350, 210,
      SensorIntrinsics2D::DistortionModel::DISTORTION_NONE,
      cv::Mat::zeros(5, 1, CV_64FC1));
  SensorExtrinsics::Ptr extr = SensorExtrinsics::MakePtr(
      meta, Eigen::Quaterniond(-0.2233364, 0.8835876, 0.1601474, 0.3791393),
      Eigen::Vector3d(-0.75, -0.75, 0.75), "world");

  // UR3 test with scale and clip of D435 / D455
  TestRobotAccuracy(intr, extr, 0.1, 10.0, 0.001, testutils::UR3_MODEL_NAME,
                    testutils::UR5_JOINTS);

  // UR3 test with scale and clip of L515
  TestRobotAccuracy(intr, extr, 0.1, 10.0, 0.00025, testutils::UR3_MODEL_NAME,
                    testutils::UR5_JOINTS);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);

  return RUN_ALL_TESTS();
}
