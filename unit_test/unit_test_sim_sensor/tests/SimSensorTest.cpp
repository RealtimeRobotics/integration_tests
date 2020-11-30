// Copyright (C) 2019 Realtime Robotics

#include <QApplication>

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rtr_geometry/MeshIO.hpp>
#include <rtr_perc_calibration/CalibrationUtil.hpp>
#include <rtr_perc_calibration/Calibrator.hpp>
#include <rtr_perc_spatial/PerceptionTestUtils.hpp>
#include <rtr_perc_spatial/SimSensor.hpp>

using namespace rtr;
using namespace rtr::perception;

// need high reprojection tolerance, since rendered version of checkerboard
// cannot achieve subpixel accuracy
const float REPROJ_TOLERANCE = 3.0;
const std::string FLANGE_FRAME = "rtr_flange";
const std::string FIDUCIAL_MESH = "rtr_fiducial_sim";

bool DepthImageEquals(const cv::Mat& a, const cv::Mat& b) {
  if (a.rows != b.rows || a.cols != b.cols) {
    return false;
  }

  cv::Mat diff;
  cv::absdiff(a, b, diff);
  double min_val, max_val;
  cv::minMaxLoc(diff, &min_val, &max_val);
  return max_val == min_val;  // uint16 image will round to nearest mm
}

bool IsBlankImage(const cv::Mat image) {
  double min_val, max_val;
  cv::minMaxLoc(image, &min_val, &max_val);
  return FuzzyEquals(max_val, min_val);
}

// Render image and check validity of output
cv::Mat TestRender(const SimSensor::Ptr sensor, const RobotObserver::Ptr robot,
                   const RobotMaskGenerator::MeshMap& mesh_map) {
  // add robot and render image
  EXPECT_TRUE(sensor->AddRobot(robot, mesh_map));
  const SensorFrameType ft = SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH;
  SensorFrame::ConstPtr rendered_frame = testutils::GetRenderedImageFromSimSensor(sensor, ft);

  //// Test validity of depth frame
  EXPECT_TRUE(rendered_frame);
  cv::Mat image;
  if (rendered_frame) {
    SensorFrameDepthImage::ConstPtr depth_frame =
        SensorFrameDepthImage::CastConstPtr(rendered_frame);
    SensorIntrinsics2D::ConstPtr frame_intr = depth_frame->getIntrinsics();
    SensorIntrinsics2D::ConstPtr default_intr =
        SensorIntrinsics2D::CastConstPtr(sensor->GetDefaultSimulatedIntrinsics(ft));
    EXPECT_EQ(depth_frame->getMetadata().getSensorUid(), sensor->getUid());
    EXPECT_EQ(depth_frame->getFrameType(), ft);
    EXPECT_EQ(*frame_intr, *default_intr);
    image = depth_frame->image();
    EXPECT_FALSE(image.empty());
    EXPECT_FALSE(IsBlankImage(image));
  }

  return image;
}

// Render depth image for each provided config and check the accuracy of the
// rendered image against ray-mesh intersection
void TestDepthAccuracy(const std::string& uid, const std::string& model_name,
                       const std::vector<JointConfiguration>& configs,
                       const int intersect_tolerance) {
  SimSensor::Ptr sensor = testutils::CreateSimSensor(
      uid, "", Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 2.0));

  RobotObserver::Ptr observer = testutils::CreateRobotObserver(model_name);
  RobotMaskGenerator::MeshMap mesh_map;
  ASSERT_TRUE(testutils::CreateMeshMap(observer, mesh_map));
  ASSERT_TRUE(sensor->AddRobot(observer, mesh_map));

  for (const auto& config : configs) {
    // render image at robot config
    observer->SetCurrentJointConfiguration(config);
    SensorFrame::ConstPtr frame =
        testutils::GetRenderedImageFromSimSensor(sensor, SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH);
    SensorFrameDepthImage::ConstPtr depth_frame = SensorFrameDepthImage::CastConstPtr(frame);
    cv::Mat image = depth_frame->image();
    ASSERT_FALSE(IsBlankImage(image));

    // generate obbtrees for new robot config (could probably optimize this out)
    RobotObserver::LinkPoseMap pose_map = observer->GetLinkPoses();
    std::vector<TriMesh::Ptr> meshes;
    std::vector<Pose> poses;
    for (const auto& mesh_pair : mesh_map.filter_meshes) {
      const std::string& link_name = mesh_pair.first;
      if (pose_map.count(link_name)) {
        for (const auto& mesh : mesh_pair.second) {
          poses.push_back(pose_map[link_name]);
          meshes.push_back(mesh);
        }
      }
    }
    EXPECT_TRUE(
        testutils::TestDepthImageAccuracy(depth_frame, 10.f, meshes, poses, intersect_tolerance));
  }
}

// Move robot to joint configuration and check for fiducial
bool DetectFiducial(const RobotObserver::Ptr observer, const JointConfiguration& config,
                    const SimSensor::Ptr sensor, const SensorFrameType& ft,
                    const CalibrationTarget& target, std::vector<cv::Point2f>& corners) {
  observer->SetCurrentJointConfiguration(config);
  SensorFrame::ConstPtr rendered_frame = testutils::GetRenderedImageFromSimSensor(sensor, ft);
  SensorFrameImage::ConstPtr image_frame = SensorFrameImage::CastConstPtr(rendered_frame);
  cv::Mat image;
  cv::cvtColor(image_frame->image(), image, CV_GRAY2RGB);
  return Calibrator::FindChessboard(image, target, corners);
}

// Check that the fiducial pose detected by the simulated sensor matches the
// expected fiducial pose from the robot link transforms
float ComputeReprojError(const RobotObserver::Ptr observer, const SimSensor::Ptr sensor,
                         const SensorFrameType& ft, const CalibrationTarget& target,
                         const std::vector<cv::Point2f>& corners) {
  RobotObserver::LinkPoseMap pose_map = observer->GetLinkPoses();
  const std::map<SensorFrameType, SensorIntrinsics::ConstPtr> intr_map = sensor->getIntrinsics();
  const std::map<SensorFrameType, SensorExtrinsics::ConstPtr> extr_map = sensor->getExtrinsics();

  if (!pose_map.count(FLANGE_FRAME)) {
    RTR_ERROR("Failed to get pose of flange frame in RobotObserver");
    return Inf;
  }
  Pose fiducial_pose = pose_map[FLANGE_FRAME];
  fiducial_pose = fiducial_pose * target.info.flange_to_target * target.info.target_to_origin;
  if (!intr_map.count(ft) || !extr_map.count(ft)) {
    RTR_ERROR("Failed to find intrinsics or extrinsics of frame type {}", ft);
    return Inf;
  }
  SensorIntrinsics2D::ConstPtr intr = SensorIntrinsics2D::CastConstPtr(intr_map.at(ft));
  Pose points_tf = extr_map.at(ft)->GetPose().Inverse() * fiducial_pose;

  for (auto pt : target.points) {
    Vec3 point(pt.x, pt.y, pt.z);
  }

  cv::Mat K(intr->getKMatrix());
  cv::Mat dist(intr->getLensDistortion());
  return Calibrator::ComputeReprojError(corners, target.points, points_tf.ToEigen(), K, dist);
}

//// Test setting and getting of calibration
TEST(SimSensor, SetCalibration) {
  const std::string uid = "INTEL_REALSENSE_D435_000000000000";
  SimSensor::Ptr sensor = SimSensor::MakePtr(uid, "");

  // set intrinsics / extrinsics
  std::set<SensorFrameType> types = sensor->GetPersistentCalibrationFrameTypes();
  std::map<SensorFrameType, SensorIntrinsics::ConstPtr> intrin_in, intrin_out;
  std::map<SensorFrameType, SensorExtrinsics::ConstPtr> extrin_in, extrin_out;
  for (const auto& ft : types) {
    SensorIntrinsics::Ptr intrin = sensor->GetDefaultSimulatedIntrinsics(ft);
    ASSERT_TRUE(intrin);
    SensorExtrinsics::Ptr extrin = sensor->GetDefaultSimulatedExtrinsics(ft);
    ASSERT_TRUE(extrin);
    intrin_in[ft] = intrin;
    extrin_in[ft] = extrin;
  }
  sensor->SetIntrinsics(intrin_in);
  sensor->SetExtrinsics(extrin_in);

  // check output intrinsics / extrinsics
  intrin_out = sensor->getIntrinsics();
  extrin_out = sensor->getExtrinsics();
  for (const auto& ft : types) {
    EXPECT_TRUE(intrin_out.count(ft));
    EXPECT_TRUE(extrin_out.count(ft));

    if (intrin_out.count(ft)) {
      SensorIntrinsics2D::ConstPtr in = SensorIntrinsics2D::CastConstPtr(intrin_in[ft]);
      SensorIntrinsics2D::ConstPtr out = SensorIntrinsics2D::CastConstPtr(intrin_out[ft]);
      EXPECT_EQ(*in, *out);
    }
    if (extrin_out.count(ft)) {
      EXPECT_EQ(*extrin_in[ft], *extrin_out[ft]);
    }
  }
}

//// Test depth accuracy of rendered UR10 robot
TEST(SimSensor, UR10DepthAccuracy) {
  const std::string uid = "INTEL_REALSENSE_D435_000000000000";
  const std::string model_name = testutils::UR10_MODEL_NAME;
  const int intersect_tolerance = 10;
  TestDepthAccuracy(uid, model_name, testutils::UR5_JOINTS, intersect_tolerance);
}

//// Test depth accuracy of rendered FANUC robot
TEST(SimSensor, FanucDepthAccuracy) {
  const std::string uid = "INTEL_REALSENSE_D435_000000000000";
  const std::string model_name = testutils::FANUC_MODEL_NAME;
  const int intersect_tolerance = 10;
  TestDepthAccuracy(uid, model_name, testutils::FANUC_JOINTS, intersect_tolerance);
}

//// Test depth accuracy of rendered MELCO robot
TEST(SimSensor, MelcoDepthAccuracy) {
  const std::string uid = "INTEL_REALSENSE_D435_000000000000";
  const std::string model_name = testutils::MELCO_MODEL_NAME;
  const int intersect_tolerance = 10;
  TestDepthAccuracy(uid, model_name, testutils::MELCO_JOINTS, intersect_tolerance);
}

//// Verify that different sensors are actually rendering images with different
/// sizes and FOVs
TEST(SimSensor, SensorModelDepthDifference) {
  const Eigen::Quaterniond q = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
  const Eigen::Vector3d xyz = Eigen::Vector3d(0.0, 0.0, 2.0);
  SimSensor::Ptr d435_sensor =
      testutils::CreateSimSensor("INTEL_REALSENSE_D435_000000000000", "", q, xyz);
  SimSensor::Ptr d435e_sensor =
      testutils::CreateSimSensor("FRAMOS_REALSENSE_D435e_000000000000", "", q, xyz);
  SimSensor::Ptr d455_sensor =
      testutils::CreateSimSensor("INTEL_REALSENSE_D455_000000000000", "", q, xyz);
  SimSensor::Ptr l515_sensor =
      testutils::CreateSimSensor("INTEL_REALSENSE_L515_0000000000000000", "", q, xyz);

  RobotObserver::Ptr observer = testutils::CreateRobotObserver(testutils::UR5_MODEL_NAME);
  RobotMaskGenerator::MeshMap mesh_map;
  ASSERT_TRUE(testutils::CreateMeshMap(observer, mesh_map));

  cv::Mat d435_mat = TestRender(d435_sensor, observer, mesh_map);
  cv::Mat d435e_mat = TestRender(d435e_sensor, observer, mesh_map);
  cv::Mat d455_mat = TestRender(d455_sensor, observer, mesh_map);
  cv::Mat l515_mat = TestRender(l515_sensor, observer, mesh_map);

  EXPECT_TRUE(DepthImageEquals(d435_mat, d435e_mat));
  EXPECT_TRUE(DepthImageEquals(d435_mat, d455_mat));
  EXPECT_FALSE(DepthImageEquals(d435_mat, l515_mat));
}

//// Test that the sim sensor can render the fiducial correctly
TEST(SimSensor, FiducialDetection) {
  const std::string uid = "INTEL_REALSENSE_D435_000000000000";
  const SensorFrameType ft(SensorFrameType::SENSOR_FRAME_IMAGE_IR_STEREO_1,
                           SensorFrameType::RECTIFIED);

  const Eigen::Quaterniond q = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
  const Eigen::Vector3d xyz = Eigen::Vector3d(0.0, 0.0, 2.5);
  SensorExtrinsics::ConstPtr extr =
      SensorExtrinsics::MakePtr(SensorMetadata(uid, ft), q, xyz, "world");

  // set up sim sensor
  SimSensor::Ptr sensor = testutils::CreateSimSensor(uid, FLANGE_FRAME, q, xyz);
  SensorIntrinsics::ConstPtr intr = sensor->GetDefaultSimulatedIntrinsics(ft);
  sensor->SetIntrinsics({{ft, intr}});
  sensor->SetExtrinsics({{ft, extr}});

  // add robot
  RobotObserver::Ptr observer = testutils::CreateRobotObserver(testutils::UR10_MODEL_NAME);
  ASSERT_TRUE(observer);
  RobotMaskGenerator::MeshMap mesh_map;
  ASSERT_TRUE(testutils::CreateMeshMap(observer, mesh_map));
  sensor->AddRobot(observer, mesh_map);

  // load fiducial mesh and add to sim sensor
  std::vector<TriMesh::Ptr> fiducial_meshes;
  std::vector<Vec4> fiducial_colors;
  ASSERT_TRUE(LoadMesh("data/fiducial_plate.dae", fiducial_meshes, fiducial_colors));
  sensor->AddMesh(FIDUCIAL_MESH, fiducial_meshes);
  sensor->AddColor(FIDUCIAL_MESH, fiducial_colors);

  // test detection and correct pose computation
  const CalibrationTarget target;
  std::vector<cv::Point2f> corners;
  JointConfiguration config({0.f, -1.57f, 0.f, 0.f, 1.57f, 0.f});
  EXPECT_TRUE(DetectFiducial(observer, config, sensor, ft, target, corners));
  EXPECT_LT(ComputeReprojError(observer, sensor, ft, target, corners), REPROJ_TOLERANCE);

  config = JointConfiguration({0.f, -0.25f, -2.f, 0.5f, 1.57f, 0.f});
  EXPECT_TRUE(DetectFiducial(observer, config, sensor, ft, target, corners));
  EXPECT_LT(ComputeReprojError(observer, sensor, ft, target, corners), REPROJ_TOLERANCE);

  config = JointConfiguration({0.55f, -0.25f, -2.f, 0.5f, 1.87f, -0.2f});
  EXPECT_TRUE(DetectFiducial(observer, config, sensor, ft, target, corners));
  EXPECT_LT(ComputeReprojError(observer, sensor, ft, target, corners), REPROJ_TOLERANCE);

  // test failed detection
  config = JointConfiguration({0.f, -1.57f, -2.f, 0.f, 1.57f, 0.f});
  EXPECT_FALSE(DetectFiducial(observer, config, sensor, ft, target, corners));
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  QApplication app(argc, argv);
  return RUN_ALL_TESTS();
}
