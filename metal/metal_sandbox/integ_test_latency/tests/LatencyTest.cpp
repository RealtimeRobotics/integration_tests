#include <numeric>
#include <string>
#include <vector>

#include <QApplication>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/Float32.h>

#include <rtr_appliance/Appliance.hpp>
#include <rtr_msgs/SchemaMessage.h>
#include <rtr_msgs/VoxelClusterList.h>
#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>
#include <rtr_perc_rapidsense_ros/RapidSenseStatus.hpp>
#include <rtr_perc_rapidsense_ros/Record.hpp>
#include <rtr_perc_sensors_ros/SensorMsgConverters.hpp>
#include <rtr_test_harness/RapidSenseTestHelper.hpp>
#include <rtr_utils/Logging.hpp>

using namespace rtr::perception;
using namespace rtr;

namespace bfs = boost::filesystem;

// This test only runs for recordings around 15 seconds
// This test only support cameras of the type INTEL_REALSENSE_D435

#define fuzzy_eval_eps 5
#define ms_diff_threshold 25 + fuzzy_eval_eps
#define last_diff_threshold 45 + fuzzy_eval_eps
#define ms_diff_mean 12 + fuzzy_eval_eps
#define ms_diff_std 0.2
#define last_diff_mean 29 + fuzzy_eval_eps
#define last_diff_std 0.2

class LatencyTestFixture : public ::testing::Test {
 protected:
  ros::NodeHandle nh_;
  RapidSenseFrontEndProxy proxy_;
  RapidSenseTestHelper appliance_;
  std::string decon_group_name, robot_name, project, rapidsense_data, records;

  void SetUp() override {
    // wait for appliance and rs server
    if (!ros::topic::waitForMessage<std_msgs::String>("/appliance_state", ros::Duration(30))) {
      RTR_ERROR("Timed out waiting for appliance");
      return;
    }
    if (!ros::topic::waitForMessage<rtr_msgs::SchemaMessage>("/rapidsense/health",
                                                             ros::Duration(30))) {
      RTR_ERROR("Timed out waiting for RapidSense server");
      return;
    }

    nh_.param<std::string>("decon_group_name", decon_group_name, "ur3_latency_test");
    nh_.param<std::string>("robot_name", robot_name, "ur3");
    nh_.param<std::string>("project", project, "../../");
    nh_.param<std::string>("rapidsense_data", rapidsense_data, "../../");
    nh_.param<std::string>("recordings", records, "../../");

    std::string TestPath = std::getenv("RTR_PERCEPTION_TEST_DATA_ROOT");
    RTR_INFO(
        "Succesfully accessed environment variable "
        "RTR_PERCEPTION_TEST_DATA_ROOT: {}",
        TestPath);

    project = TestPath + "/rapidsense_testdata/ur3_latency_test1/ur3_november_25.zip";
    rapidsense_data = TestPath + "/rapidsense_testdata/ur3_latency_test1/rapidsense_data/";
    records = TestPath + "/rapidsense_testdata/ur3_latency_test1/recordings/";
    RTR_INFO("Value of project={}", project);
    RTR_INFO("Value of rapidsense_data={}", rapidsense_data);

    std::string robot_param = TestPath + "/rapidsense_testdata/ur3_latency_test1/ur3.json";
    ASSERT_TRUE(appliance_.ClearApplianceDatabase());
    ASSERT_TRUE(appliance_.InstallProject(project));
    ASSERT_TRUE(appliance_.SetProjectRobotParam("ur3", robot_param));
    ASSERT_TRUE(appliance_.AddAllProjectsToDeconGroup(decon_group_name));
    ASSERT_TRUE(appliance_.SetVisionEnabled(decon_group_name, true));
    ASSERT_TRUE(appliance_.LoadGroup(decon_group_name));
    ASSERT_TRUE(appliance_.InitGroup(decon_group_name, "ur3", "default_state"));

    std::string rapidsense_state_directory;
    if (!proxy_.GetStateDirectory(rapidsense_state_directory)) {
      RTR_ERROR("Unable to get state directory from rapidsense");
    } else {
      RTR_INFO("Got state directory from rapidsense: {}", rapidsense_state_directory);
    }

    std::string rapidsense_data_directory =
        fmt::format("{}/{}/", rapidsense_state_directory, decon_group_name);
    std::string recording_directory =
        fmt::format("{}/{}/", rapidsense_state_directory, "recordings");

    CopyFolder(rapidsense_data, rapidsense_data_directory);
    CopyFolder(records, recording_directory);

    if (!ros::topic::waitForMessage<std_msgs::String>("/appliance_state", ros::Duration(30))) {
      RTR_ERROR("Timed out waiting for appliance");
    }
    if (!ros::topic::waitForMessage<rtr_msgs::SchemaMessage>("/rapidsense/health",
                                                             ros::Duration(30))) {
      RTR_ERROR("Timed out waiting for RapidSense server");
    }
  }
  void TearDown() override {}

 public:
  LatencyTestFixture()
      : nh_(""), proxy_(RapidSenseFrontEndProxy::ProxyHost::RAPIDSENSE_GUI), appliance_(nh_) {}
};

class LatencyTracker {
 public:
  std::mutex data_mutex_;
  std::vector<float> x_;

  void AddData(const float x) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    x_.push_back(x);
  }

  void GetData(std::vector<float>& x) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    x = x_;
  }

  float GetMean() {
    float sum = std::accumulate(x_.begin(), x_.end(), 0.0);
    float mean = sum / x_.size();
    return mean;
  }

  float GetStdDev() {
    float mean = GetMean();
    std::vector<double> diff(x_.size());
    std::transform(x_.begin(), x_.end(), diff.begin(), [mean](float y) { return y - mean; });
    float std_dev =
        std::sqrt(std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0)) / x_.size();
    return std_dev;
  }
};

TEST_F(LatencyTestFixture, VerifyLatency) {
  if (getenv("DISABLE_RS_FLAKYTESTS")) {
    RTR_INFO("Skipping flaky test LatencyTestFixture::VerifyLatency");
    return;
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();
  LatencyTracker vx_tracker, last_tracker;
  std::chrono::high_resolution_clock::time_point last_time =
      std::chrono::high_resolution_clock::now();
  int message = 0;

  boost::function<void(const rtr_msgs::VoxelClusterListConstPtr& vx)> vx_func =
      [&vx_tracker, &last_tracker, &last_time,
       &message](const rtr_msgs::VoxelClusterListConstPtr& vx) {
        std::chrono::high_resolution_clock::time_point end_time =
            std::chrono::high_resolution_clock::now();
        int64_t ms_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                              end_time - RosTimeToSystemTime(vx->timestamp))
                              .count();
        int64_t last_diff =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - last_time).count();
        last_time = RosTimeToSystemTime(vx->timestamp);

        RTR_INFO("ms_diff = {}, last_diff = {}", ms_diff, last_diff);
        message++;
        if (message > 5) {
          EXPECT_LT(ms_diff, ms_diff_threshold);
          EXPECT_LT(last_diff, last_diff_threshold);
          vx_tracker.AddData(ms_diff);
          last_tracker.AddData(last_diff);
        }
      };

  RTR_INFO("Subscribing to VoxelClusterList...");
  ros::Subscriber voxel_sub = nh_.subscribe<rtr_msgs::VoxelClusterList>(
      fmt::format("/rapidsense/streams/{}/voxels_filt", "ur3"), 1, vx_func, ros::VoidConstPtr(),
      ros::TransportHints().reliable().tcpNoDelay());
  std::thread playback_thread = std::thread(
      [this]() { proxy_.StartPlayback("ur3_latency_test_2020-10-23_15-57-32", 1.0, false); });
  playback_thread.join();

  EXPECT_LT(vx_tracker.GetMean(), ms_diff_mean);
  EXPECT_LT(vx_tracker.GetStdDev(), ms_diff_std);
  EXPECT_LT(last_tracker.GetMean(), last_diff_mean);
  EXPECT_LT(last_tracker.GetStdDev(), last_diff_std);
  RTR_INFO("Latency Test Finished!");
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("rapidsense_lat");
  ros::init(argc, argv, "LatencyTest");
  RTR_INFO("new test {}", argc);
  ::testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
  bfs::remove_all("/tmp/appliance_test");
  bfs::remove_all("/tmp/rapidsense_test");
  return res;
}
