#ifndef RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_BENCHMARKMANAGER_HPP_
#define RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_BENCHMARKMANAGER_HPP_

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

#include <rtr_msgs/VoxelCluster.h>
#include <rtr_msgs/VoxelClusterList.h>
#include <rtr_perc_api/VoxelCluster.hpp>
#include <rtr_perc_sensors_ros/SensorMsgConverters.hpp>
#include <rtr_perc_spatial/Benchmarker.hpp>
#include <rtr_utils/SmartPtr.hpp>

#include "rtr_test_harness/MetricPublisher.hpp"

namespace rtr {
namespace perception {

/************************************************************************************************
 * BenchmarkManager                                                                             *
 *                                                                                              *
 * Subscribes to voxel frames, updates the ground truth robot model to match timestamps with    *
 * the voxel frame, and runs benchmarking on the received frame                                 *
 ************************************************************************************************/
class BenchmarkManager {
  DEFINE_SMART_PTR_ABSTRACT(BenchmarkManager)

 public:
  struct MetricFrame {
    std::map<std::string, JointConfiguration> configs;
    Benchmarker::VoxelMetrics::ConstPtr metrics;
  };

  typedef std::function<void(const MetricFrame&)> OnMetricData;
  typedef Benchmarker::Metrics Metrics;
  typedef Metrics::Statistic Statistic;

  BenchmarkManager(const VoxelRegionDescription& vrd, const MetricPublisher::Ptr publisher,
                   const int buffer_limit);
  BenchmarkManager(const VoxelRegionDescription& vrd, const MetricPublisher::Ptr publisher);

  virtual ~BenchmarkManager();

  //! @brief initialize benchmarker to use robots
  bool Init(const std::vector<RobotObserver::Ptr>& observers);

  //! @brief Subscribe to benchmarking metrics. If there are no subscribers, the BenchmarkManager
  //         discards the metric results immediately after publishing
  std::string SubscribeToMetrics(OnMetricData on_data);

  //! @brief Unsubscribe from benchmarking metrics
  void UnsubscribeFromMetrics(const std::string& conn, const bool wait_for_empty);

  //! @brief Read timestamp, update ground truth robot with correct timestamp, and then benchmark
  void ProcessFrame(const SensorFrameVoxels::ConstPtr& frame);

  //! @brief Clear frame buffer - will not benchmark on frames which are cleared
  void ResetFrameBuffer();

  //! @brief Enable robot in benchmarking
  void SetRobotBenchmarkEnable(const bool enable);

 protected:
  virtual SensorFrame::ConstPtr InitVoxelFrameSubscriber() = 0;

  MetricPublisher::Ptr publisher_;

  VoxelRegionDescription region_desc_;
  std::array<std::size_t, 3> resolution_;

  BufferInterface<SensorFrameVoxels::ConstPtr>::Ptr frame_buffer_;
  DataSubscriberManagerThreaded<SensorFrameVoxels::ConstPtr>::Ptr frame_manager_;
  std::string sub_id_;

  std::atomic_int num_subscribers_;
  BufferInterface<BenchmarkManager::MetricFrame>::Ptr metrics_buffer_;
  DataSubscriberManagerThreaded<BenchmarkManager::MetricFrame>::Ptr metrics_manager_;

  std::vector<RobotObserver::Ptr> observers_;
  VoxelBenchmarker benchmarker_;

  std::atomic_bool init_;
};

/************************************************************************************************
 * RosBenchmarkManager                                                                          *
 *                                                                                              *
 * Subscribes to voxel frames, updates the ground truth robot model to match timestamps with    *
 * the voxel frame, and runs benchmarking on the received frame                                 *
 ************************************************************************************************/
class RosBenchmarkManager : public BenchmarkManager {
  DEFINE_SMART_PTR(RosBenchmarkManager)

 public:
  RosBenchmarkManager(ros::NodeHandle& nh, const std::string& stream_name,
                      const VoxelRegionDescription& vrd, const int buffer_limit);
  RosBenchmarkManager(ros::NodeHandle& nh, const std::string& stream_name,
                      const VoxelRegionDescription& vrd);

  //! @brief Called on voxel frame to add voxels to buffer
  void FrameCallback(const rtr_msgs::VoxelClusterList::ConstPtr& frame);

 protected:
  SensorFrame::ConstPtr InitVoxelFrameSubscriber() override;

  ros::Subscriber frame_sub_;
};

}  // namespace perception
}  // namespace rtr

#endif  // RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_BENCHMARKMANAGER_HPP_
