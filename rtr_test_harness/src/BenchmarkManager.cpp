#include "rtr_test_harness/BenchmarkManager.hpp"

#include <rtr_perc_rapidsense_ros/RapidSenseDefs.hpp>

namespace rtr {

namespace perception {

BenchmarkManager::BenchmarkManager(const VoxelRegionDescription& vrd,
                                   const MetricPublisher::Ptr publisher, const int buffer_limit)
    : publisher_(publisher),
      region_desc_(vrd),
      resolution_(
          {(size_t) vrd.num_voxels.x(), (size_t) vrd.num_voxels.y(), (size_t) vrd.num_voxels.z()}),
      num_subscribers_(0),
      metrics_buffer_(BufferQueueT<MetricFrame>::MakePtr()),
      metrics_manager_(DataSubscriberManagerThreaded<MetricFrame>::MakePtr(metrics_buffer_)),
      init_(false) {
  auto get_time = [](const SensorFrameVoxels::ConstPtr& frame) {
    return frame->getMetadata().getTimestamp();
  };

  // subscribe to voxel cluster frames
  frame_buffer_ =
      BufferQueueWithAgingT<SensorFrameVoxels::ConstPtr>::MakePtr(get_time, buffer_limit);
  frame_manager_ =
      DataSubscriberManagerThreaded<SensorFrameVoxels::ConstPtr>::MakePtr(frame_buffer_);
  sub_id_ = frame_manager_->Subscribe(
      std::bind(&BenchmarkManager::ProcessFrame, this, std::placeholders::_1));
}

BenchmarkManager::BenchmarkManager(const VoxelRegionDescription& vrd,
                                   const MetricPublisher::Ptr publisher)
    : BenchmarkManager(vrd, publisher, 5000) {}

BenchmarkManager::~BenchmarkManager() {
  frame_manager_->Unsubscribe(sub_id_);
}

bool BenchmarkManager::Init(const std::vector<RobotObserver::Ptr>& observers) {
  observers_ = observers;
  std::vector<std::vector<std::string>> link_names;  // TODO: let users pick?
  for (const auto& observer : observers) {
    std::set<std::string> links = observer->GetMoveableChainLinks();
    link_names.push_back(std::vector<std::string>(links.begin(), links.end()));
  }

  if (!benchmarker_.Init(observers, link_names, region_desc_)) {
    RTR_ERROR("Benchmarker failed to initialize");
    return false;
  }

  SensorFrame::ConstPtr frame = InitVoxelFrameSubscriber();
  if (!frame) {
    return false;
  }

  if (!benchmarker_.InitStaticObstacles(frame)) {
    return false;
  }

  init_.store(true);
  return true;
}

std::string BenchmarkManager::SubscribeToMetrics(BenchmarkManager::OnMetricData on_data) {
  num_subscribers_.fetch_add(1);
  return metrics_manager_->Subscribe(on_data);
}

void BenchmarkManager::UnsubscribeFromMetrics(const std::string& conn, const bool wait_for_empty) {
  num_subscribers_.fetch_sub(1);

  while (wait_for_empty && !metrics_buffer_->empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  metrics_manager_->Unsubscribe(conn);

  if (!num_subscribers_.load()) {
    metrics_buffer_->Clear();
  }
}

void BenchmarkManager::ProcessFrame(const SensorFrameVoxels::ConstPtr& frame) {
  if (!init_.load() || !frame) {
    return;
  }

  Benchmarker::Metrics::Ptr metrics = benchmarker_.EvaluateFrame(frame);

  if (num_subscribers_.load()) {
    MetricFrame metric_frame;

    for (const auto& observer : observers_) {
      metric_frame.configs[observer->GetName()] =
          observer->GetJointConfiguration(frame->getMetadata().getTimestamp());
    }

    metric_frame.metrics = Benchmarker::VoxelMetrics::CastConstPtr(metrics);
    metrics_buffer_->add(metric_frame);
  }

  publisher_->Publish(metrics);
}

void BenchmarkManager::ResetFrameBuffer() {
  frame_buffer_->Clear();
}

void BenchmarkManager::SetRobotBenchmarkEnable(const bool enable) {
  benchmarker_.SetGroundTruthEnable(enable);
}

RosBenchmarkManager::RosBenchmarkManager(ros::NodeHandle& nh, const std::string& stream_name,
                                         const VoxelRegionDescription& vrd, const int buffer_limit)
    : BenchmarkManager(vrd, RosMetricPublisher::MakePtr(nh, stream_name, vrd.voxel_lengths),
                       buffer_limit),
      frame_sub_(nh.subscribe(
          RS::GetStreamTopicName(stream_name, "",
                                 SensorFrameType(SensorFrameType::SENSOR_FRAME_VOXELS,
                                                 SensorFrameType::ROBOT_SELF_FILTERED)),
          100, &RosBenchmarkManager::FrameCallback, this,
          ros::TransportHints().reliable().tcpNoDelay())) {}

RosBenchmarkManager::RosBenchmarkManager(ros::NodeHandle& nh, const std::string& stream_name,
                                         const VoxelRegionDescription& vrd)
    : RosBenchmarkManager(nh, stream_name, vrd, 5000) {}

void RosBenchmarkManager::FrameCallback(const rtr_msgs::VoxelClusterList::ConstPtr& frame) {
  SensorFrame::Ptr voxel_frame;
  if (!FromRosMessage(*frame, resolution_, voxel_frame)) {
    RTR_ERROR("Could not convert ROS voxel cluster list back to SensorFrame");
    return;
  }

  SensorFrameVoxels::ConstPtr voxel_frame_ptr = SensorFrameVoxels::CastConstPtr(voxel_frame);
  if (!voxel_frame_ptr) {
    RTR_ERROR("Failed to cast VoxelCluster message to voxel frame");
    return;
  }
  frame_buffer_->add(voxel_frame_ptr);
}

SensorFrame::ConstPtr RosBenchmarkManager::InitVoxelFrameSubscriber() {
  RTR_WARN("Waiting for topic on {}", frame_sub_.getTopic());
  rtr_msgs::VoxelClusterList::ConstPtr frame_msg =
      ros::topic::waitForMessage<rtr_msgs::VoxelClusterList>(frame_sub_.getTopic());
  SensorFrame::Ptr frame;
  if (!FromRosMessage(*frame_msg, resolution_, frame)) {
    RTR_ERROR("Could not convert ROS voxel cluster list back to SensorFrame");
    return nullptr;
  }
  return frame;
}

}  // namespace perception

}  // namespace rtr
