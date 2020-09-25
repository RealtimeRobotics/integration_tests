#include "rtr_test_harness/MetricPublisher.hpp"

#include <ros/ros.h>

#include <rtr_perc_rapidsense_ros/RapidSenseDefs.hpp>

namespace rtr {

namespace perception {

const std::map<MetricPublisher::Statistic, Vec4> MetricPublisher::ColorMap(
    {{Benchmarker::Metrics::Statistic::TPR, Vec4(0.0, 1.0, 0.0, 1.0)},
     {Benchmarker::Metrics::Statistic::FPR, Vec4(1.0, 1.0, 0.0, 1.0)},
     {Benchmarker::Metrics::Statistic::FNR, Vec4(1.0, 0.0, 0.0, 1.0)}});

MetricPublisher::MetricPublisher(const std::string& stream_name, const Vec3& scale)
    : stream_name_(stream_name), voxel_scale_(scale) {}

RosMetricPublisher::RosMetricPublisher(ros::NodeHandle& nh, const std::string& stream_name,
                                       const Vec3& scale)
    : MetricPublisher(stream_name, scale) {
  const std::vector<Statistic> marker_stats = {Statistic::TPR, Statistic::FPR, Statistic::FNR};
  ground_truth_marker_pub_ =
      nh.advertise<visualization_msgs::Marker>(stream_name + "/ground_truth_marker", 1);

  for (const auto& stat : marker_stats) {
    std::string topic_name =
        fmt::format("{}/{}_marker", stream_name, BenchmarkStatisticToString(stat));
    marker_pubs_[stat] = nh.advertise<visualization_msgs::Marker>(RS::Topic(topic_name), 1);
  }

  const std::vector<Statistic> rate_stats = {Statistic::TPR, Statistic::FPR, Statistic::FNR,
                                             Statistic::DILATED_TPR, Statistic::DILATED_FNR};
  for (const auto& stat : rate_stats) {
    std::string topic_name = fmt::format("{}/{}", stream_name, BenchmarkStatisticToString(stat));
    rate_pubs_[stat] = nh.advertise<std_msgs::Float32>(RS::Topic(topic_name), 1);
  }
}

void RosMetricPublisher::Publish(const Benchmarker::Metrics::ConstPtr metrics) {
  visualization_msgs::Marker gt_marker;
  MarkerMap markers;
  CreateMarkers(metrics, voxel_scale_, RS::VoxelRegionName(stream_name_), stream_name_, gt_marker,
                markers);

  // publish markers
  ground_truth_marker_pub_.publish(gt_marker);
  for (auto& pub : marker_pubs_) {
    if (!markers.count(pub.first)) continue;
    pub.second.publish(markers[pub.first]);
  }

  // publish rates
  Benchmarker::VoxelMetrics::ConstPtr voxel_metrics =
      Benchmarker::VoxelMetrics::CastConstPtr(metrics);
  for (auto& pub : rate_pubs_) {
    if (!voxel_metrics->rates.count(pub.first)) continue;
    std_msgs::Float32 msg;
    msg.data = voxel_metrics->rates.at(pub.first);
    pub.second.publish(msg);
  }
}

void RosMetricPublisher::CreateMarkers(const Benchmarker::Metrics::ConstPtr metrics,
                                       const Vec3& voxel_scale, const std::string& frame,
                                       const std::string& id_namespace,
                                       visualization_msgs::Marker& ground_truth,
                                       MarkerMap& markers) {
  int id = 0;

  Benchmarker::VoxelMetrics::ConstPtr voxel_metrics =
      Benchmarker::VoxelMetrics::CastConstPtr(metrics);

  if (!voxel_metrics) {
    RTR_ERROR("Tried to create voxel markers from invalid set of VoxelMetrics");
    return;
  }

  VoxelsToMarker(voxel_metrics->ground_truth_voxels, voxel_scale, Vec4(1.0, 0.0, 1.0, 1.0), frame,
                 id_namespace, id++, ground_truth);

  for (const auto& voxel_pair : voxel_metrics->voxels) {
    visualization_msgs::Marker marker;
    VoxelsToMarker(voxel_pair.second, voxel_scale, ColorMap.at(voxel_pair.first), frame,
                   id_namespace, id++, markers[voxel_pair.first]);
  }
}

void RosMetricPublisher::VoxelsToMarker(const std::vector<Voxel>& voxels, const Vec3& voxel_scale,
                                        const Vec4& color, const std::string& frame,
                                        const std::string& id_namespace, const int id,
                                        visualization_msgs::Marker& marker) {
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = id_namespace;
  marker.id = id;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = voxel_scale.x();
  marker.scale.y = voxel_scale.y();
  marker.scale.z = voxel_scale.z();
  marker.lifetime = ros::Duration(0.05);

  marker.points.resize(voxels.size());
  marker.colors.resize(voxels.size());

  std_msgs::ColorRGBA marker_color;
  marker_color.r = color.x();
  marker_color.g = color.y();
  marker_color.b = color.z();
  marker_color.a = color.w();
  std::fill(marker.colors.begin(), marker.colors.end(), marker_color);

  for (size_t i = 0; i < voxels.size(); ++i) {
    marker.points[i].x = (voxels[i].x + 0.5) * voxel_scale.x();
    marker.points[i].y = (voxels[i].y + 0.5) * voxel_scale.y();
    marker.points[i].z = (voxels[i].z + 0.5) * voxel_scale.z();
  }
}

}  // namespace perception

}  // namespace rtr
