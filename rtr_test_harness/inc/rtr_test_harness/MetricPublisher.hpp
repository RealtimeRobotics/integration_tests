#ifndef RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_METRICPUBLISHER_HPP_
#define RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_METRICPUBLISHER_HPP_

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>

#include <rtr_perc_api/VoxelCluster.hpp>
#include <rtr_perc_sensors_ros/SensorMsgConverters.hpp>
#include <rtr_perc_spatial/Benchmarker.hpp>

namespace rtr {
namespace perception {

class MetricPublisher {
  DEFINE_SMART_PTR_ABSTRACT(MetricPublisher)

 public:
  typedef Benchmarker::Metrics Metrics;
  typedef Metrics::Statistic Statistic;
  typedef std::map<Statistic, visualization_msgs::Marker> MarkerMap;

  static const std::map<Statistic, Vec4> ColorMap;

  MetricPublisher(const std::string& stream_name, const Vec3& voxel_scale);

  virtual ~MetricPublisher() {}

  //! @brief Publish contents of Metrics
  virtual void Publish(const Benchmarker::Metrics::ConstPtr metrics) = 0;

  std::string stream_name_;
  Vec3 voxel_scale_;
};

/************************************************************************************************
 * RosMetricPublisher                                                                           *
 *                                                                                              *
 * Creates necessary publishers for Metrics. Publishes markers and rates when given frame       *
 ************************************************************************************************/
class RosMetricPublisher : public MetricPublisher {
  DEFINE_SMART_PTR(RosMetricPublisher)

 public:
  RosMetricPublisher(ros::NodeHandle& nh, const std::string& stream_name, const Vec3& voxel_scale);

  //! @brief Publish contents of Metrics
  void Publish(const Benchmarker::Metrics::ConstPtr metrics) override;

  static void CreateMarkers(const Benchmarker::Metrics::ConstPtr metrics, const Vec3& voxel_scale,
                            const std::string& frame, const std::string& id_namespace,
                            visualization_msgs::Marker& ground_truth, MarkerMap& markers);

  //! @brief Convert voxels to marker
  static void VoxelsToMarker(const std::vector<Voxel>& voxels, const Vec3& voxel_scale,
                             const Vec4& color, const std::string& frame,
                             const std::string& id_namespace, const int id,
                             visualization_msgs::Marker& marker);

  ros::Publisher ground_truth_marker_pub_;
  std::map<Statistic, ros::Publisher> marker_pubs_;
  std::map<Statistic, ros::Publisher> rate_pubs_;
};

}  // namespace perception
}  // namespace rtr

#endif  // RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_METRICPUBLISHER_HPP_
