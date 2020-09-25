#ifndef RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETESTCONFIGS_HPP_
#define RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETESTCONFIGS_HPP_

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <rtr_math/Vec.hpp>
#include <rtr_perc_spatial/Benchmarker.hpp>
#include <rtr_utils/Schema.hpp>
#include <rtr_utils/SmartPtr.hpp>

#include "rtr_test_harness/BenchmarkManager.hpp"

namespace rtr {
namespace perception {

struct RapidSenseTestHubConfig : public SchemaBase {
  RapidSenseTestHubConfig();

  void AddFields();

  std::string active_robot;
  std::string state_space;
  std::string hub_name;
  std::vector<Vec> joint_configs;
};

struct RapidSenseTestConfig : public SchemaBase {
  RapidSenseTestConfig();

  void AddFields();

  std::string decon_group;
  std::string voxel_stream_name;
  bool test_robot_filter;
  bool sim_mode;
  float speed;

  std::vector<RapidSenseTestHubConfig> hub_sequence;
  std::map<Benchmarker::Metrics::Statistic, float> thresholds;
};

class RapidSenseTestResult {
  DEFINE_SMART_PTR(RapidSenseTestResult)

 public:
  typedef Benchmarker::VoxelMetrics Metrics;
  typedef Benchmarker::Metrics::Statistic Statistic;
  typedef std::map<Statistic, float> StatSet;

  enum Result { SUCCESS, CRITERIA_FAILURE, OPERATION_FAILURE, PATH_FAILURE };

  RapidSenseTestResult(const std::string& name);

  void Update(const BenchmarkManager::MetricFrame& frame);
  void ComputeFinalResults(const RapidSenseTestConfig& test_config);
  void Print();
  std::string CSVString();

  static std::string TestResultToString(const Result& res);

  std::string name_;
  bool test_robot_filter_;
  Result result_;
  size_t num_frames_;

  std::set<Statistic> recorded_stats_;
  StatSet mean_, stddev_, min_, max_;
  std::vector<BenchmarkManager::MetricFrame> frames_;
};

}  // namespace perception
}  // namespace rtr

#endif  // RTR_APPS_RTR_SPATIAL_PERCEPTION_INC_RTR_SPATIAL_PERCEPTION_RAPIDSENSETESTCONFIGS_HPP_
