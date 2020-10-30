#include "rtr_test_harness/RapidSenseTestConfigs.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <rtr_math/MathSchema.hpp>

namespace rtr {
namespace perception {

RapidSenseTestHubConfig::RapidSenseTestHubConfig()
    : SchemaBase("RapidSenseTestHubConfig", 0) {}

void RapidSenseTestHubConfig::AddFields() {
  SchemaBase::AddFields();
  AddSimple("active_robot", active_robot);
  AddSimple("state_space", state_space);
  AddSimple("hub_name", hub_name);
  AddObjectVector("joint_configs", joint_configs);
}

RapidSenseTestConfig::RapidSenseTestConfig()
    : SchemaBase("RapidSenseTestConfig", 0), sim_mode(false) {}

void RapidSenseTestConfig::AddFields() {
  SchemaBase::AddFields();
  AddSimple("deconfliction_group", decon_group);
  AddSimple("voxel_stream_name", voxel_stream_name);
  AddSimple("test_robot_filter", test_robot_filter);
  AddSimple("speed", speed);
  AddSimple("sim_mode", sim_mode);
  AddSchemaVector("hub_sequence", hub_sequence);
  AddSimpleMap("criteria_thresholds", thresholds);
}

RapidSenseTestResult::RapidSenseTestResult(const std::string &name)
    : result_(Result::SUCCESS), num_frames_(0) {
  std::vector<std::string> strs;
  boost::split(strs, name, boost::is_any_of("/"));
  name_ = strs.back();

  recorded_stats_ = {
      Statistic::TPR,         Statistic::FPR,         Statistic::FNR,
      Statistic::DILATED_TPR, Statistic::DILATED_FPR, Statistic::DILATED_FNR,
      Statistic::DYNAMIC_TPR, Statistic::DYNAMIC_FPR, Statistic::DYNAMIC_FNR};

  for (const auto &stat : recorded_stats_) {
    mean_[stat] = 0.0;
    stddev_[stat] = 0.0;
    min_[stat] = Inf;
    max_[stat] = -Inf;
  }
}

void RapidSenseTestResult::Update(const BenchmarkManager::MetricFrame &frame) {
  num_frames_++;

  Metrics::ConstPtr voxel_metrics = Metrics::CastConstPtr(frame.metrics);
  for (const auto &stat : recorded_stats_) {
    const float &res = voxel_metrics->rates.at(stat);

    mean_[stat] += res;
    stddev_[stat] += Sqr(res);

    min_[stat] = Min(res, min_[stat]);
    max_[stat] = Max(res, max_[stat]);
  }

  frames_.push_back(frame);
}

void RapidSenseTestResult::ComputeFinalResults(
    const RapidSenseTestConfig &test_config) {
  result_ = SUCCESS;
  test_robot_filter_ = test_config.test_robot_filter;

  for (const auto &stat : recorded_stats_) {
    mean_[stat] /= num_frames_;
    stddev_[stat] = Sqrt((stddev_[stat] / num_frames_) - Sqr(mean_[stat]));

    if (test_config.thresholds.count(stat)) {
      switch (stat) {
      case Statistic::DILATED_TPR: // fall through
      case Statistic::DYNAMIC_TPR: // fall through
      case Statistic::TPR:
        if (min_[stat] < test_config.thresholds.at(stat)) {
          result_ = CRITERIA_FAILURE;
        }
        break;
      case Statistic::DILATED_FPR: // fall through
      case Statistic::DYNAMIC_FPR: // fall through
      case Statistic::FPR:
        if (max_[stat] > test_config.thresholds.at(stat)) {
          result_ = CRITERIA_FAILURE;
        }
        break;
      case Statistic::DILATED_FNR: // fall through
      case Statistic::DYNAMIC_FNR: // fall through
      case Statistic::FNR:
        if (max_[stat] > test_config.thresholds.at(stat)) {
          result_ = CRITERIA_FAILURE;
        }
        break;
      default:
        break;
      }
    }
  }
}

void RapidSenseTestResult::Print() {
  RTR_INFO("Test: {}", name_);
  RTR_INFO("Result: {}", TestResultToString(result_));
  RTR_INFO("Test Robot Filter: {}", test_robot_filter_);
  RTR_INFO("Number of frames: {}", num_frames_);

  for (const auto &stat : recorded_stats_) {
    RTR_INFO("- {}:", BenchmarkStatisticToString(stat));
    RTR_INFO("    - Mean:   {}", mean_[stat]);
    RTR_INFO("    - Stddev: {}", stddev_[stat]);
    RTR_INFO("    - Min:    {}", min_[stat]);
    RTR_INFO("    - Max:    {}", max_[stat]);
  }

  RTR_INFO("");
}

std::string RapidSenseTestResult::CSVString() {
  std::stringstream ss;
  ss << "Name," << name_ << std::endl
     << "Result," << TestResultToString(result_) << std::endl
     << "Test Robot Filter," << test_robot_filter_ << std::endl
     << "Number of frames," << num_frames_ << std::endl;

  for (const auto &stat : recorded_stats_) {
    ss << BenchmarkStatisticToString(stat) << " Mean," << mean_[stat]
       << std::endl
       << BenchmarkStatisticToString(stat) << " Stddev," << stddev_[stat]
       << std::endl
       << BenchmarkStatisticToString(stat) << " Min," << min_[stat] << std::endl
       << BenchmarkStatisticToString(stat) << " Max," << max_[stat]
       << std::endl;
  }
  return ss.str();
}

std::string RapidSenseTestResult::TestResultToString(const Result &res) {
  switch (res) {
  case SUCCESS:
    return "Success";
  case CRITERIA_FAILURE:
    return "Criteria Failure";
  case OPERATION_FAILURE:
    return "Operation Failure";
  case PATH_FAILURE:
    return "Path Failure";
  }
  return "";
}

} // namespace perception
} // namespace rtr
