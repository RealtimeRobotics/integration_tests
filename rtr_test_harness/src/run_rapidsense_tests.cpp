#include <rtr_math/Random.hpp>

#include "rtr_test_harness/RapidSenseTest.hpp"

using namespace rtr::perception;
using namespace rtr;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cout << "Usage: RunRapidSenseTests <dir name>" << std::endl;
    return EXIT_FAILURE;
  }

  if (!boost::filesystem::exists(argv[1]) || !boost::filesystem::is_directory(argv[1])) {
    std::cout << "Directory " << argv[1] << " does not exist" << std::endl;
    return EXIT_FAILURE;
  }

  rtr::InitializeLogging("RunRapidSenseTests");
  ros::init(argc, argv, "RunRapidSenseTests");
  ros::NodeHandle nh("");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::vector<std::string> test_dirs;
  boost::filesystem::directory_iterator end_it;
  for (boost::filesystem::directory_iterator it(argv[1]); it != end_it; ++it) {
    if (boost::filesystem::is_directory(it->path())) {
      test_dirs.push_back(it->path().string());
      RTR_INFO("Found test directory {}", it->path().string());
    }
  }

  if (test_dirs.empty()) {
    RTR_INFO("No tests found. Generating example test");
    std::string dir = fmt::format("{}/example-test", argv[1]);
    if (!boost::filesystem::create_directory(dir)) {
      RTR_ERROR("Could not create directory {}", dir);
      return EXIT_FAILURE;
    }

    RapidSenseTestConfig config;
    config.SetName("example-test");
    config.test_robot_filter = false;
    config.speed = 0.1;
    RapidSenseTestHubConfig hub_config;
    hub_config.hub_name = "home";
    config.hub_sequence.push_back(hub_config);
    config.thresholds[Benchmarker::Metrics::TPR] = 0.8;
    config.thresholds[Benchmarker::Metrics::FPR] = 0.05;
    config.thresholds[Benchmarker::Metrics::FNR] = 0.05;
    config.thresholds[Benchmarker::Metrics::DILATED_TPR] = 1.0;
    config.thresholds[Benchmarker::Metrics::DILATED_FNR] = 0.01;
    config.thresholds[Benchmarker::Metrics::LATENCY] = 50;
    config.thresholds[Benchmarker::Metrics::FPS] = 85;

    if (!config.Serialize(dir + "/test.xml")) {
      RTR_ERROR("Failed to serialize file {}", dir + "/test.xml");
      return EXIT_FAILURE;
    }
  } else {
    for (const auto& dir : test_dirs) {
      RapidSenseTest test;
      if (test.Init(dir)) {
        test.Run(true, true);
      }
    }
  }

  return EXIT_SUCCESS;
}
