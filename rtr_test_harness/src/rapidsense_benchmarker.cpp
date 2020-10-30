#include <rtr_perc_rapidsense_ros/RapidSenseFrontEndProxy.hpp>

#include "rtr_perc_rapidsense_ros/bench/BenchmarkManager.hpp"
#include "rtr_test_harness/RapidSenseTest.hpp"

using namespace rtr;
using namespace rtr::perception;

int main(int argc, char *argv[]) {
  rtr::InitializeLogging("rapidsense_benchmarker");
  ros::init(argc, argv, "rapidsense_benchmarker");

  ros::NodeHandle nh("");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  RapidSenseFrontEndProxy proxy;
  SpatialPerceptionProjectSchema schema;
  if (!proxy.GetConfiguration(schema).IsSuccess()) {
    RTR_ERROR("Failed to get config from server");
    return EXIT_FAILURE;
  }
  if (schema.streams.empty()) {
    RTR_ERROR("Failed to get voxel stream from server");
    return EXIT_FAILURE;
  }
  std::string stream = schema.streams.front().name;
  RTR_INFO("Benchmarking for stream {}", stream);

  if (!SetIgnoreVisionEnabledOnServer(true)) {
    return EXIT_FAILURE;
  }

  VoxelRegionDescription vrd(proxy.GetVoxelRegionNumVoxels(stream),
                             proxy.GetVoxelRegionDimensions(stream) / 2.0,
                             proxy.GetVoxelRegionPose(stream).Inverse());
  RosBenchmarkManager benchmarker(nh, stream, vrd);
  benchmarker.Init(proxy.GetObservers());

  ros::waitForShutdown();

  SetIgnoreVisionEnabledOnServer(false);
  return EXIT_SUCCESS;
}
