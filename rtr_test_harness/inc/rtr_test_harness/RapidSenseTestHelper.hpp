#include <set>
#include <string>
#include <vector>

#include <QApplication>

#include <gtest/gtest.h>
#include <ros/service.h>

#include <rtr_msgs/GetSchemaMessage.h>
#include <rtr_perc_rapidsense_ros/RapidSenseServer.hpp>
#include <rtr_perc_rapidsense_ros/SensorSimulator.hpp>

#include "rtr_test_harness/ApplianceTestHelper.hpp"
#include "rtr_test_harness/RapidSenseTestConfigs.hpp"

namespace rtr {
namespace perception {

class RapidSenseTestHelper : public ApplianceTestHelper {
 public:
  RapidSenseTestHelper(ros::NodeHandle& nh);
  ~RapidSenseTestHelper();

  bool SetupFixture_SimulatedSensors();

  // @brief Get rapidsense server config via ros
  bool GetRapidSenseServerConfig(SpatialPerceptionProjectSchema& config);

  // @brief Check rapidsense server config matches
  bool CheckRapidSenseServerConfig(RapidSenseTestConfig& config);

  // @brief Set rapidsense server to passed config
  bool SetRapidSenseServerConfig(RapidSenseTestConfig& config);

  // @brief Get health state from rapidsense server via ros
  bool GetRapidSenseServerHealth(RapidSenseHealth& health);

  // @brief Check rapidsense server health state matches
  bool CheckRapidSenseServerState(RapidSenseState state);

  // @brief start the sensor simulator and restart sim
  bool StartSensorSimulator();

 private:
  SensorSimulator::Ptr simulator_;
  ros::ServiceServer restart_sim_;
};

}  // namespace perception
}  // namespace rtr
