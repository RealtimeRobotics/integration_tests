#include <QApplication>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ros/service.h>

#include "rtr_appliance/Appliance.hpp"
#include "rtr_test_harness/ApplianceTestHelper.hpp"
#include "rtr_test_harness/RapidSenseTestConfigs.hpp"

namespace rtr {
namespace perception {

class RapidSenseTestHelper : public ApplianceTestHelper {
public:
  RapidSenseTestHelper(ros::NodeHandle& nh);

  bool GetRapidSenseServerConfig(SpatialPerceptionProjectSchema& config);

  bool CheckRapidSenseServerConfig(RapidSenseTestConfig& config);

  bool SetRapidSenseServerConfig(RapidSenseTestConfig& config);

  bool GetRapidSenseServerHealth(RapidSenseHealth& health);

  bool CheckRapidSenseServerState(RapidSenseState& state);

};

} // perception
} // rtr
