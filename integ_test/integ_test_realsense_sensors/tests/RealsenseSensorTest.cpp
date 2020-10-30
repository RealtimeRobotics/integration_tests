#include <gtest/gtest.h>
#include <tbb2/parallel_for_each.h>

#include <rtr_perc_sensors/SensorManager.hpp>
#include <rtr_perc_sensors/SerialKiller.hpp>
#include <rtr_utils/Timer.hpp>

using namespace rtr;
using namespace rtr::perception;

enum SelectedFrameTypes { DEPTH, EXTRINSICS, INTRINSICS };

bool StartStopSensors(const std::set<std::string> &sensors,
                      const SelectedFrameTypes selected_types) {
  std::atomic_bool sensors_ok(true);
  tbb::parallel_for_each(
      sensors.begin(), sensors.end(),
      [&sensors_ok, &selected_types](const std::string &uid) {
        std::set<SensorFrameType> types;
        if (selected_types == SelectedFrameTypes::DEPTH) {
          types = {SensorFrameType::SENSOR_FRAME_IMAGE_DEPTH};
        } else if (selected_types == SelectedFrameTypes::EXTRINSICS) {
          types = {
              SensorManager::getInstance()->GetExtrinsicsCalibrationFrameType(
                  uid)};
        } else {
          types =
              SensorManager::getInstance()->GetIntrinsicsCalibrationFrameTypes(
                  uid);
        }

        if (!SensorManager::getInstance()->start(uid, types)) {
          sensors_ok = false;
        }
      });
  tbb::parallel_for_each(
      sensors.begin(), sensors.end(),
      [](const std::string &uid) { SensorManager::getInstance()->stop(uid); });
  return sensors_ok;
}

bool TestStartStopSensors(const std::set<std::string> &sensors,
                          const SelectedFrameTypes types,
                          const bool has_serial_killer) {
  // if sensors could not start / stop, recover them and try again
  // if serial killer is not present, log error but allow the test to pass
  if (!StartStopSensors(sensors, types)) {
    if (has_serial_killer) {
      RTR_ERROR("Failed to start and stop sensors. Attempting to recover "
                "malfunctioning devices");
      EXPECT_FALSE(
          SensorManager::getInstance()->RecoverMalfunctioningDevices().empty());
      EXPECT_TRUE(SensorManager::getInstance()->connect(sensors));
      return StartStopSensors(sensors, types);
    } else {
      RTR_ERROR("Failed to start and stop sensors, but no serial killer is "
                "present. Allowing test to "
                "pass");
    }
  }
  return true;
}

TEST(RealsenseSensor, SerialKillerReconnect) {
  if (getenv("DISABLE_RS_FLAKYTESTS")) {
    RTR_INFO("Skipping flaky test RealsenseSensor::SerialKillerReconnect");
    return;
  }

  std::string serial;
  if (!SerialKiller::GetInstance()->SerialNumber(serial) || serial.empty()) {
    RTR_WARN("No serial killer discovered. Skipping test "
             "RealsenseSensor::SerialKillerReconnect");
    return;
  }

  SensorManager::Ptr smgr = SensorManager::getInstance();
  smgr->initFactories({SensorManager::INTEL_REALSENSE});

  // check for sensors, skip if not available
  std::set<std::string> sensors = smgr->discover();
  if (sensors.empty()) {
    RTR_WARN("No sensors discovered. Skipping test "
             "RealsenseSensor::SerialKillerReconnect");
    return;
  }

  // check that sensors can be connected, skip if not possible
  if (!smgr->connect(sensors)) {
    smgr->disconnect(sensors);
    RTR_WARN("Sensors could not be connected. Firmware update may be required. "
             "Skipping test "
             "RealsenseSensor::NormalDeviceUsage");
    return;
  }

  //// Test SerialKiller functionality
  // power cycle serial killer
  ASSERT_TRUE(SerialKiller::GetInstance()->Toggle());
  smgr->initFactories({SensorManager::INTEL_REALSENSE});
  smgr->SetCalibrationDirectory("/tmp");
  std::size_t sensor_count = sensors.size();

  // wait for devices to reconnect
  sensors.clear();
  Timer timer;
  timer.Start();
  while (sensor_count != sensors.size() && timer.Elapsed() < 30.f) {
    sensors = smgr->discover();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  EXPECT_EQ(sensor_count, sensors.size());
  smgr->shutdown();
}

TEST(RealsenseSensor, NormalDeviceUsage) {
  std::string serial;
  const bool has_serial_killer =
      SerialKiller::GetInstance()->SerialNumber(serial) && !serial.empty();

  SensorManager::Ptr smgr = SensorManager::getInstance();
  smgr->initFactories({SensorManager::INTEL_REALSENSE});

  // check for sensors, skip if not available
  std::set<std::string> sensors = smgr->discover();
  if (sensors.empty()) {
    RTR_WARN("No sensors discovered. Skipping test "
             "RealsenseSensor::NormalDeviceUsage");
    return;
  }

  // check that sensors can be connected, skip if not possible
  if (!smgr->connect(sensors)) {
    smgr->disconnect(sensors);
    RTR_WARN("Sensors could not be connected. Firmware update may be required. "
             "Skipping test "
             "RealsenseSensor::NormalDeviceUsage");
    return;
  }

  RTR_INFO("Running Realsense DeviceUsage test for {} sensors", sensors.size());

  //// Test multi-threaded functionality in RealsenseDeviceManager
  // Discover sensors over and over in the background
  std::atomic_bool keep_going_(true);
  auto update_fun = [&smgr, &keep_going_]() {
    while (keep_going_.load()) {
      auto discovered = smgr->discover();
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
  };
  std::thread run_updates(update_fun);

  //// Test connect
  ASSERT_TRUE(smgr->connect(sensors));

  //// Test start / stop
  EXPECT_TRUE(TestStartStopSensors(sensors, SelectedFrameTypes::DEPTH,
                                   has_serial_killer));
  EXPECT_TRUE(TestStartStopSensors(sensors, SelectedFrameTypes::EXTRINSICS,
                                   has_serial_killer));
  EXPECT_TRUE(TestStartStopSensors(sensors, SelectedFrameTypes::INTRINSICS,
                                   has_serial_killer));

  keep_going_.store(false);
  if (run_updates.joinable()) {
    run_updates.join();
  }

  smgr->disconnect(sensors);
  smgr->shutdown();
}

int main(int argc, char **argv) {
  RapidSenseMessage::Setup(false);

  SensorManager::Init();
  SerialKiller::Init();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
