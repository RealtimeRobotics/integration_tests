#include <gtest/gtest.h>

#include <rtr_perc_api/SensorFrame.hpp>
#include <rtr_perc_spatial/RobotFilter.hpp>
#include <rtr_perc_spatial/RobotMaskGenerator.hpp>
#include <rtr_perc_spatial/StampedJointBuffer.hpp>
#include <rtr_utils/time/Timer.hpp>

using namespace rtr;
using namespace rtr::perception;

class FakeMaskGenerator : public RobotMaskGenerator {
  DEFINE_SMART_PTR(FakeMaskGenerator)

 public:
  FakeMaskGenerator() : RobotMaskGenerator(MeshMap(), nullptr) {}

  bool Init() override {
    return true;
  }

 protected:
  SensorFrame::ConstPtr GenerateMask(const SensorTime&) override {
    SensorMetadata meta("", SensorFrameType::SENSOR_FRAME_VOXELS);
    return SensorFrameVoxels::MakePtr(std::vector<Voxel>(),
                                      std::array<std::size_t, 3>({64, 64, 64}), meta);
  }
};

class FakeRobotFilter : public RobotFilter {
 public:
  FakeRobotFilter() {
    generator_ = FakeMaskGenerator::MakePtr();
    enable_ = true;
  }

  SensorFrame::ConstPtr GetFiltered(const SensorFrame::ConstPtr, SensorFrame::ConstPtr&) {
    return nullptr;
  }

  SensorFrame::ConstPtr GetFiltered(const SensorFrame::ConstPtr frame) {
    SensorFrame::ConstPtr mask_frame;
    if (GetMatchingMask(frame->getMetadata().getTimestamp(), mask_frame)) {
      return mask_frame;
    }
    return nullptr;
  }
};

// Test the timing mechanism on the robot filter base class
TEST(RobotFilter, RobotFilterFrameRateTest) {
  FakeRobotFilter filter;
  filter.Start();

  int sample_size = 20;
  std::vector<int64_t> ms_diffs;

  SensorMetadata meta("", SensorFrameType::SENSOR_FRAME_VOXELS);
  SensorFrame::Ptr frame = SensorFrameVoxels::MakePtr(
      std::vector<Voxel>(), std::array<std::size_t, 3>({64, 64, 64}), meta);

  rtr::Timer timer;
  int64_t max_time_diff = 0;
  for (int i = 0; i < sample_size; ++i) {
    timer.Start();

    auto time_point = std::chrono::high_resolution_clock::now()
                      - std::chrono::milliseconds(15);  // 15ms old frame
    meta.setTimestamp(time_point);
    frame->setMetadata(meta);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // random delay

    SensorFrame::ConstPtr new_frame = filter.GetFiltered(frame);
    if (new_frame && i != 0) {
      auto duration = time_point - new_frame->getMetadata().getTimestamp();
      ms_diffs.push_back(
          std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(7));  // random delay
    int64_t time_diff = timer.Elapsed() * 1000.0;
    max_time_diff = std::max(time_diff, max_time_diff);
  }

  // if the robot filter base class is functioning correctly,
  // the time difference between the mask and the current frame should always
  // be less than the max time difference between provided frames
  for (const auto& diff : ms_diffs) {
    EXPECT_LT(diff, max_time_diff);
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
