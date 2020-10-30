#include <functional>

#include <gtest/gtest.h>

#include <rtr_perc_api/Buffer.hpp>
#include <rtr_utils/Logging.hpp>

using namespace rtr;
using namespace rtr::perception;
using namespace std;

// A simple data structure for testing buffers
struct Temp {
  Temp() : time(std::chrono::high_resolution_clock::now()), index(0) {}
  Temp(const int idx)
      : time(std::chrono::high_resolution_clock::now()), index(idx) {}
  Temp(const int idx, const SensorTime timestamp)
      : time(timestamp), index(idx) {}

  SensorTime time;
  int index;
};

void EmptyBufferCheck(BufferInterface<Temp> &buffer) {
  buffer.Clear();
  Temp t;
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_FALSE(buffer.front(t));
  EXPECT_TRUE(buffer.empty());
  EXPECT_FALSE(buffer.get(t));
}

void GenerateData(std::vector<Temp> &data, const int &start, const int &end) {
  data.clear();
  for (int i = start; i < end; i++) {
    data.emplace_back(Temp(i));
  }
}

void AddData(BufferInterface<Temp> &buffer, const std::vector<Temp> &add_data,
             const int &delay_ms) {
  int num_data = add_data.size();
  for (int i = 0; i < num_data; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(delay_ms * 1000));
    Temp t(add_data[i].index);
    buffer.add(t);
  }
}

void RemoveData(BufferInterface<Temp> &buffer, const int &num_remove,
                std::vector<Temp> &removed_data, const int &delay_ms) {
  removed_data.clear();
  for (int i = 0; i < num_remove; i++) {
    Temp t;
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    if (!buffer.get(t)) {
      i--;
      continue;
    }
    removed_data.emplace_back(t);
  }
}

void BufferAddRemoveElementCheck(BufferInterface<Temp> &buffer) {
  buffer.Clear();
  Temp t;
  buffer.add(Temp(-2));
  EXPECT_EQ(buffer.size(), 1u);
  EXPECT_TRUE(buffer.front(t));
  EXPECT_EQ(t.index, -2);
  t.index = 0;
  EXPECT_TRUE(buffer.get(t));
  EXPECT_EQ(t.index, -2);
}

void BufferAddDataCheck(BufferInterface<Temp> &buffer,
                        const std::vector<Temp> &add_data,
                        const int &size_after_add) {
  AddData(buffer, add_data, 0);
  EXPECT_FALSE(buffer.empty());
  EXPECT_EQ(buffer.size(), size_t(size_after_add));
}

void BufferRemoveArrayCheck(BufferInterface<Temp> &buffer,
                            const std::vector<Temp> &removed_data_gt,
                            const int &size_after_remove) {
  std::vector<Temp> removed_data;
  int num_remove = removed_data_gt.size();
  RemoveData(buffer, num_remove, removed_data, 0);
  EXPECT_EQ(size_t(num_remove), removed_data.size());

  // check with ground truth
  for (int i = 0; i < num_remove; i++) {
    EXPECT_EQ(removed_data[i].index, removed_data_gt[i].index);
  }
  EXPECT_EQ(buffer.size(), size_t(size_after_remove));
}

void FIFOBufferCheck(BufferInterface<Temp> &buffer) {
  buffer.Clear();
  Temp t1, t2;
  AddData(buffer, {Temp(3), Temp(4), Temp(5)}, 0);
  EXPECT_TRUE(buffer.front(t1));
  EXPECT_TRUE(buffer.get(t2));
  EXPECT_EQ(t1.index, 3);
  EXPECT_EQ(t1.index, t2.index);
  EXPECT_TRUE(buffer.front(t1));
  EXPECT_TRUE(buffer.get(t2));
  EXPECT_EQ(t1.index, 4);
  EXPECT_EQ(t1.index, t2.index);
}

void BufferWaitForCheck(BufferInterface<Temp> &buffer) {
  if (getenv("DISABLE_RS_FLAKYTESTS")) {
    RTR_INFO("Skipping buffer WaitFor Check");
    return;
  }
  buffer.Clear();
  Temp t, t_add(51);
  std::thread add_thread(
      std::bind(AddData, std::ref(buffer), std::vector<Temp>(1, t_add), 2));
  EXPECT_TRUE(buffer.empty());
  EXPECT_FALSE(buffer.wait_for(t, 0.001));
  EXPECT_NE(t.index, t_add.index);
  EXPECT_TRUE(buffer.wait_for(t, 0.002));
  EXPECT_EQ(t.index, t_add.index);
  EXPECT_FALSE(buffer.wait_for(t, 0.001));

  if (add_thread.joinable()) {
    add_thread.join();
  }
}

void AgingBufferCheck(BufferInterface<Temp> &buffer) {
  if (getenv("DISABLE_RS_FLAKYTESTS")) {
    RTR_INFO("Skipping Buffer Aging Data Check");
    return;
  }

  buffer.Clear();
  int num_erased = buffer.numErased();
  std::vector<Temp> data;
  GenerateData(data, 1, 11);

  AddData(buffer, data, 10);

  std::this_thread::sleep_for(std::chrono::milliseconds(405));

  Temp t;
  EXPECT_TRUE(buffer.get(t));
  EXPECT_EQ(buffer.size(), 9u);
  EXPECT_EQ(t.index, 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_TRUE(buffer.get(t));
  EXPECT_EQ(buffer.size(), 6u);
  EXPECT_EQ(t.index, 4);
  EXPECT_EQ(buffer.numErased(), size_t(num_erased + 2));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_EQ(buffer.numErased(), size_t(num_erased + 8));
}

/*******************************************************************************
 * BufferQueueT Test
 ******************************************************************************/

TEST(BufferTest, BufferQueueT) {
  RTR_INFO("Starting BufferQueueT test");
  BufferQueueT<Temp> buffer;

  EmptyBufferCheck(buffer);
  BufferAddRemoveElementCheck(buffer);
  std::vector<Temp> array_data;
  GenerateData(array_data, 0, 100);
  BufferAddDataCheck(buffer, array_data, 100);
  BufferRemoveArrayCheck(buffer, array_data, 0);
  FIFOBufferCheck(buffer);
  BufferWaitForCheck(buffer);
  // default value
  EXPECT_EQ(buffer.numErased(), 0u);
  RTR_INFO("Ending BufferQueueT test");
}

/*******************************************************************************
 * BufferQueueT Multi-Threaded Test
 ******************************************************************************/

TEST(BufferTest, BufferQueueTMultiThreaded) {
  RTR_INFO("Starting BufferQueueT Multi-Threaded test");
  BufferQueueT<Temp> buffer;

  int add1 = 0;
  int add2 = 0;
  std::vector<Temp> data1, data2, data_removed;
  GenerateData(data1, 1000, 2000);
  GenerateData(data2, 3000, 4000);

  std::thread q1(std::bind(&AddData, std::ref(buffer), data1, 2));
  std::thread q2(std::bind(&AddData, std::ref(buffer), data2, 2));
  std::thread q3(std::bind(&RemoveData, std::ref(buffer), 1000,
                           std::ref(data_removed), 4));

  if (q1.joinable()) {
    q1.join();
  }
  if (q2.joinable()) {
    q2.join();
  }
  if (q3.joinable()) {
    q3.join();
  }

  for (const Temp &dat : data_removed) {
    int val = dat.index;
    if (val >= 1000 && val < 2000) {
      add1 += 1;
    } else if (val >= 3000 && val < 4000) {
      add2 += 1;
    } else {
      FAIL() << "Buffer has invalid values, that should not have been there.";
    }
  }

  EXPECT_FALSE(buffer.empty());
  EXPECT_NE(add1, 1000);
  EXPECT_NE(add2, 1000);
  EXPECT_EQ(buffer.size(), 1000u);

  RemoveData(buffer, 1000, data_removed, 0);

  for (const Temp &dat : data_removed) {
    int val = dat.index;
    if (val >= 1000 && val < 2000) {
      add1 += 1;
    } else if (val >= 3000 && val < 4000) {
      add2 += 1;
    } else {
      FAIL() << "Buffer has invalid values, that should not have been there.";
    }
  }

  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(add1, 1000);
  EXPECT_EQ(add2, 1000);

  RTR_INFO("End of BufferQueueT Multi-Threaded test");
}

/*******************************************************************************
 * BufferLastT Test
 ******************************************************************************/

TEST(BufferTest, BufferLastT) {
  RTR_INFO("Starting BufferLastT test");
  BufferLastT<Temp> buffer;

  EmptyBufferCheck(buffer);
  BufferAddRemoveElementCheck(buffer);
  std::vector<Temp> data;
  GenerateData(data, 0, 100);
  BufferAddDataCheck(buffer, data, 1);
  BufferRemoveArrayCheck(buffer, {Temp(99)}, 0);
  BufferWaitForCheck(buffer);
  // default value
  EXPECT_EQ(buffer.numErased(), 0u);
  RTR_INFO("Ending BufferLastT test");
}

/*******************************************************************************
 * BufferLastT Multi-Threaded Test
 ******************************************************************************/

TEST(BufferTest, BufferLastTMultithreaded) {
  RTR_INFO("Starting BufferLastT Multi-Threaded test");
  BufferLastT<Temp> buffer;

  std::vector<Temp> data1, data2, data_removed;
  GenerateData(data1, 1000, 2000);
  GenerateData(data2, 3000, 4000);

  std::thread q1(std::bind(&AddData, std::ref(buffer), data1, 2));
  std::thread q2(std::bind(&AddData, std::ref(buffer), data2, 2));
  std::thread q3(
      std::bind(&RemoveData, std::ref(buffer), 1, std::ref(data_removed), 40));

  if (q1.joinable()) {
    q1.join();
  }
  if (q2.joinable()) {
    q2.join();
  }
  if (q3.joinable()) {
    q3.join();
  }

  EXPECT_EQ(data_removed.size(), 1u);
  int val = data_removed[0].index;
  if (!(val >= 1000 && val < 2000) && !(val >= 3000 && val < 4000)) {
    FAIL() << "Buffer has invalid values, that should not have been there.";
  }

  Temp t;
  EXPECT_FALSE(buffer.empty());
  EXPECT_EQ(buffer.size(), 1u);
  EXPECT_TRUE(buffer.get(t));
  EXPECT_TRUE(t.index == 1999 || t.index == 3999);
  EXPECT_EQ(buffer.size(), 0u);
  EXPECT_TRUE(buffer.empty());

  RTR_INFO("End of BufferLastT test");
}

#if 0

// todo: re-enable this later! with changes to BufferQueueWithAgingT
// fix bug.

/*******************************************************************************
 * BufferQueueWithAgingT Test
 ******************************************************************************/

const int AGE_DECAY_TIME_MS = 500;

TEST(BufferTest, BufferQueueWithAgingT) {
  RTR_INFO("Starting BufferQueueWithAgingT test");
  auto get_time = [](const Temp& tp) { return tp.time; };
  BufferQueueWithAgingT<Temp> buffer(get_time, AGE_DECAY_TIME_MS);

  EmptyBufferCheck(buffer);
  BufferAddRemoveElementCheck(buffer);
  std::vector<Temp> array_data;
  GenerateData(array_data, 0, 100);
  BufferAddDataCheck(buffer, array_data, 100);
  BufferRemoveArrayCheck(buffer, array_data, 0);
  FIFOBufferCheck(buffer);
  BufferWaitForCheck(buffer);
  EXPECT_EQ(buffer.numErased(), 0u);

  AgingBufferCheck(buffer);

  RTR_INFO("Ending BufferQueueWithAgingT test");
}

/*******************************************************************************
 * BufferQueueWithAgingT Multi-Threaded Test
 ******************************************************************************/

TEST(BufferTest, BufferQueueWithAgingTMultiThreaded) {
  RTR_INFO("Starting BufferQueueWithAgingT Multi-Threaded test");

  auto get_time = [](const Temp& tp) { return tp.time; };
  BufferQueueWithAgingT<Temp> buffer(get_time, AGE_DECAY_TIME_MS);

  EXPECT_EQ(buffer.numErased(), 0u);

  std::vector<Temp> data1, data2, data_removed;
  GenerateData(data1, 100, 200);
  GenerateData(data2, 300, 400);

  std::thread q1(std::bind(&AddData, std::ref(buffer), data1, 10));
  std::thread q2(std::bind(&AddData, std::ref(buffer), data2, 10));

  if (q1.joinable()) {
    q1.join();
  }
  if (q2.joinable()) {
    q2.join();
  }

  int total_data = buffer.size() + buffer.numErased();
  EXPECT_NE(buffer.numErased(), 0u);
  EXPECT_EQ(total_data, 200);

  Temp t;
  EXPECT_TRUE(buffer.get(t));
  int val = t.index;
  if (!(val >= 100 && val < 200) && !(val >= 300 && val < 400)) {
    FAIL() << "Buffer has invalid values, that should not have been there.";
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  EXPECT_TRUE(buffer.empty());
  total_data = buffer.size() + buffer.numErased() + 1;
  EXPECT_EQ(total_data, 200);

  RTR_INFO("End of BufferQueueWithAgingT Multi-Threaded test");
}

#endif

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  RTR_INFO("Running Buffer tests...");
  return RUN_ALL_TESTS();
}
