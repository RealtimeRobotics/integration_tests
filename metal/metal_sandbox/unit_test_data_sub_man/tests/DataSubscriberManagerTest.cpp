#include <functional>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rtr_perc_api/Buffer.hpp>
#include <rtr_perc_api/DataSubscriberManager.hpp>
#include <rtr_utils/Logging.hpp>

using namespace rtr;
using namespace rtr::perception;
using namespace std;

struct DataFixture {
  DataFixture() : time(std::chrono::high_resolution_clock::now()), index(0) {}
  DataFixture(const int idx) : time(std::chrono::high_resolution_clock::now()), index(idx) {}
  DataFixture(const int idx, const SensorTime timestamp) : time(timestamp), index(idx) {}
  bool operator==(const DataFixture& p) const {
    return p.index == index;
  }
  SensorTime time;
  int index;
};

class TestFixture {
  DEFINE_SMART_PTR_ABSTRACT(TestFixture)

 private:
  BufferQueueT<DataFixture>::Ptr buffer_;
  DataSubscriberManagerThreaded<DataFixture>::Ptr manager_;
  std::vector<DataFixture> data_;

 public:
  TestFixture() {
    buffer_ = BufferQueueT<DataFixture>::MakePtr();
    EXPECT_TRUE(buffer_->empty());
    manager_ = DataSubscriberManagerThreaded<DataFixture>::MakePtr(buffer_);
    EXPECT_TRUE(buffer_->empty());
  }

  void GenerateData(int start, int end) {
    data_.clear();
    for (int i = start; i < end; i++) {
      data_.emplace_back(DataFixture(i));
    }
  }

  void AddDataToBuffer(int start, int end) {
    GenerateData(start, end);
    int num_data = data_.size();
    for (int i = 0; i < num_data; i++) {
      DataFixture t(data_[i].index);
      buffer_->add(t);
    }
  }

  std::vector<DataFixture> GetData() {
    return data_;
  }

  DataSubscriberManagerThreaded<DataFixture>::Ptr GetManager() {
    return manager_;
  }

  void CheckBufferEmpty() {
    EXPECT_TRUE(buffer_->empty());
  }
};

TEST(DataSubscriberManagerTest, TestSingleSubscriber) {
  TestFixture base;
  std::vector<DataFixture> result;
  auto on_data = [&result](const DataFixture& value) { result.push_back(value); };
  std::string sub_id = base.GetManager()->Subscribe(on_data);
  EXPECT_FALSE(sub_id.empty());

  base.AddDataToBuffer(1, 5);
  std::this_thread::sleep_for(std::chrono::seconds(5));
  base.GetManager()->Unsubscribe(sub_id);
  base.CheckBufferEmpty();
  EXPECT_EQ(base.GetData(), result);
}

TEST(DataSubscriberManagerTest, TestMultipleSubscribers) {
  TestFixture base;
  std::mutex data_mutex;
  std::vector<DataFixture> result1, result2, result3;
  auto on_data1 = [&result1, &data_mutex](const DataFixture& value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    result1.push_back(value);
  };
  std::string sub_id1 = base.GetManager()->Subscribe(on_data1);
  auto on_data2 = [&result2, &data_mutex](const DataFixture& value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    result2.push_back(value);
  };
  std::string sub_id2 = base.GetManager()->Subscribe(on_data2);
  auto on_data3 = [&result3, &data_mutex](const DataFixture& value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    result3.push_back(value);
  };
  std::string sub_id3 = base.GetManager()->Subscribe(on_data3);
  std::set<std::string> id_set = {sub_id1, sub_id2, sub_id3};
  EXPECT_FALSE(sub_id1.empty());  // Testing that no id is empty
  EXPECT_FALSE(sub_id2.empty());
  EXPECT_FALSE(sub_id3.empty());
  EXPECT_EQ(id_set.size(), 3u);

  base.AddDataToBuffer(1, 500);
  base.GetManager()->Unsubscribe(sub_id1);
  EXPECT_TRUE(result1.empty());
  std::this_thread::sleep_for(std::chrono::seconds(5));
  base.GetManager()->Unsubscribe(sub_id2);
  base.GetManager()->Unsubscribe(sub_id3);
  EXPECT_EQ(base.GetData(), result2);
  EXPECT_EQ(base.GetData(), result3);
  base.CheckBufferEmpty();
}

TEST(DataSubscriberManagerTest, PushAndNotifySubscribersTest) {
  TestFixture base;
  std::mutex data_mutex;
  std::vector<DataFixture> result1, result2, result3;
  auto on_data1 = [&result1, &data_mutex](const DataFixture& value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    result1.push_back(value);
  };
  std::string sub_id1 = base.GetManager()->Subscribe(on_data1);
  auto on_data2 = [&result2, &data_mutex](const DataFixture& value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    result2.push_back(value);
  };
  std::string sub_id2 = base.GetManager()->Subscribe(on_data2);
  auto on_data3 = [&result3, &data_mutex](const DataFixture& value) {
    std::lock_guard<std::mutex> lock(data_mutex);
    result3.push_back(value);
  };
  std::string sub_id3 = base.GetManager()->Subscribe(on_data3);
  std::set<std::string> id_set = {sub_id1, sub_id2, sub_id3};
  EXPECT_FALSE(sub_id1.empty());  // Testing that no id is empty
  EXPECT_FALSE(sub_id2.empty());
  EXPECT_FALSE(sub_id3.empty());
  EXPECT_EQ(id_set.size(), 3u);

  base.AddDataToBuffer(1, 500);
  base.GetManager()->Unsubscribe(sub_id1);
  EXPECT_TRUE(result1.empty());
  std::this_thread::sleep_for(std::chrono::seconds(5));
  base.GetManager()->Push(DataFixture(501));
  base.GetManager()->Push(DataFixture(502));
  std::this_thread::sleep_for(std::chrono::seconds(2));
  base.GetManager()->Unsubscribe(sub_id2);
  base.GetManager()->Push(DataFixture(503));
  std::this_thread::sleep_for(std::chrono::seconds(1));
  base.GetManager()->Unsubscribe(sub_id3);
  EXPECT_EQ(result2.size(), base.GetData().size() + 2);
  EXPECT_EQ(result3.size(), base.GetData().size() + 3);
  base.CheckBufferEmpty();
}

TEST(DataSubscriberManagerTest, MultithreadedSubscribing1) {
  TestFixture base;
  std::mutex data_mutex;
  std::vector<DataFixture> result1, result2, result3;
  std::string sub_id1, sub_id2, sub_id3;
  std::thread t1([&result1, &sub_id1, &data_mutex, &base] {
    auto on_data1 = [&result1, &data_mutex](const DataFixture& value) {
      std::lock_guard<std::mutex> lock(data_mutex);
      result1.push_back(value);
    };
    sub_id1 = base.GetManager()->Subscribe(on_data1);
    for (int i = 501; i < 5000; i++) {
      base.GetManager()->Push(DataFixture(i));
    }
  });
  std::thread t2([&result2, &sub_id2, &data_mutex, &base] {
    auto on_data2 = [&result2, &data_mutex](const DataFixture& value) {
      std::lock_guard<std::mutex> lock(data_mutex);
      result2.push_back(value);
    };
    sub_id2 = base.GetManager()->Subscribe(on_data2);
    base.GetManager()->Unsubscribe(sub_id2);
  });
  std::thread t3([&result3, &sub_id3, &data_mutex, &base] {
    auto on_data3 = [&result3, &data_mutex](const DataFixture& value) {
      std::lock_guard<std::mutex> lock(data_mutex);
      result3.push_back(value);
    };
    sub_id3 = base.GetManager()->Subscribe(on_data3);
  });

  if (t2.joinable()) {
    t2.join();
  };
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  base.GetManager()->Unsubscribe(sub_id3);
  t3.join();
  std::this_thread::sleep_for(std::chrono::seconds(2));
  t1.join();
  base.GetManager()->Unsubscribe(sub_id1);
  std::set<std::string> id_set = {sub_id1, sub_id2, sub_id3};
  EXPECT_FALSE(sub_id1.empty());  // Testing that no id is empty
  EXPECT_FALSE(sub_id2.empty());
  EXPECT_FALSE(sub_id3.empty());
  EXPECT_EQ(id_set.size(), 3u);
  EXPECT_EQ(result1.size(), 4499u);
  EXPECT_LE(result3.size(), 4499u);
  EXPECT_EQ(result2.size(), 0u);
}

TEST(DataSubscriberManagerTest, MultithreadedSubscribing2) {
  TestFixture base;
  std::mutex data_mutex;
  std::vector<DataFixture> result1, result2, result3;
  std::string sub_id1, sub_id2, sub_id3;
  std::thread t1([&result1, &sub_id1, &data_mutex, &base] {
    auto on_data1 = [&result1, &data_mutex](const DataFixture& value) {
      std::lock_guard<std::mutex> lock(data_mutex);
      result1.push_back(value);
    };
    sub_id1 = base.GetManager()->Subscribe(on_data1);
  });
  std::thread t2([&result2, &sub_id2, &data_mutex, &base] {
    auto on_data2 = [&result2, &data_mutex](const DataFixture& value) {
      std::lock_guard<std::mutex> lock(data_mutex);
      result2.push_back(value);
    };
    sub_id2 = base.GetManager()->Subscribe(on_data2);
    base.GetManager()->Unsubscribe(sub_id2);
  });
  std::thread t3([&result3, &sub_id3, &data_mutex, &base] {
    auto on_data3 = [&result3, &data_mutex](const DataFixture& value) {
      std::lock_guard<std::mutex> lock(data_mutex);
      result3.push_back(value);
    };
    sub_id3 = base.GetManager()->Subscribe(on_data3);
  });
  base.AddDataToBuffer(1, 500);
  base.GetManager()->Unsubscribe(sub_id1);
  t1.join();
  std::thread t4([&base] {
    for (int i = 501; i < 5000; i++) {
      base.GetManager()->Push(DataFixture(i));
    }
  });
  if (t4.joinable()) {
    t4.join();
  }
  std::this_thread::sleep_for(std::chrono::seconds(5));
  base.GetManager()->Unsubscribe(sub_id2);
  base.GetManager()->Unsubscribe(sub_id3);
  t2.join();
  t3.join();
  std::set<std::string> id_set = {sub_id1, sub_id2, sub_id3};
  EXPECT_FALSE(sub_id1.empty());  // Testing that no id is empty
  EXPECT_FALSE(sub_id2.empty());
  EXPECT_FALSE(sub_id3.empty());
  EXPECT_EQ(id_set.size(), 3u);
  EXPECT_LE(result1.size(), 4499u);
  EXPECT_EQ(result2.size(), 0u);
  EXPECT_LE(result3.size(), 4499u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  RTR_INFO("Running DataSubscriberManager tests...");
  return RUN_ALL_TESTS();
}
