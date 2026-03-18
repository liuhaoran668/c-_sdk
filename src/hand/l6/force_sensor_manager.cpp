#include "linkerhand/hand/l6/force_sensor_manager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <mutex>
#include <thread>

#include "linkerhand/exceptions.hpp"
#include "linkerhand/iterable_queue.hpp"

#include "../common.hpp"

namespace linkerhand::hand::l6 {

namespace {

constexpr std::size_t kFrameCount = 12;
constexpr std::size_t kBytesPerFrame = 6;

constexpr std::array<std::uint8_t, kFingerCount> kFingerCommands = {0xB1, 0xB2, 0xB3, 0xB4, 0xB5};

class FrameBatch {
 public:
  void add_frame(std::size_t frame_idx, const std::uint8_t* payload) {
    if (!received_[frame_idx]) {
      received_[frame_idx] = true;
      ++count_;
    }
    std::copy_n(payload, kBytesPerFrame, values_.begin() + frame_idx * kBytesPerFrame);
  }

  bool is_complete() const { return count_ == kFrameCount; }

  ForceSensorData assemble() const {
    ForceSensorData out{};
    out.values = values_;
    out.timestamp = detail::now_unix_seconds();
    return out;
  }

  void reset() {
    received_.fill(false);
    count_ = 0;
  }

 private:
  std::array<std::uint8_t, kFrameCount * kBytesPerFrame> values_{};
  std::array<bool, kFrameCount> received_{};
  std::size_t count_ = 0;
};

struct ForceWaiter {
  std::mutex mutex;
  std::condition_variable cv;
  bool ready = false;
  std::optional<ForceSensorData> data;
};

}  // namespace

// ---------------------------------------------------------------------------
// SingleForceSensorManager::Impl
// ---------------------------------------------------------------------------

struct SingleForceSensorManager::Impl {
  Impl(
      std::uint32_t arbitration_id_,
      CANMessageDispatcher& dispatcher_,
      std::uint8_t command_prefix_,
      std::shared_ptr<linkerhand::Lifecycle> lifecycle_)
      : arbitration_id(arbitration_id_),
        dispatcher(dispatcher_),
        command_prefix(command_prefix_),
        lifecycle(std::move(lifecycle_)) {
    if (!lifecycle) {
      throw ValidationError("lifecycle must not be null");
    }
    subscription_id = dispatcher.subscribe([this](const CanMessage& msg) { on_message(msg); });
    lifecycle_subscription_id = lifecycle->subscribe([this] { notify_waiters(); });
  }

  ~Impl() {
    try {
      stop_streaming();
    } catch (...) {
    }
    try {
      lifecycle->unsubscribe(lifecycle_subscription_id);
    } catch (...) {
    }
    dispatcher.unsubscribe(subscription_id);
  }

  ForceSensorData get_data_blocking(std::chrono::milliseconds timeout) {
    if (timeout <= std::chrono::milliseconds::zero()) {
      throw ValidationError("timeout must be positive");
    }

    auto waiter = std::make_shared<ForceWaiter>();
    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters.push_back(waiter);
    }

    try {
      send_request();
    } catch (...) {
      remove_waiter(waiter);
      throw;
    }

    std::unique_lock<std::mutex> waiter_lock(waiter->mutex);
    const bool ok = waiter->cv.wait_for(
        waiter_lock, timeout,
        [&] { return waiter->ready || lifecycle->state() != linkerhand::LifecycleState::Open; });
    if (ok && waiter->data.has_value()) {
      return *waiter->data;
    }

    remove_waiter(waiter);
    lifecycle->ensure_open();
    throw TimeoutError("No force sensor data received within timeout");
  }

  std::optional<ForceSensorData> get_latest_data() const {
    std::lock_guard<std::mutex> lock(latest_mutex);
    return latest_data;
  }

  IterableQueue<ForceSensorData> stream(std::chrono::milliseconds interval, std::size_t maxsize) {
    if (interval <= std::chrono::milliseconds::zero()) {
      throw ValidationError("interval must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }
    if (!dispatcher.is_running()) {
      throw StateError("CAN dispatcher is stopped");
    }

    IterableQueue<ForceSensorData> queue(maxsize);
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_queue.has_value() || streaming_thread.joinable()) {
        throw StateError("Streaming is already active. Call stop_streaming() first.");
      }
      streaming_queue = queue;
      streaming_running.store(true, std::memory_order_release);
      try {
        streaming_thread = std::thread([this, interval, queue] {
          try {
            streaming_loop(interval);
          } catch (...) {
          }
          queue.close();
          streaming_running.store(false, std::memory_order_release);
        });
      } catch (...) {
        streaming_running.store(false, std::memory_order_release);
        streaming_queue.reset();
        throw;
      }
    }

    return queue;
  }

  void stop_streaming() {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    if (!streaming_queue.has_value()) {
      if (streaming_thread.joinable()) {
        streaming_running.store(false, std::memory_order_release);
        streaming_thread.join();
      }
      return;
    }

    streaming_running.store(false, std::memory_order_release);
    streaming_queue->close();
    streaming_queue.reset();
    if (streaming_thread.joinable()) {
      streaming_thread.join();
    }
  }

  void send_request() {
    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.dlc = 2;
    msg.data[0] = command_prefix;
    msg.data[1] = 0xC6;
    dispatcher.send(msg);
  }

  void streaming_loop(std::chrono::milliseconds interval) {
    while (streaming_running.load(std::memory_order_acquire)) {
      send_request();
      std::this_thread::sleep_for(interval);
    }
  }

  void remove_waiter(const std::shared_ptr<ForceWaiter>& waiter) {
    std::lock_guard<std::mutex> lock(waiters_mutex);
    waiters.erase(
        std::remove_if(
            waiters.begin(), waiters.end(),
            [&](const auto& w) { return w.get() == waiter.get(); }),
        waiters.end());
  }

  void notify_waiters() {
    std::vector<std::shared_ptr<ForceWaiter>> waiters_copy;
    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters_copy = waiters;
    }
    for (const auto& waiter : waiters_copy) {
      waiter->cv.notify_one();
    }
  }

  void on_message(const CanMessage& msg) {
    if (msg.arbitration_id != arbitration_id) {
      return;
    }
    if (msg.dlc < 8 || msg.data[0] != command_prefix) {
      return;
    }

    const std::size_t frame_idx = static_cast<std::size_t>(msg.data[1] >> 4);
    if (frame_idx >= kFrameCount) {
      return;
    }

    frame_batch_.add_frame(frame_idx, &msg.data[2]);

    if (!frame_batch_.is_complete()) {
      return;
    }

    const ForceSensorData complete_data = frame_batch_.assemble();
    frame_batch_.reset();

    on_complete_data(complete_data);
  }

  void on_complete_data(const ForceSensorData& data) {
    {
      std::lock_guard<std::mutex> lock(latest_mutex);
      latest_data = data;
    }

    std::vector<std::shared_ptr<ForceWaiter>> waiters_copy;
    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters_copy = std::move(waiters);
    }
    for (const auto& waiter : waiters_copy) {
      std::lock_guard<std::mutex> lock(waiter->mutex);
      waiter->data = data;
      waiter->ready = true;
      waiter->cv.notify_one();
    }

    std::optional<IterableQueue<ForceSensorData>> q;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      q = streaming_queue;
    }
    if (q.has_value()) {
      q->force_put(data);
    }
  }

  std::uint32_t arbitration_id;
  CANMessageDispatcher& dispatcher;
  std::uint8_t command_prefix;
  std::size_t subscription_id = 0;
  std::shared_ptr<linkerhand::Lifecycle> lifecycle;
  std::size_t lifecycle_subscription_id = 0;

  FrameBatch frame_batch_;

  mutable std::mutex latest_mutex;
  std::optional<ForceSensorData> latest_data;

  mutable std::mutex waiters_mutex;
  std::vector<std::shared_ptr<ForceWaiter>> waiters;

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<ForceSensorData>> streaming_queue;
  std::atomic<bool> streaming_running{false};
  std::thread streaming_thread;
};

SingleForceSensorManager::SingleForceSensorManager(
    std::uint32_t arbitration_id,
    CANMessageDispatcher& dispatcher,
    std::uint8_t command_prefix,
    std::shared_ptr<linkerhand::Lifecycle> lifecycle)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher, command_prefix, lifecycle)),
      lifecycle_(std::move(lifecycle)) {}

SingleForceSensorManager::~SingleForceSensorManager() = default;

ForceSensorData SingleForceSensorManager::get_data_blocking(std::chrono::milliseconds timeout) {
  lifecycle_->ensure_open();
  return impl_->get_data_blocking(timeout);
}

std::optional<ForceSensorData> SingleForceSensorManager::get_latest_data() const {
  lifecycle_->ensure_open();
  return impl_->get_latest_data();
}

IterableQueue<ForceSensorData> SingleForceSensorManager::stream(
    std::chrono::milliseconds interval, std::size_t maxsize) {
  lifecycle_->ensure_open();
  return impl_->stream(interval, maxsize);
}

void SingleForceSensorManager::stop_streaming() {
  impl_->stop_streaming();
}

// ---------------------------------------------------------------------------
// ForceSensorManager::Impl
// ---------------------------------------------------------------------------

struct ForceSensorManager::Impl {
  explicit Impl(
      std::uint32_t arbitration_id_,
      CANMessageDispatcher& dispatcher_,
      std::shared_ptr<linkerhand::Lifecycle> lifecycle_)
      : arbitration_id(arbitration_id_),
        dispatcher(dispatcher_),
        lifecycle(std::move(lifecycle_)) {
    if (!lifecycle) {
      throw ValidationError("lifecycle must not be null");
    }
    for (std::size_t i = 0; i < kFingerCount; ++i) {
      fingers[i] = std::make_unique<SingleForceSensorManager>(
          arbitration_id, dispatcher, kFingerCommands[i], lifecycle);
    }
  }

  ~Impl() {
    try {
      stop_streaming();
    } catch (...) {
    }
  }

  AllFingersData get_data_blocking(std::chrono::milliseconds timeout) {
    if (timeout <= std::chrono::milliseconds::zero()) {
      throw ValidationError("timeout must be positive");
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;

    FingerArray<std::future<ForceSensorData>> futures;
    for (std::size_t i = 0; i < kFingerCount; ++i) {
      auto* finger = fingers[i].get();
      futures[i] = std::async(std::launch::async,
          [finger, timeout] { return finger->get_data_blocking(timeout); });
    }

    AllFingersData out{};
    for (std::size_t i = 0; i < kFingerCount; ++i) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
      if (remaining <= std::chrono::milliseconds::zero()) {
        throw TimeoutError("Force sensor data collection timed out");
      }

      auto status = futures[i].wait_for(remaining);
      if (status != std::future_status::ready) {
        throw TimeoutError("Force sensor data collection timed out");
      }
      out.fingers[i] = futures[i].get();
    }
    return out;
  }

  IterableQueue<AllFingersData> stream(std::chrono::milliseconds interval, std::size_t maxsize) {
    if (interval <= std::chrono::milliseconds::zero()) {
      throw ValidationError("interval must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }

    IterableQueue<AllFingersData> queue(maxsize);
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_queue.has_value() || aggregation_thread.joinable()) {
        throw StateError("Streaming is already active. Call stop_streaming() first.");
      }
      streaming_queue = queue;

      FingerArray<IterableQueue<ForceSensorData>> finger_queues;
      try {
        for (std::size_t i = 0; i < kFingerCount; ++i) {
          finger_queues[i] = fingers[i]->stream(interval, maxsize);
        }

        aggregation_running.store(true, std::memory_order_release);
        aggregation_thread = std::thread([this, finger_queues, queue] {
          try {
            aggregation_loop(finger_queues, queue);
          } catch (...) {
          }
        });
      } catch (...) {
        for (auto& f : fingers) {
          try {
            f->stop_streaming();
          } catch (...) {
          }
        }
        aggregation_running.store(false, std::memory_order_release);
        streaming_queue.reset();
        throw;
      }
    }

    return queue;
  }

  void stop_streaming() {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    if (!streaming_queue.has_value()) {
      if (aggregation_thread.joinable()) {
        aggregation_running.store(false, std::memory_order_release);
        aggregation_thread.join();
      }
      for (auto& t : reader_threads) {
        if (t.joinable()) {
          t.join();
        }
      }
      reader_threads.clear();
      return;
    }

    for (auto& f : fingers) {
      f->stop_streaming();
    }

    aggregation_running.store(false, std::memory_order_release);
    streaming_queue->close();
    streaming_queue.reset();

    if (aggregation_thread.joinable()) {
      aggregation_thread.join();
    }
    for (auto& t : reader_threads) {
      if (t.joinable()) {
        t.join();
      }
    }
    reader_threads.clear();
  }

  void aggregation_loop(
      const FingerArray<IterableQueue<ForceSensorData>>& finger_queues,
      IterableQueue<AllFingersData> output_queue) {
    using TaggedData = std::pair<std::size_t, ForceSensorData>;
    auto intermediate_queue = std::make_shared<IterableQueue<TaggedData>>();

    reader_threads.clear();
    for (std::size_t i = 0; i < kFingerCount; ++i) {
      auto fq = finger_queues[i];
      reader_threads.emplace_back([i, fq, intermediate_queue] {
        try {
          for (const auto& data : fq) {
            intermediate_queue->put({i, data});
          }
        } catch (...) {
        }
      });
    }

    FingerArray<std::optional<ForceSensorData>> latest{};
    std::size_t filled = 0;

    while (aggregation_running.load(std::memory_order_acquire)) {
      auto item = intermediate_queue->get_for(std::chrono::seconds{1});
      if (!item.has_value()) {
        continue;
      }

      auto& [finger_idx, sensor_data] = *item;
      if (!latest[finger_idx].has_value()) {
        ++filled;
      }
      latest[finger_idx] = std::move(sensor_data);

      if (filled < kFingerCount) {
        continue;
      }

      AllFingersData snapshot{};
      for (std::size_t i = 0; i < kFingerCount; ++i) {
        snapshot.fingers[i] = *latest[i];
      }

      if (!output_queue.force_put(std::move(snapshot))) {
        break;
      }

      latest.fill(std::nullopt);
      filled = 0;
    }

    intermediate_queue->close();
    output_queue.close();
  }

  FingerArray<std::optional<ForceSensorData>> get_latest_data() const {
    FingerArray<std::optional<ForceSensorData>> out{};
    for (std::size_t i = 0; i < kFingerCount; ++i) {
      out[i] = fingers[i]->get_latest_data();
    }
    return out;
  }

  std::uint32_t arbitration_id;
  CANMessageDispatcher& dispatcher;
  std::shared_ptr<linkerhand::Lifecycle> lifecycle;

  FingerArray<std::unique_ptr<SingleForceSensorManager>> fingers{};

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<AllFingersData>> streaming_queue;
  std::atomic<bool> aggregation_running{false};
  std::thread aggregation_thread;
  std::vector<std::thread> reader_threads;
};

ForceSensorManager::ForceSensorManager(
    std::uint32_t arbitration_id,
    CANMessageDispatcher& dispatcher,
    std::shared_ptr<linkerhand::Lifecycle> lifecycle)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher, lifecycle)),
      lifecycle_(std::move(lifecycle)) {}

ForceSensorManager::~ForceSensorManager() = default;

AllFingersData ForceSensorManager::get_data_blocking(std::chrono::milliseconds timeout) {
  lifecycle_->ensure_open();
  return impl_->get_data_blocking(timeout);
}

IterableQueue<AllFingersData> ForceSensorManager::stream(
    std::chrono::milliseconds interval, std::size_t maxsize) {
  lifecycle_->ensure_open();
  return impl_->stream(interval, maxsize);
}

void ForceSensorManager::stop_streaming() {
  impl_->stop_streaming();
}

FingerArray<std::optional<ForceSensorData>> ForceSensorManager::get_latest_data() const {
  lifecycle_->ensure_open();
  return impl_->get_latest_data();
}

}  // namespace linkerhand::hand::l6
