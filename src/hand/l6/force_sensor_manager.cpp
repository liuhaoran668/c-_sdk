#include "linkerhand/hand/l6/force_sensor_manager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <thread>

#include "linkerhand/exceptions.hpp"
#include "linkerhand/iterable_queue.hpp"

#include "../common.hpp"

namespace linkerhand::hand::l6 {
namespace {

constexpr std::size_t kFrameCount = 12;
constexpr std::size_t kBytesPerFrame = 6;
constexpr std::size_t kTotalBytes = kFrameCount * kBytesPerFrame;

struct FrameBatch {
  std::array<std::optional<std::array<std::uint8_t, kBytesPerFrame>>, kFrameCount> frames{};
  std::size_t count = 0;
  double started_at = detail::now_unix_seconds();

  FrameBatch add_frame(std::size_t frame_id, const std::array<std::uint8_t, kBytesPerFrame>& data) const {
    FrameBatch next = *this;
    if (!next.frames[frame_id].has_value()) {
      next.count += 1;
    }
    next.frames[frame_id] = data;
    return next;
  }

  bool is_complete() const { return count == kFrameCount; }

  ForceSensorData assemble() const {
    ForceSensorData out{};
    std::size_t offset = 0;
    for (std::size_t i = 0; i < kFrameCount; ++i) {
      const auto& frame = frames[i];
      if (!frame.has_value()) {
        throw StateError("Incomplete frame batch");
      }
      for (std::size_t j = 0; j < kBytesPerFrame; ++j) {
        out.values[offset++] = (*frame)[j];
      }
    }
    out.timestamp = detail::now_unix_seconds();
    return out;
  }
};

struct ForceWaiter {
  std::mutex mutex;
  std::condition_variable cv;
  bool ready = false;
  std::optional<ForceSensorData> data;
};

}  // namespace

struct SingleForceSensorManager::Impl {
  Impl(std::uint32_t arbitration_id_, CANMessageDispatcher& dispatcher_, std::uint8_t command_prefix_)
      : arbitration_id(arbitration_id_),
        dispatcher(dispatcher_),
        command_prefix(command_prefix_) {
    subscription_id = dispatcher.subscribe([this](const CanMessage& msg) { on_message(msg); });
  }

  ~Impl() {
    try {
      stop_streaming();
    } catch (...) {
    }
    dispatcher.unsubscribe(subscription_id);
  }

  ForceSensorData get_data_blocking(double timeout_ms) {
    if (timeout_ms <= 0) {
      throw ValidationError("timeout_ms must be positive");
    }

    auto waiter = std::make_shared<ForceWaiter>();
    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters.push_back(waiter);
    }

    send_request();

    const auto timeout =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(timeout_ms));

    std::unique_lock<std::mutex> waiter_lock(waiter->mutex);
    const bool ok = waiter->cv.wait_for(waiter_lock, timeout, [&] { return waiter->ready; });
    if (ok && waiter->data.has_value()) {
      return *waiter->data;
    }

    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters.erase(
          std::remove_if(
              waiters.begin(),
              waiters.end(),
              [&](const auto& w) { return w.get() == waiter.get(); }),
          waiters.end());
    }

    throw TimeoutError("No data received within " + std::to_string(timeout_ms) + "ms");
  }

  std::optional<ForceSensorData> get_latest_data() const {
    std::lock_guard<std::mutex> lock(latest_mutex);
    return latest_data;
  }

  IterableQueue<ForceSensorData> stream(double interval_ms, std::size_t maxsize) {
    if (interval_ms <= 0) {
      throw ValidationError("interval_ms must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }

    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_queue.has_value()) {
        throw StateError("Streaming is already active. Call stop_streaming() first.");
      }
      streaming_queue = IterableQueue<ForceSensorData>(maxsize);
      streaming_interval_ms = interval_ms;
      streaming_running.store(true);
    }

    streaming_thread = std::thread([this] {
      try {
        streaming_loop();
      } catch (...) {
      }
    });

    std::lock_guard<std::mutex> lock(streaming_mutex);
    return *streaming_queue;
  }

  void stop_streaming() {
    std::optional<IterableQueue<ForceSensorData>> queue_to_close;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (!streaming_queue.has_value()) {
        return;
      }
      streaming_running.store(false);
      queue_to_close = streaming_queue;
      streaming_queue.reset();
      streaming_interval_ms.reset();
    }

    queue_to_close->close();
    if (streaming_thread.joinable()) {
      streaming_thread.join();
    }
  }

  void send_request() {
    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.is_extended_id = false;
    msg.dlc = 2;
    msg.data[0] = command_prefix;
    msg.data[1] = 0xC6;
    dispatcher.send(msg);
  }

  void streaming_loop() {
    double interval_ms = 0.0;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (!streaming_interval_ms.has_value()) {
        throw StateError("Streaming is not active. Call stream() first.");
      }
      interval_ms = *streaming_interval_ms;
    }

    const auto sleep_duration =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(interval_ms));

    while (streaming_running.load()) {
      send_request();
      std::this_thread::sleep_for(sleep_duration);
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

    std::array<std::uint8_t, kBytesPerFrame> frame_data{};
    for (std::size_t i = 0; i < kBytesPerFrame; ++i) {
      frame_data[i] = msg.data[2 + i];
    }

    if (!frame_batch.has_value()) {
      frame_batch = FrameBatch{};
    }
    frame_batch = frame_batch->add_frame(frame_idx, frame_data);

    if (!frame_batch->is_complete()) {
      return;
    }

    const ForceSensorData complete_data = frame_batch->assemble();
    frame_batch.reset();

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
      waiters_copy = waiters;
      waiters.clear();
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
    if (!q.has_value()) {
      return;
    }

    try {
      q->put_nowait(data);
    } catch (const QueueFull&) {
      try {
        q->get_nowait();
        q->put_nowait(data);
      } catch (const QueueEmpty&) {
      }
    }
  }

  std::uint32_t arbitration_id;
  CANMessageDispatcher& dispatcher;
  std::uint8_t command_prefix;
  std::size_t subscription_id = 0;

  std::optional<FrameBatch> frame_batch;

  mutable std::mutex latest_mutex;
  std::optional<ForceSensorData> latest_data;

  mutable std::mutex waiters_mutex;
  std::vector<std::shared_ptr<ForceWaiter>> waiters;

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<ForceSensorData>> streaming_queue;
  std::optional<double> streaming_interval_ms;
  std::atomic<bool> streaming_running{false};
  std::thread streaming_thread;
};

SingleForceSensorManager::SingleForceSensorManager(
    std::uint32_t arbitration_id,
    CANMessageDispatcher& dispatcher,
    std::uint8_t command_prefix)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher, command_prefix)) {}

SingleForceSensorManager::~SingleForceSensorManager() = default;

ForceSensorData SingleForceSensorManager::get_data_blocking(double timeout_ms) {
  return impl_->get_data_blocking(timeout_ms);
}

std::optional<ForceSensorData> SingleForceSensorManager::get_latest_data() const {
  return impl_->get_latest_data();
}

IterableQueue<ForceSensorData> SingleForceSensorManager::stream(double interval_ms, std::size_t maxsize) {
  return impl_->stream(interval_ms, maxsize);
}

void SingleForceSensorManager::stop_streaming() { impl_->stop_streaming(); }

struct ForceSensorManager::Impl {
  explicit Impl(std::uint32_t arbitration_id_, CANMessageDispatcher& dispatcher_)
      : arbitration_id(arbitration_id_), dispatcher(dispatcher_) {
    fingers.emplace("thumb", std::make_unique<SingleForceSensorManager>(arbitration_id, dispatcher, 0xB1));
    fingers.emplace("index", std::make_unique<SingleForceSensorManager>(arbitration_id, dispatcher, 0xB2));
    fingers.emplace("middle", std::make_unique<SingleForceSensorManager>(arbitration_id, dispatcher, 0xB3));
    fingers.emplace("ring", std::make_unique<SingleForceSensorManager>(arbitration_id, dispatcher, 0xB4));
    fingers.emplace("pinky", std::make_unique<SingleForceSensorManager>(arbitration_id, dispatcher, 0xB5));
  }

  ~Impl() {
    try {
      stop_streaming();
    } catch (...) {
    }
  }

  AllFingersData get_data_blocking(double timeout_ms) {
    if (timeout_ms <= 0) {
      throw ValidationError("timeout_ms must be positive");
    }

    AllFingersData out{};
    out.thumb = fingers.at("thumb")->get_data_blocking(timeout_ms);
    out.index = fingers.at("index")->get_data_blocking(timeout_ms);
    out.middle = fingers.at("middle")->get_data_blocking(timeout_ms);
    out.ring = fingers.at("ring")->get_data_blocking(timeout_ms);
    out.pinky = fingers.at("pinky")->get_data_blocking(timeout_ms);
    return out;
  }

  IterableQueue<AllFingersData> stream(double interval_ms, std::size_t maxsize) {
    if (interval_ms <= 0) {
      throw ValidationError("interval_ms must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }

    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_queue.has_value()) {
        throw StateError("Streaming is already active. Call stop_streaming() first.");
      }
      streaming_queue = IterableQueue<AllFingersData>(maxsize);

      finger_queues.emplace("thumb", fingers.at("thumb")->stream(interval_ms, maxsize));
      finger_queues.emplace("index", fingers.at("index")->stream(interval_ms, maxsize));
      finger_queues.emplace("middle", fingers.at("middle")->stream(interval_ms, maxsize));
      finger_queues.emplace("ring", fingers.at("ring")->stream(interval_ms, maxsize));
      finger_queues.emplace("pinky", fingers.at("pinky")->stream(interval_ms, maxsize));

      aggregation_running.store(true);
    }

    aggregation_thread = std::thread([this] {
      try {
        aggregation_loop();
      } catch (...) {
      }
    });

    std::lock_guard<std::mutex> lock(streaming_mutex);
    return *streaming_queue;
  }

  void stop_streaming() {
    std::optional<IterableQueue<AllFingersData>> queue_to_close;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (!streaming_queue.has_value()) {
        return;
      }

      for (auto& [name, sensor] : fingers) {
        (void)name;
        sensor->stop_streaming();
      }

      aggregation_running.store(false);
      queue_to_close = streaming_queue;
      streaming_queue.reset();
      finger_queues.clear();
    }

    queue_to_close->close();

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

  void aggregation_loop() {
    std::unordered_map<std::string, IterableQueue<ForceSensorData>> finger_queues_copy;
    std::optional<IterableQueue<AllFingersData>> output_queue;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (finger_queues.empty()) {
        throw StateError("Streaming is not active. Call stream() first.");
      }
      finger_queues_copy = finger_queues;
      output_queue = streaming_queue;
    }

    IterableQueue<std::pair<std::string, ForceSensorData>> intermediate_queue;

    reader_threads.clear();
    for (const auto& entry : finger_queues_copy) {
      const std::string finger_name = entry.first;
      const auto finger_queue = entry.second;

      reader_threads.emplace_back([finger_name, finger_queue, &intermediate_queue] {
        try {
          for (const auto& data : finger_queue) {
            intermediate_queue.put({finger_name, data});
          }
        } catch (...) {
        }
      });
    }

    std::unordered_map<std::string, ForceSensorData> latest{};

    try {
      while (aggregation_running.load()) {
        auto item = intermediate_queue.get(/*block=*/true, /*timeout_s=*/1.0);
        latest[item.first] = item.second;

        if (latest.size() != 5) {
          continue;
        }

        AllFingersData snapshot{
            latest.at("thumb"),
            latest.at("index"),
            latest.at("middle"),
            latest.at("ring"),
            latest.at("pinky"),
        };

        if (output_queue.has_value()) {
          try {
            output_queue->put_nowait(snapshot);
          } catch (const QueueFull&) {
            try {
              output_queue->get_nowait();
              output_queue->put_nowait(snapshot);
            } catch (const QueueEmpty&) {
            }
          }
        }

        latest.clear();
      }
    } catch (const QueueEmpty&) {
      // Match Python behavior: exit on timeout.
    }
  }

  std::unordered_map<std::string, std::optional<ForceSensorData>> get_latest_data() const {
    std::unordered_map<std::string, std::optional<ForceSensorData>> out;
    for (const auto& [name, sensor] : fingers) {
      out[name] = sensor->get_latest_data();
    }
    return out;
  }

  std::uint32_t arbitration_id;
  CANMessageDispatcher& dispatcher;

  std::unordered_map<std::string, std::unique_ptr<SingleForceSensorManager>> fingers;

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<AllFingersData>> streaming_queue;
  std::unordered_map<std::string, IterableQueue<ForceSensorData>> finger_queues;
  std::atomic<bool> aggregation_running{false};
  std::thread aggregation_thread;
  std::vector<std::thread> reader_threads;
};

ForceSensorManager::ForceSensorManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher)) {}

ForceSensorManager::~ForceSensorManager() = default;

AllFingersData ForceSensorManager::get_data_blocking(double timeout_ms) {
  return impl_->get_data_blocking(timeout_ms);
}

IterableQueue<AllFingersData> ForceSensorManager::stream(double interval_ms, std::size_t maxsize) {
  return impl_->stream(interval_ms, maxsize);
}

void ForceSensorManager::stop_streaming() { impl_->stop_streaming(); }

std::unordered_map<std::string, std::optional<ForceSensorData>> ForceSensorManager::get_latest_data() const {
  return impl_->get_latest_data();
}

}  // namespace linkerhand::hand::l6
