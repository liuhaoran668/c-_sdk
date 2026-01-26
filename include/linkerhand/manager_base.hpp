#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/exceptions.hpp"
#include "linkerhand/iterable_queue.hpp"

namespace linkerhand {

/// CRTP 基类，提供流式数据管理的通用实现
/// @tparam Derived 子类类型（CRTP 模式）
/// @tparam DataType 数据类型（如 AngleData, TorqueData 等）
template <typename Derived, typename DataType>
class StreamableManagerBase {
 protected:
  /// 等待者结构，用于阻塞式获取数据
  struct Waiter {
    std::mutex mutex;
    std::condition_variable cv;
    bool ready = false;
    std::optional<DataType> data;
  };

 public:
  StreamableManagerBase(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
      : arbitration_id_(arbitration_id), dispatcher_(dispatcher) {
    subscription_id_ = dispatcher_.subscribe([this](const CanMessage& msg) { on_message(msg); });
  }

  ~StreamableManagerBase() {
    try {
      stop_streaming();
    } catch (...) {
    }
    dispatcher_.unsubscribe(subscription_id_);
  }

  StreamableManagerBase(const StreamableManagerBase&) = delete;
  StreamableManagerBase& operator=(const StreamableManagerBase&) = delete;

  /// 启动流式数据采集
  /// @param interval_ms 采集间隔（毫秒）
  /// @param maxsize 队列最大容量
  /// @return 可迭代队列
  IterableQueue<DataType> stream(double interval_ms = 100, std::size_t maxsize = 100) {
    if (interval_ms <= 0) {
      throw ValidationError("interval_ms must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }

    {
      std::lock_guard<std::mutex> lock(streaming_mutex_);
      if (streaming_queue_.has_value()) {
        throw StateError("Streaming is already active. Call stop_streaming() first.");
      }
      streaming_queue_ = IterableQueue<DataType>(maxsize);
      streaming_interval_ms_ = interval_ms;
      streaming_running_.store(true);
    }

    streaming_thread_ = std::thread([this] {
      try {
        streaming_loop();
      } catch (...) {
      }
    });

    std::lock_guard<std::mutex> lock(streaming_mutex_);
    return *streaming_queue_;
  }

  /// 停止流式数据采集
  void stop_streaming() {
    std::optional<IterableQueue<DataType>> queue_to_close;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex_);
      if (!streaming_queue_.has_value()) {
        return;
      }
      streaming_running_.store(false);
      queue_to_close = streaming_queue_;
      streaming_queue_.reset();
      streaming_interval_ms_.reset();
    }

    queue_to_close->close();
    if (streaming_thread_.joinable()) {
      streaming_thread_.join();
    }
  }

 protected:
  /// 阻塞式获取数据
  /// @param timeout_ms 超时时间（毫秒）
  /// @param data_label 数据标签（用于错误消息）
  /// @return 数据
  DataType get_blocking(double timeout_ms, const char* data_label) {
    if (timeout_ms <= 0) {
      throw ValidationError("timeout_ms must be positive");
    }

    auto waiter = std::make_shared<Waiter>();
    {
      std::lock_guard<std::mutex> lock(waiters_mutex_);
      waiters_.push_back(waiter);
    }

    bool is_streaming = false;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex_);
      is_streaming = streaming_queue_.has_value();
    }
    if (!is_streaming) {
      derived().do_send_sense_request();
    }

    const auto timeout =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(timeout_ms));

    std::unique_lock<std::mutex> waiter_lock(waiter->mutex);
    const bool ok = waiter->cv.wait_for(waiter_lock, timeout, [&] { return waiter->ready; });
    if (ok && waiter->data.has_value()) {
      return *waiter->data;
    }

    {
      std::lock_guard<std::mutex> lock(waiters_mutex_);
      waiters_.erase(
          std::remove_if(
              waiters_.begin(),
              waiters_.end(),
              [&](const auto& w) { return w.get() == waiter.get(); }),
          waiters_.end());
    }

    throw TimeoutError(
        std::string("No ") + data_label + " data received within " + std::to_string(timeout_ms) +
        "ms");
  }

  /// 获取当前缓存的数据
  /// @return 数据（如果有）
  std::optional<DataType> get_current() const {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    return latest_data_;
  }

  std::uint32_t arbitration_id_ = 0;
  CANMessageDispatcher& dispatcher_;

 private:
  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  void streaming_loop() {
    double interval_ms = 0.0;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex_);
      if (!streaming_interval_ms_.has_value()) {
        throw StateError("Streaming is not active. Call stream() first.");
      }
      interval_ms = *streaming_interval_ms_;
    }

    const auto sleep_duration =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(interval_ms));

    while (streaming_running_.load()) {
      derived().do_send_sense_request();
      std::this_thread::sleep_for(sleep_duration);
    }
  }

  void on_message(const CanMessage& msg) {
    if (msg.arbitration_id != arbitration_id_) {
      return;
    }

    const auto data = derived().do_parse_message(msg);
    if (!data.has_value()) {
      return;
    }

    on_complete_data(*data);
  }

  void on_complete_data(const DataType& data) {
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_data_ = data;
    }

    std::vector<std::shared_ptr<Waiter>> waiters_copy;
    {
      std::lock_guard<std::mutex> lock(waiters_mutex_);
      waiters_copy = waiters_;
      waiters_.clear();
    }
    for (const auto& waiter : waiters_copy) {
      std::lock_guard<std::mutex> lock(waiter->mutex);
      waiter->data = data;
      waiter->ready = true;
      waiter->cv.notify_one();
    }

    std::optional<IterableQueue<DataType>> q;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex_);
      q = streaming_queue_;
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

  std::size_t subscription_id_ = 0;

  mutable std::mutex latest_mutex_;
  std::optional<DataType> latest_data_;

  mutable std::mutex waiters_mutex_;
  std::vector<std::shared_ptr<Waiter>> waiters_;

  mutable std::mutex streaming_mutex_;
  std::optional<IterableQueue<DataType>> streaming_queue_;
  std::optional<double> streaming_interval_ms_;
  std::atomic<bool> streaming_running_{false};
  std::thread streaming_thread_;
};

}  // namespace linkerhand
