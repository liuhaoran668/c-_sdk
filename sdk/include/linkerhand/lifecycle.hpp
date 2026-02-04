#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "linkerhand/exceptions.hpp"

namespace linkerhand {

enum class LifecycleState {
  Open,
  Closed,
};

class Lifecycle {
 public:
  using Callback = std::function<void()>;

  explicit Lifecycle(std::string owner_name = "L6") : owner_name_(std::move(owner_name)) {}

  Lifecycle(const Lifecycle&) = delete;
  Lifecycle& operator=(const Lifecycle&) = delete;

  LifecycleState state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  bool is_open() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == LifecycleState::Open;
  }

  bool is_closed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ == LifecycleState::Closed;
  }

  void ensure_open() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::Open) {
      throw StateError(owner_name_ + " interface is closed. Create a new instance or use context manager.");
    }
  }

  /// Single atomic close operation: sets state to Closed and notifies all subscribers.
  /// Idempotent: returns false if already closed, true if this call performed the close.
  bool close() {
    std::vector<std::shared_ptr<Subscription>> subscriptions;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == LifecycleState::Closed) {
        return false;  // Already closed
      }
      state_ = LifecycleState::Closed;
      notification_count_ += 1;
      subscriptions.reserve(subscribers_.size());
      for (const auto& [id, sub] : subscribers_) {
        (void)id;
        subscriptions.push_back(sub);
      }
    }

    cv_.notify_all();
    for (const auto& sub : subscriptions) {
      if (!sub) {
        continue;
      }
      if (!sub->active.load()) {
        continue;
      }
      try {
        sub->cb();
      } catch (...) {
      }
    }
    return true;
  }

  std::uint64_t notification_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return notification_count_;
  }

  bool wait_for_notification(std::uint64_t last_notification_count, double timeout_ms) const {
    if (timeout_ms < 0) {
      throw ValidationError("timeout_ms must be non-negative");
    }

    std::unique_lock<std::mutex> lock(mutex_);
    auto pred = [&] { return notification_count_ != last_notification_count || state_ != LifecycleState::Open; };
    if (timeout_ms == 0) {
      return pred();
    }

    const auto timeout =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(timeout_ms));
    return cv_.wait_for(lock, timeout, pred);
  }

  std::size_t subscribe(Callback callback) {
    if (!callback) {
      throw ValidationError("callback must be callable");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t id = next_subscription_id_++;
    auto sub = std::make_shared<Subscription>();
    sub->cb = std::move(callback);
    subscribers_.emplace(id, std::move(sub));
    return id;
  }

  void unsubscribe(std::size_t subscription_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = subscribers_.find(subscription_id);
    if (it == subscribers_.end()) {
      return;
    }
    it->second->active.store(false);
    subscribers_.erase(it);
  }

 private:
  struct Subscription {
    std::atomic<bool> active{true};
    Callback cb;
  };

  const std::string owner_name_;

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  LifecycleState state_ = LifecycleState::Open;

  std::uint64_t notification_count_ = 0;

  std::size_t next_subscription_id_ = 1;
  std::unordered_map<std::size_t, std::shared_ptr<Subscription>> subscribers_;
};

}  // namespace linkerhand
