#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

#include "linkerhand/can_backend.hpp"
#include "linkerhand/exceptions.hpp"

namespace linkerhand {

class MockCANBackend final : public CANBackend {
 public:
  std::vector<CanMessage> sent_messages;

  void inject(const CanMessage& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    recv_queue_.push_back(msg);
    cv_.notify_one();
  }

  void send(const CanMessage& msg) override {
    if (msg.dlc > 8) {
      throw ValidationError("CAN frame dlc must be <= 8");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    sent_messages.push_back(msg);
  }

  std::optional<CanMessage> recv(std::chrono::milliseconds timeout) override {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, timeout, [&] { return !recv_queue_.empty() || !open_; });
    if (recv_queue_.empty()) {
      return std::nullopt;
    }

    CanMessage msg = recv_queue_.front();
    recv_queue_.pop_front();
    return msg;
  }

  void close() override {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
    cv_.notify_all();
  }

  bool is_open() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return open_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<CanMessage> recv_queue_;
  bool open_ = true;
};

}  // namespace linkerhand

