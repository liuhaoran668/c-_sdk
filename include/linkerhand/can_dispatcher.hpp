#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace linkerhand {

struct CanMessage {
  std::uint32_t arbitration_id = 0;
  bool is_extended_id = false;
  std::array<std::uint8_t, 8> data{};
  std::size_t dlc = 0;

  std::vector<std::uint8_t> data_bytes() const {
    return std::vector<std::uint8_t>(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(dlc));
  }
};

class CANMessageDispatcher {
 public:
  using Callback = std::function<void(const CanMessage&)>;

  explicit CANMessageDispatcher(
      const std::string& interface_name,
      const std::string& interface_type = "socketcan");
  ~CANMessageDispatcher();

  CANMessageDispatcher(const CANMessageDispatcher&) = delete;
  CANMessageDispatcher& operator=(const CANMessageDispatcher&) = delete;
  CANMessageDispatcher(CANMessageDispatcher&&) = delete;
  CANMessageDispatcher& operator=(CANMessageDispatcher&&) = delete;

  std::size_t subscribe(Callback callback);
  void unsubscribe(std::size_t subscription_id);

  void send(const CanMessage& msg);
  void stop();
  bool is_running() const noexcept { return running_.load(std::memory_order_acquire); }

 private:
  struct SubscriberState {
    std::size_t id = 0;
    Callback callback;
    std::atomic<bool> active{true};

    SubscriberState(std::size_t id_, Callback callback_)
        : id(id_), callback(std::move(callback_)) {}

    void deactivate() { active.store(false, std::memory_order_release); }
  };

  void recv_loop();

  std::string interface_name_;
  std::string interface_type_;
  int socket_fd_ = -1;

  std::atomic<bool> running_{false};
  std::thread recv_thread_;
  std::thread::id recv_thread_id_{};
  std::mutex recv_thread_join_mutex_;

  std::mutex socket_mutex_;

  std::mutex subscribers_mutex_;
  std::size_t next_subscription_id_ = 1;
  std::vector<std::shared_ptr<SubscriberState>> subscribers_;
};

}  // namespace linkerhand
