#include "linkerhand/can_dispatcher.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>

#include "linkerhand/exceptions.hpp"

#ifdef __linux__
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#endif

namespace linkerhand {

namespace {

thread_local std::size_t tls_current_subscription_id = 0;

class SubscriptionCallbackScope final {
 public:
  explicit SubscriptionCallbackScope(std::size_t subscription_id)
      : previous_(tls_current_subscription_id) {
    tls_current_subscription_id = subscription_id;
  }

  ~SubscriptionCallbackScope() { tls_current_subscription_id = previous_; }

  SubscriptionCallbackScope(const SubscriptionCallbackScope&) = delete;
  SubscriptionCallbackScope& operator=(const SubscriptionCallbackScope&) = delete;

 private:
  std::size_t previous_ = 0;
};

}  // namespace

CANMessageDispatcher::SubscriberState::SubscriberState(std::size_t id_, Callback callback_)
    : id(id_), callback(std::move(callback_)) {}

bool CANMessageDispatcher::SubscriberState::try_enter() {
  if (!active.load(std::memory_order_acquire)) {
    return false;
  }

  in_flight.fetch_add(1, std::memory_order_acq_rel);
  if (!active.load(std::memory_order_acquire)) {
    exit();
    return false;
  }

  return true;
}

void CANMessageDispatcher::SubscriberState::exit() {
  const std::size_t prev = in_flight.fetch_sub(1, std::memory_order_acq_rel);
  if (prev == 1) {
    std::lock_guard<std::mutex> lock(mutex);
    cv.notify_all();
  }
}

void CANMessageDispatcher::SubscriberState::deactivate() {
  active.store(false, std::memory_order_release);
}

void CANMessageDispatcher::SubscriberState::wait_for_idle() {
  std::unique_lock<std::mutex> lock(mutex);
  cv.wait(lock, [&] { return in_flight.load(std::memory_order_acquire) == 0; });
}

CANMessageDispatcher::CANMessageDispatcher(
    const std::string& interface_name,
    const std::string& interface_type)
    : interface_name_(interface_name), interface_type_(interface_type) {
  if (interface_type_ != "socketcan") {
    throw ValidationError("Only 'socketcan' interface_type is supported");
  }

#ifndef __linux__
  (void)interface_name_;
  throw CANError("SocketCAN backend is only supported on Linux");
#else
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    throw CANError(std::string("Failed to open CAN socket: ") + std::strerror(errno));
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = static_cast<int>(::if_nametoindex(interface_name_.c_str()));
  if (addr.can_ifindex == 0) {
    const int saved_errno = errno;
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw CANError(std::string("Unknown CAN interface: ") + interface_name_ + " (" +
                   std::strerror(saved_errno) + ")");
  }

  if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const int saved_errno = errno;
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw CANError(std::string("Failed to bind CAN socket: ") + std::strerror(saved_errno));
  }

  try {
    start_callback_pool();
  } catch (...) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw;
  }

  running_.store(true);
  try {
    recv_thread_ = std::thread(&CANMessageDispatcher::recv_loop, this);
  } catch (...) {
    stop_callback_pool();
    ::close(socket_fd_);
    socket_fd_ = -1;
    throw;
  }
#endif
}

CANMessageDispatcher::~CANMessageDispatcher() {
  try {
    stop();
  } catch (...) {
  }
}

std::size_t CANMessageDispatcher::subscribe(Callback callback) {
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  const std::size_t id = next_subscription_id_++;
  subscribers_.push_back(std::make_shared<SubscriberState>(id, std::move(callback)));
  return id;
}

void CANMessageDispatcher::unsubscribe(std::size_t subscription_id) {
  std::shared_ptr<SubscriberState> subscriber;
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    const auto it = std::find_if(
        subscribers_.begin(),
        subscribers_.end(),
        [&](const auto& entry) { return entry->id == subscription_id; });
    if (it == subscribers_.end()) {
      return;
    }
    subscriber = *it;
    subscribers_.erase(it);
  }

  subscriber->deactivate();
  {
    std::lock_guard<std::mutex> lock(subscriber->queue_mutex);
    subscriber->pending_messages.clear();
  }

  const bool on_recv_thread =
      recv_thread_.joinable() && std::this_thread::get_id() == recv_thread_.get_id();
  const bool in_own_callback = tls_current_subscription_id == subscriber->id;
  if (on_recv_thread || in_own_callback) {
    return;
  }
  subscriber->wait_for_idle();
}

void CANMessageDispatcher::send(const CanMessage& msg) {
#ifndef __linux__
  (void)msg;
  throw CANError("SocketCAN backend is only supported on Linux");
#else
  if (!running_.load(std::memory_order_acquire)) {
    throw StateError("CAN dispatcher is stopped");
  }
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (!running_.load(std::memory_order_acquire)) {
    throw StateError("CAN dispatcher is stopped");
  }
  if (socket_fd_ < 0) {
    throw CANError("CAN dispatcher is not initialized");
  }
  if (msg.dlc > 8) {
    throw ValidationError("CAN frame dlc must be <= 8");
  }

  can_frame frame{};
  frame.can_id = msg.arbitration_id;
  if (msg.is_extended_id) {
    frame.can_id |= CAN_EFF_FLAG;
  } else {
    frame.can_id &= CAN_SFF_MASK;
  }
  frame.can_dlc = static_cast<__u8>(msg.dlc);
  for (std::size_t i = 0; i < msg.dlc; ++i) {
    frame.data[i] = msg.data[i];
  }

  const ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
  if (n != static_cast<ssize_t>(sizeof(frame))) {
    throw CANError(std::string("Failed to send CAN frame: ") + std::strerror(errno));
  }
#endif
}

void CANMessageDispatcher::stop() {
#ifndef __linux__
  return;
#else
  const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
  if (was_running && recv_thread_.joinable()) {
    recv_thread_.join();
  }
  stop_callback_pool();
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
#endif
}

void CANMessageDispatcher::start_callback_pool() {
  if (callbacks_running_.load(std::memory_order_acquire)) {
    return;
  }

  std::size_t thread_count = std::thread::hardware_concurrency();
  if (thread_count == 0) {
    thread_count = 2;
  }
  thread_count = std::max<std::size_t>(2, std::min<std::size_t>(4, thread_count));

  callbacks_running_.store(true, std::memory_order_release);

  std::vector<std::thread> workers;
  workers.reserve(thread_count);
  try {
    for (std::size_t i = 0; i < thread_count; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(callbacks_mutex_);
            callbacks_cv_.wait(lock, [&] {
              return !callbacks_running_.load(std::memory_order_acquire) ||
                     !callbacks_tasks_.empty();
            });

            if (!callbacks_running_.load(std::memory_order_acquire) && callbacks_tasks_.empty()) {
              return;
            }

            task = std::move(callbacks_tasks_.front());
            callbacks_tasks_.pop_front();
          }

          try {
            task();
          } catch (const std::exception& e) {
            std::cerr << "CANMessageDispatcher worker error: " << e.what() << "\n";
          } catch (...) {
            std::cerr << "CANMessageDispatcher worker error: unknown\n";
          }
        }
      });
    }
  } catch (...) {
    callbacks_running_.store(false, std::memory_order_release);
    callbacks_cv_.notify_all();
    for (auto& worker : workers) {
      if (!worker.joinable()) {
        continue;
      }
      try {
        worker.join();
      } catch (...) {
      }
    }
    throw;
  }

  callback_threads_ = std::move(workers);
}

void CANMessageDispatcher::stop_callback_pool() noexcept {
  const bool was_running = callbacks_running_.exchange(false, std::memory_order_acq_rel);
  if (!was_running) {
    return;
  }

  callbacks_cv_.notify_all();
  for (auto& worker : callback_threads_) {
    if (!worker.joinable()) {
      continue;
    }
    try {
      worker.join();
    } catch (...) {
    }
  }
  callback_threads_.clear();

  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  callbacks_tasks_.clear();
}

bool CANMessageDispatcher::submit_callback_task(std::function<void()> task) {
  if (!callbacks_running_.load(std::memory_order_acquire)) {
    return false;
  }
  try {
    {
      std::lock_guard<std::mutex> lock(callbacks_mutex_);
      if (!callbacks_running_.load(std::memory_order_acquire)) {
        return false;
      }
      callbacks_tasks_.push_back(std::move(task));
    }
    callbacks_cv_.notify_one();
    return true;
  } catch (...) {
    return false;
  }
}

void CANMessageDispatcher::drain_subscriber_queue(std::shared_ptr<SubscriberState> subscriber) {
  while (true) {
    CanMessage msg{};
    {
      std::lock_guard<std::mutex> lock(subscriber->queue_mutex);
      if (subscriber->pending_messages.empty()) {
        subscriber->worker_scheduled = false;
        return;
      }
      msg = subscriber->pending_messages.front();
      subscriber->pending_messages.pop_front();
    }

    if (!subscriber->try_enter()) {
      continue;
    }

    try {
      SubscriptionCallbackScope scope(subscriber->id);
      subscriber->callback(msg);
    } catch (const std::exception& e) {
      std::cerr << "CANMessageDispatcher callback error: " << e.what() << "\n";
    } catch (...) {
      std::cerr << "CANMessageDispatcher callback error: unknown\n";
    }

    subscriber->exit();
  }
}

void CANMessageDispatcher::recv_loop() {
#ifndef __linux__
  return;
#else
  pollfd pfd{};
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  while (running_.load()) {
    const int ret = ::poll(&pfd, 1, 10);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::cerr << "CANMessageDispatcher poll error: " << std::strerror(errno) << "\n";
      continue;
    }
    if (ret == 0) {
      continue;
    }
    if ((pfd.revents & POLLIN) == 0) {
      continue;
    }

    can_frame frame{};
    const ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::cerr << "CANMessageDispatcher read error: " << std::strerror(errno) << "\n";
      continue;
    }
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    CanMessage msg{};
    msg.is_extended_id = (frame.can_id & CAN_EFF_FLAG) != 0;
    msg.arbitration_id =
        msg.is_extended_id ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
    msg.dlc = frame.can_dlc;
    for (std::size_t i = 0; i < msg.dlc && i < msg.data.size(); ++i) {
      msg.data[i] = frame.data[i];
    }

    std::vector<std::shared_ptr<SubscriberState>> subscribers_copy;
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      subscribers_copy = subscribers_;
    }

    for (const auto& subscriber : subscribers_copy) {
      bool should_schedule = false;
      bool should_warn = false;
      std::size_t dropped_count = 0;
      {
        std::lock_guard<std::mutex> lock(subscriber->queue_mutex);
        if (!subscriber->active.load(std::memory_order_acquire)) {
          continue;
        }

        if (subscriber->pending_messages.size() >= kSubscriberQueueMax) {
          subscriber->pending_messages.pop_front();
          dropped_count = ++subscriber->dropped_messages;
          should_warn = dropped_count == 1 || dropped_count % 100 == 0;
        }

        subscriber->pending_messages.push_back(msg);

        if (!subscriber->worker_scheduled) {
          subscriber->worker_scheduled = true;
          should_schedule = true;
        }
      }

      if (should_warn) {
        std::cerr << "CANMessageDispatcher subscriber queue overflow (subscription_id="
                  << subscriber->id << ", max=" << kSubscriberQueueMax
                  << "), dropped=" << dropped_count << "\n";
      }

      if (!should_schedule) {
        continue;
      }

      const bool submitted =
          submit_callback_task([subscriber_weak = std::weak_ptr<SubscriberState>(subscriber)] {
        if (auto subscriber_locked = subscriber_weak.lock()) {
          CANMessageDispatcher::drain_subscriber_queue(std::move(subscriber_locked));
        }
      });
      if (!submitted) {
        std::lock_guard<std::mutex> lock(subscriber->queue_mutex);
        subscriber->worker_scheduled = false;
        std::cerr << "CANMessageDispatcher failed to schedule subscriber worker (subscription_id="
                  << subscriber->id << ")\n";
      }
    }
  }
#endif
}

}  // namespace linkerhand
