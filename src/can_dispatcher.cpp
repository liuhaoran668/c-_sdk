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

  running_.store(true);
  recv_thread_ = std::thread(&CANMessageDispatcher::recv_loop, this);
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
  subscribers_.push_back({id, std::move(callback)});
  return id;
}

void CANMessageDispatcher::unsubscribe(std::size_t subscription_id) {
  std::lock_guard<std::mutex> lock(subscribers_mutex_);
  subscribers_.erase(
      std::remove_if(
          subscribers_.begin(),
          subscribers_.end(),
          [&](const auto& entry) { return entry.first == subscription_id; }),
      subscribers_.end());
}

void CANMessageDispatcher::send(const CanMessage& msg) {
#ifndef __linux__
  (void)msg;
  throw CANError("SocketCAN backend is only supported on Linux");
#else
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
  const bool was_running = running_.exchange(false);
  if (was_running && recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
#endif
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

    std::vector<std::pair<std::size_t, Callback>> subscribers_copy;
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      subscribers_copy = subscribers_;
    }

    for (const auto& [id, cb] : subscribers_copy) {
      (void)id;
      try {
        cb(msg);
      } catch (const std::exception& e) {
        std::cerr << "CANMessageDispatcher callback error: " << e.what() << "\n";
      } catch (...) {
        std::cerr << "CANMessageDispatcher callback error: unknown\n";
      }
    }
  }
#endif
}

}  // namespace linkerhand

