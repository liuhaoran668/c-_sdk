#include "socketcan_backend.hpp"

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

SocketCANBackend::SocketCANBackend(const std::string& interface_name) : interface_name_(interface_name) {
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
#endif
}

SocketCANBackend::~SocketCANBackend() {
  try {
    close();
  } catch (...) {
  }
}

void SocketCANBackend::send(const CanMessage& msg) {
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

std::optional<CanMessage> SocketCANBackend::recv(std::chrono::milliseconds timeout) {
#ifndef __linux__
  (void)timeout;
  throw CANError("SocketCAN backend is only supported on Linux");
#else
  if (socket_fd_ < 0) {
    throw CANError("CAN dispatcher is not initialized");
  }

  pollfd pfd{};
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  const int ret = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (ret < 0) {
    if (errno == EINTR) {
      return std::nullopt;
    }
    std::cerr << "SocketCANBackend poll error: " << std::strerror(errno) << "\n";
    return std::nullopt;
  }
  if (ret == 0) {
    return std::nullopt;
  }
  if ((pfd.revents & POLLIN) == 0) {
    return std::nullopt;
  }

  can_frame frame{};
  const ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
  if (n < 0) {
    if (errno == EINTR) {
      return std::nullopt;
    }
    std::cerr << "SocketCANBackend read error: " << std::strerror(errno) << "\n";
    return std::nullopt;
  }
  if (n != static_cast<ssize_t>(sizeof(frame))) {
    return std::nullopt;
  }

  CanMessage msg{};
  msg.is_extended_id = (frame.can_id & CAN_EFF_FLAG) != 0;
  msg.arbitration_id =
      msg.is_extended_id ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
  msg.dlc = frame.can_dlc;
  for (std::size_t i = 0; i < msg.dlc && i < msg.data.size(); ++i) {
    msg.data[i] = frame.data[i];
  }
  return msg;
#endif
}

void SocketCANBackend::close() {
#ifdef __linux__
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
#else
  socket_fd_ = -1;
#endif
}

bool SocketCANBackend::is_open() const {
  return socket_fd_ >= 0;
}

}  // namespace linkerhand
