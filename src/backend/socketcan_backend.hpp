#pragma once

#include <chrono>
#include <optional>
#include <string>

#include "linkerhand/can_backend.hpp"

namespace linkerhand {

class SocketCANBackend final : public CANBackend {
 public:
  explicit SocketCANBackend(const std::string& interface_name);
  ~SocketCANBackend() override;

  SocketCANBackend(const SocketCANBackend&) = delete;
  SocketCANBackend& operator=(const SocketCANBackend&) = delete;
  SocketCANBackend(SocketCANBackend&&) = delete;
  SocketCANBackend& operator=(SocketCANBackend&&) = delete;

  void send(const CanMessage& msg) override;
  std::optional<CanMessage> recv(std::chrono::milliseconds timeout) override;
  void close() override;
  bool is_open() const override;

 private:
  std::string interface_name_;
  int socket_fd_ = -1;
};

}  // namespace linkerhand

