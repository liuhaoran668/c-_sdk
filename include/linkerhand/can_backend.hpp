#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "linkerhand/can_message.hpp"

namespace linkerhand {

class CANBackend {
 public:
  virtual ~CANBackend() = default;

  virtual void send(const CanMessage& msg) = 0;
  virtual std::optional<CanMessage> recv(std::chrono::milliseconds timeout) = 0;

  virtual void close() = 0;
  virtual bool is_open() const = 0;
};

std::unique_ptr<CANBackend> create_backend(
    const std::string& interface_name,
    const std::string& interface_type);

}  // namespace linkerhand

