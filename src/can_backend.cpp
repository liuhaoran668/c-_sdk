#include "linkerhand/can_backend.hpp"

#include <memory>

#include "backend/socketcan_backend.hpp"
#include "linkerhand/exceptions.hpp"

namespace linkerhand {

std::unique_ptr<CANBackend> create_backend(
    const std::string& interface_name,
    const std::string& interface_type) {
  if (interface_type == "socketcan") {
    return std::make_unique<SocketCANBackend>(interface_name);
  }
  throw ValidationError("Only 'socketcan' interface_type is supported");
}

}  // namespace linkerhand

