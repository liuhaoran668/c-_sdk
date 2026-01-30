#include "linkerhand/hand/o6/o6.hpp"

namespace linkerhand::hand::o6 {

O6::O6(const std::string& side, const std::string& interface_name, const std::string& interface_type)
    : dispatcher_(interface_name, interface_type),
      arbitration_id_(side == "right" ? 0x27 : 0x28),
      closed_(false),
      angle(arbitration_id_, dispatcher_),
      torque(arbitration_id_, dispatcher_),
      speed(arbitration_id_, dispatcher_),
      temperature(arbitration_id_, dispatcher_) {}

O6::~O6() {
  try {
    close();
  } catch (...) {
  }
}

void O6::close() {
  if (closed_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }

  try {
    angle.stop_streaming();
    temperature.stop_streaming();
  } catch (...) {
  }

  try {
    dispatcher_.stop();
  } catch (...) {
  }
}

bool O6::is_closed() const { return closed_.load(std::memory_order_acquire); }

void O6::ensure_open() const {
  if (closed_.load(std::memory_order_acquire)) {
    throw StateError(
        "O6 interface is closed. Create a new instance or use context manager.");
  }
}

}  // namespace linkerhand::hand::o6
