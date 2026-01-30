#include "linkerhand/hand/l6/l6.hpp"

namespace linkerhand::hand::l6 {

L6::L6(const std::string& side, const std::string& interface_name, const std::string& interface_type)
    : dispatcher_(interface_name, interface_type),
      arbitration_id_(side == "right" ? 0x27 : 0x28),
      closed_(false),
      angle(arbitration_id_, dispatcher_),
      force_sensor(arbitration_id_, dispatcher_),
      torque(arbitration_id_, dispatcher_),
      speed(arbitration_id_, dispatcher_),
      temperature(arbitration_id_, dispatcher_),
      current(arbitration_id_, dispatcher_),
      fault(arbitration_id_, dispatcher_) {}

L6::~L6() {
  try {
    close();
  } catch (...) {
  }
}

void L6::close() {
  if (closed_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }

  try {
    force_sensor.stop_streaming();
    angle.stop_streaming();
    torque.stop_streaming();
    temperature.stop_streaming();
    current.stop_streaming();
  } catch (...) {
  }

  try {
    dispatcher_.stop();
  } catch (...) {
  }
}

bool L6::is_closed() const { return closed_.load(std::memory_order_acquire); }

void L6::ensure_open() const {
  if (closed_.load(std::memory_order_acquire)) {
    throw StateError(
        "L6 interface is closed. Create a new instance or use context manager.");
  }
}

}  // namespace linkerhand::hand::l6
