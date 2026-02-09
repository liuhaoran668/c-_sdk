#include "linkerhand/hand/o6/o6.hpp"

namespace linkerhand::hand::o6 {

O6::O6(const std::string& side, const std::string& interface_name, const std::string& interface_type)
    : lifecycle_(std::make_shared<linkerhand::Lifecycle>("O6")),
      dispatcher_(interface_name, interface_type),
      arbitration_id_(side == "right" ? 0x27 : 0x28),
      angle(arbitration_id_, dispatcher_, lifecycle_),
      force_sensor(arbitration_id_, dispatcher_, lifecycle_) {}

O6::~O6() {
  try {
    close();
  } catch (...) {
  }
}

void O6::close() {
  // Idempotent: lifecycle_->close() returns false if already closed
  if (!lifecycle_->close()) {
    return;
  }

  // Stop all streaming (cleanup operations, allowed after close)
  try {
    force_sensor.stop_streaming();
    angle.stop_streaming();
  } catch (...) {
  }

  // Stop the communication layer
  try {
    dispatcher_.stop();
  } catch (...) {
  }
}

bool O6::is_closed() const {
  return lifecycle_->is_closed();
}

}  // namespace linkerhand::hand::o6
