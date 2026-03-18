#include "linkerhand/hand/l6/l6.hpp"

#include <utility>

namespace linkerhand::hand::l6 {

L6::L6(std::string side, std::string interface_name)
    : lifecycle_(std::make_shared<linkerhand::Lifecycle>("L6")),
      dispatcher_(std::move(interface_name)),
      arbitration_id_(side == "right" ? 0x27 : 0x28),
      angle(arbitration_id_, dispatcher_, lifecycle_),
      force_sensor(arbitration_id_, dispatcher_, lifecycle_) {}

L6::~L6() {
  try {
    close();
  } catch (...) {
  }
}

void L6::close() {
  if (!lifecycle_->close()) {
    return;
  }

  try {
    force_sensor.stop_streaming();
    angle.stop_streaming();
  } catch (...) {
  }

  try {
    dispatcher_.stop();
  } catch (...) {
  }
}

bool L6::is_closed() const {
  return lifecycle_->is_closed();
}

}  // namespace linkerhand::hand::l6
