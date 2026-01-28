#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace linkerhand {

struct CanMessage {
  std::uint32_t arbitration_id = 0;
  bool is_extended_id = false;
  std::array<std::uint8_t, 8> data{};
  std::size_t dlc = 0;

  std::vector<std::uint8_t> data_bytes() const {
    return std::vector<std::uint8_t>(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(dlc));
  }
};

}  // namespace linkerhand

