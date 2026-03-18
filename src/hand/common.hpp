#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "linkerhand/exceptions.hpp"

namespace linkerhand::hand::detail {

inline double now_unix_seconds() {
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

template <std::size_t N>
inline void validate_fixed_u8_array(
    const std::array<int, N>& values,
    int max_inclusive,
    const std::string& label) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] < 0 || values[i] > max_inclusive) {
      throw ValidationError(label + " " + std::to_string(i) + " value " +
                            std::to_string(values[i]) + " out of range [0, " +
                            std::to_string(max_inclusive) + "]");
    }
  }
}

}  // namespace linkerhand::hand::detail
