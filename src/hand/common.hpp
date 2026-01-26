#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "linkerhand/exceptions.hpp"

namespace linkerhand::hand::detail {

inline double now_unix_seconds() {
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

inline void validate_fixed_u8_vector(
    const std::vector<int>& values,
    std::size_t expected_size,
    int max_inclusive,
    const std::string& collection_label,
    const std::string& element_label) {
  if (values.size() != expected_size) {
    throw ValidationError("Expected " + std::to_string(expected_size) + " " + collection_label +
                          ", got " + std::to_string(values.size()));
  }
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] < 0 || values[i] > max_inclusive) {
      throw ValidationError(element_label + " " + std::to_string(i) + " value " +
                            std::to_string(values[i]) + " out of range [0, " +
                            std::to_string(max_inclusive) + "]");
    }
  }
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

template <std::size_t N>
inline std::array<std::uint8_t, N> to_u8_array(const std::array<int, N>& values) {
  std::array<std::uint8_t, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = static_cast<std::uint8_t>(values[i]);
  }
  return out;
}

inline std::array<std::uint8_t, 6> to_u8_array_6(const std::vector<int>& values) {
  std::array<std::uint8_t, 6> out{};
  for (std::size_t i = 0; i < 6; ++i) {
    out[i] = static_cast<std::uint8_t>(values[i]);
  }
  return out;
}

}  // namespace linkerhand::hand::detail
