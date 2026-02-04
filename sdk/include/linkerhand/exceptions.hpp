#pragma once

#include <stdexcept>
#include <string>

namespace linkerhand {

class LinkerHandError : public std::runtime_error {
 public:
  explicit LinkerHandError(const std::string& message) : std::runtime_error(message) {}
};

class TimeoutError : public LinkerHandError {
 public:
  explicit TimeoutError(const std::string& message) : LinkerHandError(message) {}
};

class CANError : public LinkerHandError {
 public:
  explicit CANError(const std::string& message) : LinkerHandError(message) {}
};

class ValidationError : public LinkerHandError {
 public:
  explicit ValidationError(const std::string& message) : LinkerHandError(message) {}
};

class StateError : public LinkerHandError {
 public:
  explicit StateError(const std::string& message) : LinkerHandError(message) {}
};

}  // namespace linkerhand

