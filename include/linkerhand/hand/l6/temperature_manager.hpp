#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "linkerhand/manager_base.hpp"

namespace linkerhand::hand::l6 {

/// 温度数据结构
struct TemperatureData {
  std::array<int, 6> temperatures{};  ///< 6个关节的温度值
  double timestamp = 0.0;             ///< 数据接收时的 Unix 时间戳
};

/// L6 机械手温度管理器
class TemperatureManager : public StreamableManagerBase<TemperatureManager, TemperatureData> {
 public:
  TemperatureManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
      : StreamableManagerBase(arbitration_id, dispatcher) {}
  ~TemperatureManager() = default;

  TemperatureManager(const TemperatureManager&) = delete;
  TemperatureManager& operator=(const TemperatureManager&) = delete;

  /// 阻塞式获取温度数据
  /// @param timeout_ms 超时时间（毫秒）
  /// @return 温度数据
  TemperatureData get_temperatures_blocking(double timeout_ms = 100) {
    return get_blocking(timeout_ms, "temperature");
  }

  /// 获取当前缓存的温度数据
  /// @return 温度数据（如果有）
  std::optional<TemperatureData> get_current_temperatures() const { return get_current(); }

 private:
  friend class StreamableManagerBase<TemperatureManager, TemperatureData>;

  void do_send_sense_request();
  std::optional<TemperatureData> do_parse_message(const CanMessage& msg);
};

}  // namespace linkerhand::hand::l6
