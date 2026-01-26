#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "linkerhand/manager_base.hpp"

namespace linkerhand::hand::l6 {

/// 电流数据结构
struct CurrentData {
  std::array<int, 6> currents{};  ///< 6个关节的电流值
  double timestamp = 0.0;         ///< 数据接收时的 Unix 时间戳
};

/// L6 机械手电流管理器
class CurrentManager : public StreamableManagerBase<CurrentManager, CurrentData> {
 public:
  CurrentManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
      : StreamableManagerBase(arbitration_id, dispatcher) {}
  ~CurrentManager() = default;

  CurrentManager(const CurrentManager&) = delete;
  CurrentManager& operator=(const CurrentManager&) = delete;

  /// 阻塞式获取电流数据
  /// @param timeout_ms 超时时间（毫秒）
  /// @return 电流数据
  CurrentData get_currents_blocking(double timeout_ms = 100) {
    return get_blocking(timeout_ms, "current");
  }

  /// 获取当前缓存的电流数据
  /// @return 电流数据（如果有）
  std::optional<CurrentData> get_current_currents() const { return get_current(); }

 private:
  friend class StreamableManagerBase<CurrentManager, CurrentData>;

  void do_send_sense_request();
  std::optional<CurrentData> do_parse_message(const CanMessage& msg);
};

}  // namespace linkerhand::hand::l6
