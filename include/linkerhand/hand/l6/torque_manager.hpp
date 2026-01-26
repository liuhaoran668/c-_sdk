#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "linkerhand/manager_base.hpp"

namespace linkerhand::hand::l6 {

/// 扭矩数据结构
struct TorqueData {
  std::array<int, 6> torques{};  ///< 6个关节的扭矩值 (0-255)
  double timestamp = 0.0;        ///< 数据接收时的 Unix 时间戳
};

/// L6 机械手扭矩管理器
class TorqueManager : public StreamableManagerBase<TorqueManager, TorqueData> {
 public:
  TorqueManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
      : StreamableManagerBase(arbitration_id, dispatcher) {}
  ~TorqueManager() = default;

  TorqueManager(const TorqueManager&) = delete;
  TorqueManager& operator=(const TorqueManager&) = delete;

  /// 设置扭矩值
  /// @param torques 6个关节的扭矩数组 (0-255)
  void set_torques(const std::array<int, 6>& torques);
  void set_torques(const std::vector<int>& torques);

  /// 阻塞式获取扭矩数据
  /// @param timeout_ms 超时时间（毫秒）
  /// @return 扭矩数据
  TorqueData get_torques_blocking(double timeout_ms = 100) {
    return get_blocking(timeout_ms, "torque");
  }

  /// 获取当前缓存的扭矩数据
  /// @return 扭矩数据（如果有）
  std::optional<TorqueData> get_current_torques() const { return get_current(); }

 private:
  friend class StreamableManagerBase<TorqueManager, TorqueData>;

  void do_send_sense_request();
  std::optional<TorqueData> do_parse_message(const CanMessage& msg);
};

}  // namespace linkerhand::hand::l6
