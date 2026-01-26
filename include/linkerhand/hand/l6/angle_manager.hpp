#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "linkerhand/manager_base.hpp"

namespace linkerhand::hand::l6 {

/// 角度数据结构
struct AngleData {
  std::array<int, 6> angles{};  ///< 6个关节的角度值 (0-255)
  double timestamp = 0.0;       ///< 数据接收时的 Unix 时间戳
};

/// L6 机械手角度管理器
class AngleManager : public StreamableManagerBase<AngleManager, AngleData> {
 public:
  AngleManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
      : StreamableManagerBase(arbitration_id, dispatcher) {}
  ~AngleManager() = default;

  AngleManager(const AngleManager&) = delete;
  AngleManager& operator=(const AngleManager&) = delete;

  /// 设置角度值
  /// @param angles 6个关节的角度数组 (0-255)
  void set_angles(const std::array<int, 6>& angles);
  void set_angles(const std::vector<int>& angles);

  /// 阻塞式获取角度数据
  /// @param timeout_ms 超时时间（毫秒）
  /// @return 角度数据
  AngleData get_angles_blocking(double timeout_ms = 100) {
    return get_blocking(timeout_ms, "angle");
  }

  /// 获取当前缓存的角度数据
  /// @return 角度数据（如果有）
  std::optional<AngleData> get_current_angles() const { return get_current(); }

 private:
  friend class StreamableManagerBase<AngleManager, AngleData>;

  void do_send_sense_request();
  std::optional<AngleData> do_parse_message(const CanMessage& msg);
};

}  // namespace linkerhand::hand::l6
