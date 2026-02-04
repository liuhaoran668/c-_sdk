# LinkerHand ROS2 封装设计文档

## 概述

本文档描述 LinkerHand C++ SDK 的 ROS2 封装设计方案，目标是为 ROS2 用户提供易用的机械手控制接口，同时保持 SDK 的独立性和可维护性。

## 设计目标

1. **单一维护点**：仅需维护纯 C++ SDK，ROS2 层调用 SDK 接口
2. **ROS2 生态兼容**：使用标准消息类型，便于与 MoveIt、rviz 等工具集成
3. **易于扩展**：SDK 新增功能时，ROS2 层可快速跟进
4. **简洁架构**：遵循 KISS、YAGNI 原则，避免过度设计

## 关键决策

| 决策项 | 选择 |
|--------|------|
| 通信方式 | Topic 发布/订阅 |
| 代码组织 | 单一仓库，`sdk/` + `ros2/` 子目录分离 |
| ROS2 版本 | 仅支持 Humble |
| 关节数据消息 | `sensor_msgs/JointState` |
| 力传感器消息 | `linkerhand_msgs/ForceSensors`（自定义） |
| 控制命令消息 | `sensor_msgs/JointState` |
| 数据读取方式 | 定时器 + SDK 阻塞读取 |
| 节点模式 | 单节点单手，通过参数配置 |
| 错误处理 | ROS2 日志 + Diagnostics Topic |

---

## 项目结构

```
linkerhand-sdk/
├── sdk/                              # 纯 C++ SDK（现有代码）
│   ├── CMakeLists.txt
│   ├── include/linkerhand/
│   │   ├── linkerhand.hpp
│   │   ├── can_dispatcher.hpp
│   │   ├── lifecycle.hpp
│   │   ├── iterable_queue.hpp
│   │   ├── exceptions.hpp
│   │   └── hand/l6/
│   │       ├── l6.hpp
│   │       ├── angle_manager.hpp
│   │       └── force_sensor_manager.hpp
│   └── src/
│
├── ros2/                             # ROS2 封装（新增）
│   ├── linkerhand_msgs/              # 自定义消息包
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── msg/
│   │       ├── ForceSensorData.msg
│   │       └── ForceSensors.msg
│   │
│   └── linkerhand_ros2/              # 节点实现包
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/linkerhand_ros2/
│       │   └── linkerhand_node.hpp
│       ├── src/
│       │   └── linkerhand_node.cpp
│       ├── config/
│       │   └── linkerhand_params.yaml
│       └── launch/
│           └── linkerhand.launch.py
│
└── README.md
```

---

## Topic 设计

### Topic 命名规范

```
/linkerhand/{side}/{data_type}
```

- `{side}`: `left` 或 `right`
- `{data_type}`: 数据类型名称

### 当前 Topic 列表

| Topic | 消息类型 | 方向 | 说明 |
|-------|----------|------|------|
| `/linkerhand/left/joint_states` | `sensor_msgs/JointState` | 发布 | 关节状态（角度/速度/力矩） |
| `/linkerhand/left/cmd_joint_states` | `sensor_msgs/JointState` | 订阅 | 关节控制命令（使用 position 字段） |
| `/linkerhand/left/force_sensors` | `linkerhand_msgs/ForceSensors` | 发布 | 力传感器数据 |
| `/linkerhand/left/diagnostics` | `diagnostic_msgs/DiagnosticStatus` | 发布 | 诊断状态 |

### 关节命名

```cpp
{"thumb", "index", "middle", "ring", "little", "thumb_rotation"}
```

---

## 消息定义

### ForceSensorData.msg

```
std_msgs/Header header
string finger_name              # 手指名称
float64[] pressure_values       # 压力传感器阵列数据
```

### ForceSensors.msg

```
std_msgs/Header header
ForceSensorData[] fingers       # 所有手指的力传感器数据
```

---

## 节点设计

### 类结构

```cpp
class LinkerHandNode : public rclcpp::Node {
public:
    LinkerHandNode(const rclcpp::NodeOptions& options);
    ~LinkerHandNode();

private:
    // SDK 实例
    std::unique_ptr<linkerhand::hand::l6::L6> hand_;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<linkerhand_msgs::msg::ForceSensors>::SharedPtr force_sensors_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_pub_;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // 参数
    std::vector<std::string> joint_names_;
    bool enable_force_sensors_;
    double timeout_ms_;

    // 回调
    void publish_callback();
    void cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};
```

### 数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                      LinkerHandNode                              │
│                                                                  │
│  ┌─────────────┐     ┌─────────────────────────────────────┐    │
│  │ SDK (L6)    │     │           ROS2 层                    │    │
│  │             │     │                                      │    │
│  │ angle.      │     │  ┌──────────┐    ┌───────────────┐  │    │
│  │ get_angles  │◄───────│ Timer    │───►│ joint_state   │───────► /joint_states
│  │ _blocking() │     │  │ (100Hz)  │    │ _pub_         │  │    │
│  │             │     │  └──────────┘    └───────────────┘  │    │
│  │             │     │        │                            │    │
│  │ force_sensor│     │        │         ┌───────────────┐  │    │
│  │ .get_data   │◄────────────┴────────►│ force_sensors │───────► /force_sensors
│  │ _blocking() │     │                  │ _pub_         │  │    │
│  │             │     │                  └───────────────┘  │    │
│  │             │     │                                      │    │
│  │ angle.      │     │  ┌───────────────┐                  │    │
│  │ set_angles()│◄───────│ cmd_sub_      │◄──────────────────────── /cmd_joint_states
│  │             │     │  └───────────────┘                  │    │
│  └─────────────┘     └─────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 参数配置

### linkerhand_params.yaml

```yaml
linkerhand_node:
  ros__parameters:
    # 基础配置
    side: "left"                    # "left" 或 "right"
    can_interface: "can0"           # CAN 接口名称

    # 发布配置
    publish_rate: 100.0             # 发布频率 (Hz)
    timeout_ms: 50.0                # 阻塞读取超时 (ms)

    # 功能开关
    enable_force_sensors: true

    # 关节名称
    joint_names:
      - "thumb"
      - "index"
      - "middle"
      - "ring"
      - "little"
      - "thumb_rotation"
```

---

## Launch 文件

### linkerhand.launch.py

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='left'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('publish_rate', default_value='100.0'),
        DeclareLaunchArgument('enable_force_sensors', default_value='true'),

        Node(
            package='linkerhand_ros2',
            executable='linkerhand_node',
            name='linkerhand_node',
            namespace=LaunchConfiguration('side'),
            parameters=[{
                'side': LaunchConfiguration('side'),
                'can_interface': LaunchConfiguration('can_interface'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'enable_force_sensors': LaunchConfiguration('enable_force_sensors'),
            }],
            output='screen',
        ),
    ])
```

### 使用方式

```bash
# 启动左手
ros2 launch linkerhand_ros2 linkerhand.launch.py side:=left can_interface:=can0

# 启动右手
ros2 launch linkerhand_ros2 linkerhand.launch.py side:=right can_interface:=can1
```

---

## 构建配置

### ros2/linkerhand_msgs/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(linkerhand_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ForceSensorData.msg"
    "msg/ForceSensors.msg"
    DEPENDENCIES std_msgs
)

ament_package()
```

### ros2/linkerhand_ros2/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(linkerhand_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(linkerhand_msgs REQUIRED)

# 引入 SDK
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../sdk ${CMAKE_CURRENT_BINARY_DIR}/sdk)

# 节点可执行文件
add_executable(linkerhand_node src/linkerhand_node.cpp)
target_link_libraries(linkerhand_node linkerhand_cpp_sdk)
ament_target_dependencies(linkerhand_node
    rclcpp
    sensor_msgs
    diagnostic_msgs
    linkerhand_msgs
)

# 安装
install(TARGETS linkerhand_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## 错误处理

### 策略

| 级别 | 场景 | 处理方式 |
|------|------|----------|
| OK | 正常运行 | 定期发布诊断状态 |
| WARN | 读取超时 | 日志警告（节流），继续运行 |
| ERROR | CAN 通信故障 | 日志错误，发布诊断状态 |

### 日志节流

使用 `RCLCPP_WARN_THROTTLE` 避免日志刷屏：

```cpp
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
    "Angle read timeout: %s", e.what());
```

---

## 扩展性设计

### SDK 新增功能时的扩展流程

当 SDK 添加新功能（如电流、温度读取）时：

1. **新增 Publisher**：节点增加对应 publisher
2. **增加读取逻辑**：`publish_callback` 中增加读取和发布代码
3. **参数开关**：添加 `enable_xxx` 参数，默认关闭
4. **新增 Topic**：独立 Topic，不影响现有接口

### 示例：新增电流读取

```cpp
// 1. 新增 publisher
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr current_pub_;

// 2. 初始化
current_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("joint_currents", 10);
enable_current_ = declare_parameter("enable_current", false);

// 3. publish_callback 中增加
if (enable_current_) {
    auto data = hand_->current.get_current_blocking(timeout_ms_);
    std_msgs::msg::Float64MultiArray msg;
    msg.data = data.values;
    current_pub_->publish(msg);
}
```

### 扩展后的 Topic 列表

```
# 现有
/linkerhand/left/joint_states
/linkerhand/left/cmd_joint_states
/linkerhand/left/force_sensors
/linkerhand/left/diagnostics

# 未来按需添加
/linkerhand/left/joint_currents      # std_msgs/Float64MultiArray
/linkerhand/left/joint_temperatures  # std_msgs/Float64MultiArray
```

---

## 实现计划

### Phase 1：基础框架
- [ ] 重构目录结构（`sdk/` + `ros2/`）
- [ ] 创建 `linkerhand_msgs` 消息包
- [ ] 创建 `linkerhand_ros2` 节点框架

### Phase 2：核心功能
- [ ] 实现关节状态发布（JointState）
- [ ] 实现控制命令订阅
- [ ] 实现力传感器发布

### Phase 3：完善
- [ ] 实现诊断状态发布
- [ ] 添加 Launch 文件
- [ ] 添加参数配置
- [ ] 编写使用文档

### Phase 4：测试
- [ ] 单元测试
- [ ] 集成测试
- [ ] 实机验证

---

## 附录：角度单位转换

SDK 使用整数角度值（0-255 或特定范围），ROS2 使用弧度：

```cpp
// SDK 角度值 -> 弧度
std::vector<double> to_radians(const std::array<int, 6>& angles) {
    std::vector<double> result(6);
    for (size_t i = 0; i < 6; ++i) {
        // 根据实际 SDK 角度范围调整转换公式
        result[i] = angles[i] * M_PI / 180.0;  // 假设 SDK 使用度数
    }
    return result;
}

// 弧度 -> SDK 角度值
std::array<int, 6> from_radians(const std::vector<double>& radians) {
    std::array<int, 6> result{};
    for (size_t i = 0; i < 6; ++i) {
        result[i] = static_cast<int>(radians[i] * 180.0 / M_PI);
    }
    return result;
}
```

---

**文档版本**：1.0
**创建日期**：2026-02-04
**状态**：待实施
