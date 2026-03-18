# LinkerHand C++ SDK 架构设计文档

> **文档版本**: 2.0
> **目标读者**: 架构师、高级开发者

---

## 1. 项目概览

### 1.1 项目定位与核心职责

LinkerHand C++ SDK 是一个用于控制 LinkerHand 机械手的底层通信库，通过 Linux SocketCAN 接口与硬件进行通信。项目提供：

- **CAN 总线通信抽象**: 封装 SocketCAN 操作，提供发布-订阅模式的消息分发
- **手型控制 API**: 支持 L6（六自由度灵巧手）等手型
- **传感器数据采集**: 角度、力传感器数据

### 1.2 技术栈与依赖

| 组件 | 技术选型 |
|------|----------|
| 语言标准 | C++17 |
| 构建系统 | CMake 3.16+ |
| 线程库 | POSIX Threads (pthread) |
| CAN 通信 | Linux SocketCAN |
| 平台支持 | 仅 Linux |

### 1.3 目录结构与模块划分

```
include/linkerhand/
├── linkerhand.hpp          # 统一入口头文件
├── can_dispatcher.hpp      # CAN 消息调度器
├── lifecycle.hpp           # 生命周期管理
├── exceptions.hpp          # 异常类型定义
├── iterable_queue.hpp      # 可迭代阻塞队列
└── hand/l6/                # L6 手型模块
    ├── l6.hpp
    ├── angle_manager.hpp
    └── force_sensor_manager.hpp

src/
├── can_dispatcher.cpp
└── hand/
    ├── common.hpp          # 内部共享工具
    └── l6/*.cpp
```

---

## 2. 核心架构设计

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Application Layer                           │
│                     (L6 Hand Interface)                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    L6 (Facade)                               │   │
│  │  ┌─────────┐ ┌─────────┐                                   │   │
│  │  │ Angle   │ │ Force   │                                   │   │
│  │  │ Manager │ │ Sensor  │                                   │   │
│  │  └────┬────┘ └────┬────┘                                   │   │
│  │       └───────────┘                                         │   │
│  │             │ subscribe/send                                 │   │
│  └─────────────┼───────────────────────────────────────────────┘   │
│                │                                                    │
│  ┌─────────────▼───────────────────────────────────────────────┐   │
│  │                  CANMessageDispatcher                        │   │
│  │  ┌──────────────┐    ┌───────────────┐    ┌──────────────┐  │   │
│  │  │ recv_thread_ │───▶│ subscribers_  │───▶│  Callbacks   │  │   │
│  │  │ (poll loop)  │    │ (vector)      │    │  (dispatch)  │  │   │
│  │  └──────────────┘    └───────────────┘    └──────────────┘  │   │
│  │         ▲                                                    │   │
│  └─────────┼────────────────────────────────────────────────────┘   │
│            │                                                        │
├────────────┼────────────────────────────────────────────────────────┤
│            │                  System Layer                          │
│  ┌─────────▼─────────┐                                             │
│  │  Linux SocketCAN  │                                             │
│  └───────────────────┘                                             │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 核心类职责与协作关系

| 类名 | 职责 | 关键依赖 |
|------|------|----------|
| `L6` | 外观类，聚合所有 Manager，管理整体生命周期 | `CANMessageDispatcher` |
| `CANMessageDispatcher` | CAN 消息收发与分发，管理接收线程 | Linux SocketCAN |
| `AngleManager` | 6 自由度角度控制/查询 | `CANMessageDispatcher` |
| `ForceSensorManager` | 5 指力传感器数据采集 | `CANMessageDispatcher` |
| `IterableQueue<T>` | 线程安全的阻塞队列，支持 range-based for 迭代 | - |
| `Lifecycle` | 共享生命周期状态 Open/Closed | - |

### 2.3 数据流向

```
[硬件] ──CAN Frame──▶ [SocketCAN] ──read()──▶ [recv_thread_]
                                                    │
                                                    ▼
                                            [on_message 回调]
                                                    │
                                         ┌──────────┴──────────┐
                                         ▼                      ▼
                                  [latest_data_]        [streaming_queue]
                                  (缓存最新值)           (流式数据队列)
                                         │                      │
                                         ▼                      ▼
                                [get_current_*()]      [stream() 迭代器]
```

---

## 3. C++ 惯用法设计决策

### 3.1 类型安全的超时参数

所有超时/间隔参数使用 `std::chrono::milliseconds` 替代 `double`：

```cpp
using namespace std::chrono_literals;
auto data = hand.angle.get_angles_blocking(100ms);
auto queue = hand.angle.stream(50ms, 100);
```

**优点**:
- 编译期类型检查，防止单位混淆
- 支持 `100ms`、`1s` 等字面量
- 与 `std::this_thread::sleep_for`、`condition_variable::wait_for` 等标准库无缝集成

### 3.2 枚举类型安全

使用 `enum class Finger` 和 `FingerArray<T>` 替代 `unordered_map<string, T>`：

```cpp
enum class Finger : uint8_t { Thumb = 0, Index, Middle, Ring, Pinky };
constexpr std::size_t kFingerCount = 5;

template<typename T>
using FingerArray = std::array<T, kFingerCount>;
```

**优点**:
- 编译期索引范围检查
- 零堆分配（`std::array` vs `unordered_map`）
- 可迭代、可索引

### 3.3 返回值替代异常流控

`IterableQueue` 的非阻塞操作使用返回值替代 Python 风格的异常：

| 旧 API (Python 式) | 新 API (C++ 式) |
|---------------------|------------------|
| `put_nowait()` 抛 `QueueFull` | `try_put()` 返回 `bool` |
| `get_nowait()` 抛 `QueueEmpty` | `try_get()` 返回 `std::optional<T>` |
| catch+drop+retry | `force_put()` 自动丢弃最旧 |

### 3.4 原地修改替代不可变拷贝

`FrameBatch` 从 Python 式的"返回新对象"改为 C++ 式的原地修改：

```cpp
// 旧: FrameBatch next = *this; next.frames[id] = data; return next;
// 新: 直接修改内部数组，零拷贝
void add_frame(size_t idx, const uint8_t* payload) {
    std::copy_n(payload, 6, values_.begin() + idx * 6);
}
```

### 3.5 并发多指查询

`ForceSensorManager::get_data_blocking` 使用 `std::async` 并发查询 5 个手指，共享 deadline：

```cpp
auto deadline = steady_clock::now() + timeout;
FingerArray<std::future<ForceSensorData>> futures;
for (size_t i = 0; i < kFingerCount; ++i) {
    futures[i] = std::async(std::launch::async, ...);
}
```

**优点**: 原来串行 5 次超时（最差 5×timeout），现在并发（最差 1×timeout）。

---

## 4. 生命周期契约

### 4.1 资源管理模型

| 方法 | 语义 | 幂等性 |
|------|------|--------|
| `L6::close()` | 停止所有流、关闭 dispatcher | ✅ |
| `CANMessageDispatcher::stop()` | 停止接收线程、关闭 socket | ✅ |
| `*Manager::stop_streaming()` | 停止流式数据采集 | ✅ |
| 析构函数 | 调用 `close()`/`stop()`，吞掉异常 | N/A |

### 4.2 API 行为矩阵

| API 方法 | Open 状态 | Closed 状态 |
|----------|-----------|-------------|
| `set_angles()` | 正常执行 | 抛出 `StateError` |
| `get_*_blocking()` | 正常执行 | 立即抛出 `StateError` |
| `get_current_*()` | 正常执行 | 抛出 `StateError` |
| `stream()` | 正常执行 | 抛出 `StateError` |
| `stop_streaming()` | 正常执行 | **正常执行**（清理操作） |
| `close()` | 执行关闭 | 幂等，直接返回 |

---

## 5. 并发模型与线程安全

### 5.1 线程模型

| 线程 | 所属组件 | 职责 | 生命周期 |
|------|----------|------|----------|
| `recv_thread_` | `CANMessageDispatcher` | 轮询 CAN socket，分发消息 | 构造→stop() |
| `streaming_thread` | 各 `*Manager::Impl` | 周期性发送传感器请求 | stream()→stop_streaming() |
| `aggregation_thread` | `ForceSensorManager::Impl` | 聚合五指数据 | stream()→stop_streaming() |
| `reader_threads` | `ForceSensorManager::Impl` | 从单指队列读取数据 | 随 aggregation_thread |

### 5.2 锁的职责划分

```
┌──────────────────────────────────────────────────────────────────┐
│  subscribers_mutex_       保护 subscribers_ 列表的增删和拷贝      │
├──────────────────────────────────────────────────────────────────┤
│  socket_mutex_            保护 socket_fd_ 的读写和关闭           │
├──────────────────────────────────────────────────────────────────┤
│  recv_thread_join_mutex_  保护 recv_thread_.join() 的幂等性      │
└──────────────────────────────────────────────────────────────────┘

设计要点：各锁独立使用，无嵌套获取，从根本上避免死锁。
```

---

## 6. 关键 API 使用指南

### 6.1 构造与初始化

```cpp
#include <linkerhand/linkerhand.hpp>
using namespace std::chrono_literals;

linkerhand::L6 hand("left", "can0");
```

### 6.2 同步获取数据

```cpp
try {
  auto angles = hand.angle.get_angles_blocking(100ms);
  // angles.angles: std::array<int, 6>
  // angles.timestamp: double (Unix 时间戳)
} catch (const linkerhand::TimeoutError& e) {
  // 超时处理
}

if (auto angles = hand.angle.get_current_angles()) {
  // 使用 angles->angles
}
```

### 6.3 流式数据采集

```cpp
auto queue = hand.angle.stream(50ms, 100);

for (const auto& data : queue) {
  // 处理 data
  if (should_stop) break;
}

hand.angle.stop_streaming();
```

### 6.4 设置角度

```cpp
hand.angle.set_angles({128, 128, 128, 128, 128, 128});
```

### 6.5 力传感器

```cpp
using linkerhand::hand::l6::Finger;

// 阻塞获取所有手指 (并发查询)
auto all = hand.force_sensor.get_data_blocking(1000ms);
auto thumb_data = all[Finger::Thumb];

// 获取缓存
auto latest = hand.force_sensor.get_latest_data();
if (latest[static_cast<size_t>(Finger::Index)].has_value()) {
  // ...
}
```

### 6.6 错误处理

| 异常类型 | 触发场景 |
|----------|----------|
| `LinkerHandError` | 所有异常的基类 |
| `TimeoutError` | 阻塞操作超时 |
| `CANError` | CAN 通信错误 |
| `ValidationError` | 参数校验失败 |
| `StateError` | 状态不正确（如 close 后调用、重复 stream()） |
| `StopIteration` | 队列迭代结束（内部使用） |

---

## 7. 扩展指南

### 7.1 添加新的 Manager

1. 在 `include/linkerhand/hand/{version}/` 下创建头文件
2. 定义 `*Data` 结构体和 `*Manager` 类
3. 在 `src/hand/{version}/` 下实现 Impl
4. 在设备类中添加成员并初始化

### 7.2 添加新型号手

每个型号独立实现，不使用继承：

```
include/linkerhand/hand/
├── l6/     # L6 型号（独立完整）
├── l10/    # L10 型号（独立完整）
└── l20/    # L20 型号（独立完整）
```

在 `linkerhand.hpp` 中注册：

```cpp
#include "linkerhand/hand/l6/l6.hpp"
#include "linkerhand/hand/l10/l10.hpp"

namespace linkerhand {
using L6 = hand::l6::L6;
using L10 = hand::l10::L10;
}
```

---

## 附录 A: 消息协议摘要

| 功能 | CAN ID (left/right) | CMD Byte | 数据格式 |
|------|---------------------|----------|----------|
| 角度控制/查询 | 0x28 / 0x27 | 0x01 | [cmd, angle1..6] |
| 力传感器 (thumb) | 0x28 / 0x27 | 0xB1 | 多帧协议 (12 frames × 6 bytes) |
| 力传感器 (index) | 0x28 / 0x27 | 0xB2 | 同上 |
| 力传感器 (middle) | 0x28 / 0x27 | 0xB3 | 同上 |
| 力传感器 (ring) | 0x28 / 0x27 | 0xB4 | 同上 |
| 力传感器 (pinky) | 0x28 / 0x27 | 0xB5 | 同上 |

---

*文档结束*
