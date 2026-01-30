# LinkerHand C++ SDK 设计文档

## 1. 项目概述

本项目是 LinkerHand Python SDK 到 C++ 的完整移植，用于控制 L6/O6 机械手。SDK 通过 CAN 总线与硬件通信，提供角度控制、扭矩控制、速度控制、温度/电流读取、力传感器数据采集等功能。

### 1.1 设计目标

| 目标 | 说明 |
|------|------|
| **接口一致性** | C++ 接口与 Python 版本保持 1:1 映射，降低学习成本 |
| **线程安全** | 在对象生命周期内支持多线程并发调用；`close()/stop()/stop_streaming()` 幂等且可并发；对象析构与并发调用需由调用方保证同步 |
| **资源安全** | RAII 机制确保资源自动释放，避免泄漏 |
| **低复杂度** | 避免过度设计，代码易读易维护 |

### 1.2 技术栈

- **语言标准**: C++17
- **通信协议**: Linux SocketCAN
- **并发模型**: std::thread + std::mutex + std::condition_variable
- **构建系统**: CMake 3.16+

---

## 2. 架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                      Application Layer                       │
│                   (用户代码调用 L6/O6 API)                    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      Facade Layer                            │
│                      L6 / O6 类                              │
│            (组合各 Manager，提供统一入口)                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                     Manager Layer                            │
│   AngleManager | TorqueManager | SpeedManager | ...          │
│         (各功能模块的具体实现，订阅 CAN 消息)                   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  Communication Layer                         │
│                  CANMessageDispatcher                        │
│            (CAN 总线收发，消息分发给订阅者)                     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Layer                            │
│                  Linux SocketCAN                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 设计模式

### 3.1 发布-订阅模式 (Publish-Subscribe Pattern)

**应用位置**: `CANMessageDispatcher`

**问题**: 多个 Manager 需要接收 CAN 总线消息，但消息来源只有一个。

**解决方案**:
- `CANMessageDispatcher` 作为消息中心，后台线程持续接收 CAN 消息
- 各 Manager 通过 `subscribe()` 注册回调函数
- 收到消息后，Dispatcher 遍历所有订阅者并调用其回调

```
┌──────────────────┐
│ CANMessageDispatcher │
│   (Publisher)    │
└────────┬─────────┘
         │ notify
         ▼
┌────────┴─────────┬─────────────┬─────────────┐
│                  │             │             │
▼                  ▼             ▼             ▼
AngleManager   TorqueManager  SpeedManager  ForceSensorManager
(Subscriber)   (Subscriber)   (Subscriber)   (Subscriber)
```

**优势**:
- 松耦合：Dispatcher 不关心订阅者的具体类型
- 可扩展：新增 Manager 只需订阅，无需修改 Dispatcher
- 单点接收：避免多个线程同时读取 CAN socket

---

### 3.2 外观模式 (Facade Pattern)

**应用位置**: `L6` / `O6` 类

**问题**: 用户需要同时操作多个子系统（角度、扭矩、速度等），直接暴露所有 Manager 会增加使用复杂度。

**解决方案**:
- `L6`/`O6` 作为统一入口，内部组合所有 Manager
- 用户通过 `hand.angle`、`hand.torque` 等成员访问子系统
- 资源管理（CAN 连接、线程启停）由 Facade 统一处理

```cpp
class L6 {
public:
    AngleManager angle;
    TorqueManager torque;
    SpeedManager speed;
    TemperatureManager temperature;
    CurrentManager current;
    ForceSensorManager force_sensor;
    FaultManager fault;

    void close();  // 统一关闭所有资源
};
```

**优势**:
- 简化接口：用户只需了解 L6/O6 即可使用全部功能
- 生命周期管理：Facade 负责协调各子系统的创建和销毁

---

### 3.3 观察者模式 (Observer Pattern)

**应用位置**: 各 Manager 的阻塞等待机制

**问题**: `get_xxx_blocking()` 需要等待 CAN 响应，但响应在 Dispatcher 线程中到达。

**解决方案**:
- 调用者创建 `Waiter` 对象（包含 condition_variable）
- Waiter 注册到 Manager 的等待列表
- CAN 回调收到数据后，通知所有 Waiter

```
调用线程                           Dispatcher 线程
    │                                   │
    │  1. 创建 Waiter                    │
    │  2. 注册到等待列表                  │
    │  3. 发送 CAN 请求                   │
    │  4. cv.wait() 阻塞                 │
    │      ┃                            │
    │      ┃         ◄──────────────────┤ 5. 收到 CAN 响应
    │      ┃                            │ 6. 遍历 Waiter 列表
    │      ┃         ◄──────────────────┤ 7. cv.notify_one()
    │      ┃                            │
    │  8. 获取数据返回                    │
    ▼                                   ▼
```

**优势**:
- 非轮询：使用条件变量高效等待
- 多等待者支持：多个线程可同时等待同一数据

---

### 3.4 Pimpl 模式 (Pointer to Implementation)

**应用位置**: 所有 Manager 类

**问题**:
- 头文件暴露实现细节会增加编译依赖
- Manager 内部状态复杂（锁、队列、线程），不宜直接暴露

**解决方案**:
- 头文件只声明公开接口和 `std::unique_ptr<Impl>` 成员
- 实现细节封装在 `.cpp` 文件的 `Impl` 结构体中

```cpp
// angle_manager.hpp
class AngleManager {
public:
    void set_angles(const std::array<int, 6>& angles);
    AngleData get_angles_blocking(double timeout_ms = 100);
    // ...
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// angle_manager.cpp
struct AngleManager::Impl {
    std::mutex waiters_mutex;
    std::vector<std::shared_ptr<Waiter>> waiters;
    std::optional<IterableQueue<AngleData>> streaming_queue;
    // ... 所有实现细节
};
```

**优势**:
- 编译隔离：修改实现不触发头文件重编译
- ABI 稳定：二进制接口不随实现变化
- 封装性：实现细节对用户完全隐藏

---

### 3.5 RAII 模式 (Resource Acquisition Is Initialization)

**应用位置**: 全局资源管理

**问题**: CAN 连接、线程等资源需要正确释放，手动管理容易出错。

**解决方案**:
- 构造函数获取资源（打开 socket、启动线程）
- 析构函数释放资源（关闭 socket、join 线程）
- 禁用拷贝，避免资源所有权混乱

```cpp
class CANMessageDispatcher {
public:
    CANMessageDispatcher(const std::string& interface_name, ...);
    ~CANMessageDispatcher();  // 自动 stop() 并关闭 socket

    // 禁用拷贝
    CANMessageDispatcher(const CANMessageDispatcher&) = delete;
    CANMessageDispatcher& operator=(const CANMessageDispatcher&) = delete;
};
```

**优势**:
- 异常安全：即使发生异常，析构函数仍会执行
- 无需手动清理：用户不必显式调用 close()

---

### 3.6 不可变数据模式 (Immutable Data Pattern)

**应用位置**: 数据传输对象（AngleData, TorqueData, ForceSensorData 等）

**问题**: 数据在多线程间传递，可变数据需要加锁保护。

**解决方案**:
- 数据结构设计为值语义，创建后不可修改
- 传递时使用拷贝而非引用

```cpp
struct AngleData {
    std::array<int, 6> angles{};  // 值类型，拷贝安全
    double timestamp = 0.0;
};

// Python 中使用 @dataclass(frozen=True)
// C++ 中通过值语义 + 不提供 setter 实现
```

**优势**:
- 线程安全：无需加锁即可在线程间传递
- 可预测性：数据创建后状态不变

---

### 3.7 组合帧模式 (Frame Assembly Pattern)

**应用位置**: `ForceSensorManager` 的多帧数据组装

**问题**: 力传感器数据分 12 帧 CAN 消息传输，需要组装成完整的 72 字节数据。

**解决方案**:
- `FrameBatch` 结构体累积收到的帧
- 每帧根据帧序号放入对应位置
- 收齐 12 帧后组装并触发回调

```
CAN Frame 0  ──┐
CAN Frame 1  ──┤
CAN Frame 2  ──┤
    ...       ├──► FrameBatch ──► assemble() ──► ForceSensorData (72 bytes)
CAN Frame 10 ──┤
CAN Frame 11 ──┘
```

**优势**:
- 容错性：帧可乱序到达
- 解耦：组装逻辑独立于 CAN 接收逻辑

---

### 3.8 生产者-消费者模式 (Producer-Consumer Pattern)

**应用位置**: `IterableQueue` 流式数据传输

**问题**: Manager 持续产生数据，用户需要异步消费。

**解决方案**:
- 线程安全的有界队列
- 生产者（CAN 回调线程）调用 `put()`
- 消费者（用户线程）调用 `get()` 或 for-range 迭代

```cpp
// 生产者
void on_complete_data(const AngleData& data) {
    streaming_queue->put_nowait(data);
}

// 消费者
auto q = hand.angle.stream(100);
for (const auto& data : q) {  // 阻塞等待数据
    process(data);
}
```

**优势**:
- 背压处理：队列满时丢弃旧数据
- 类 Go channel 语义：支持 for-range 迭代

---

## 4. 关键设计决策

### 4.1 为什么使用 Pimpl 而非继承？

| 对比项 | Pimpl | 继承 |
|--------|-------|------|
| 编译依赖 | ✅ 隔离 | ❌ 暴露基类 |
| 运行时开销 | 一次指针解引用 | 虚函数表查找 |
| 扩展性 | 内部扩展 | 需要新子类 |
| 复杂度 | 低 | 高（需设计继承层次）|

**结论**: 各 Manager 功能相似但协议细节不同，Pimpl 比继承更适合。

### 4.2 为什么订阅返回 ID 而非移除回调？

```cpp
// 方案 A: 通过回调指针移除（Python 风格）
void unsubscribe(Callback callback);  // ❌ std::function 不可比较

// 方案 B: 通过 ID 移除（采用）
std::size_t subscribe(Callback callback);
void unsubscribe(std::size_t subscription_id);  // ✅
```

**原因**: `std::function` 不支持相等比较，无法直接通过回调对象移除。

### 4.3 为什么流式模式返回队列拷贝？

```cpp
IterableQueue<AngleData> stream(...);  // 返回值（队列内部共享状态）
```

**原因**:
- `IterableQueue` 内部使用 `shared_ptr<State>` 管理状态
- 拷贝队列对象只是拷贝智能指针，多个拷贝共享同一状态
- 用户可安全持有队列，Manager 内部也持有引用

---

## 5. 线程模型

```
┌─────────────────────────────────────────────────────────────┐
│                      用户线程                                │
│   - 调用 set_angles(), get_angles_blocking()                │
│   - 迭代 IterableQueue                                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ 阻塞等待 / 队列读取
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Dispatcher 接收线程                        │
│   - 循环 poll() + read() CAN socket                         │
│   - 调用所有订阅者回调                                        │
│   - 唤醒阻塞等待者 / 推送数据到队列                            │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ 流式模式
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Streaming 请求线程                         │
│   - 每个 Manager 的 stream() 创建独立线程                     │
│   - 周期性发送感知请求                                        │
└─────────────────────────────────────────────────────────────┘
```

**锁粒度设计**:
- `subscribers_mutex_`: 保护订阅者列表
- `waiters_mutex`: 保护阻塞等待者列表
- `latest_mutex`: 保护最新数据缓存
- `streaming_mutex`: 保护流式状态

**生命周期与析构约束（要点）**:
- `stop_streaming()` 会关闭 `IterableQueue` 以唤醒消费者，并 join 内部线程，避免析构时遗留后台线程。
- `unsubscribe()` 需要与回调执行并发安全协调（避免析构期间 UAF）；如在回调栈内调用需避免触发“销毁回调目标对象”的行为。

---

## 6. 与 Python 版本的差异

| 特性 | Python | C++ |
|------|--------|-----|
| 资源管理 | `with` 语句 / 手动 `close()` | RAII 析构函数自动清理 |
| 类型系统 | 动态类型 + 类型注解 | 静态类型 + 编译期检查 |
| 数据容器 | `tuple` / `dataclass` | `std::array` / `struct` |
| 异常类型 | 继承 `Exception` | 继承 `std::runtime_error` |
| 队列迭代 | `for data in queue` | `for (auto& data : queue)` |
| 平台支持 | python-can 多后端 | 仅 Linux SocketCAN |

---

## 7. 扩展指南

### 7.1 添加新的 Manager

1. 在 `include/linkerhand/hand/l6/` 创建头文件
2. 定义数据结构（如 `NewData`）和 Manager 类
3. 在 `src/hand/l6/` 创建实现文件
4. 在 `L6` 类中添加成员
5. 更新 CMakeLists.txt（GLOB_RECURSE 自动包含）

### 7.2 支持新的 CAN 后端

1. 在 `can_dispatcher.cpp` 添加条件编译分支
2. 实现对应平台的 socket 创建、读写逻辑
3. 保持 `CanMessage` 结构不变

---

## 8. 总结

本 SDK 的 C++ 实现遵循以下设计原则：

1. **KISS** - 避免不必要的抽象，Manager 之间允许适度代码重复
2. **单一职责** - 每个类只做一件事（Dispatcher 收发、Manager 处理协议）
3. **依赖倒置** - Manager 依赖 Dispatcher 抽象，而非具体实现
4. **接口隔离** - 用户只需了解 L6/O6 公开接口

通过合理运用设计模式，在保持与 Python 版本接口一致的同时，充分利用了 C++ 的类型安全和 RAII 机制。
