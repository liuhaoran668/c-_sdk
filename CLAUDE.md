# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

LinkerHand C++ SDK 是一个用于控制 LinkerHand 机械手的底层通信库，通过 Linux SocketCAN 接口与硬件通信。

**技术栈**: C++17 / CMake 3.16+ / Linux SocketCAN

## 常用命令

### 构建
```bash
cmake -S . -B build_local
cmake --build build_local
```

### 运行测试程序
```bash
./build_local/test_force_sensor
./build_local/test_angle
```

### CAN 接口配置 (Linux)
```bash
scripts/setup_can.sh can0 1000000
```

## 代码架构

### 目录结构
```
include/linkerhand/
├── linkerhand.hpp          # 统一入口
├── can_dispatcher.hpp      # CAN 消息调度器
├── lifecycle.hpp           # 生命周期管理
├── exceptions.hpp          # 异常类型
├── iterable_queue.hpp      # 线程安全阻塞队列
└── hand/l6/                # L6 手型模块
    ├── l6.hpp              # Facade 类
    ├── angle_manager.hpp   # 角度管理
    └── force_sensor_manager.hpp  # 力传感器

src/
├── can_dispatcher.cpp
└── hand/
    ├── common.hpp          # 内部共享工具
    └── l6/*.cpp
```

### 核心组件关系
```
L6 (Facade)
├── CANMessageDispatcher  (CAN 消息收发，单 recv 线程)
├── Lifecycle             (共享生命周期状态 Open/Closed)
├── AngleManager          (角度控制/查询)
└── ForceSensorManager    (力传感器数据采集)
```

### 线程模型
- `recv_thread_`: CANMessageDispatcher 的接收线程，poll CAN socket 并串行执行订阅回调
- `streaming_thread`: 各 Manager 的流式数据采集线程
- 回调在 `recv_thread_` 中执行，不应阻塞

### 关键设计模式
- **Pimpl**: Manager 使用 Pimpl 隐藏实现
- **订阅者快照分发**: recv_loop 复制订阅者列表后释放锁，允许回调中安全调用 subscribe/unsubscribe
- **架构保证安全退订**: `stop()` join `recv_thread_` 后 Manager 才析构，无需复杂同步机制

## 代码风格

- **类型/类**: `PascalCase` (如 `CANMessageDispatcher`)
- **方法/函数**: `snake_case` (如 `get_angles_blocking`)
- **常量**: `kPrefix` 风格 (如 `kControlCmd`)
- **异常**: 使用 `StateError`, `ValidationError`, `CANError`, `TimeoutError`
- **线程安全**: `std::mutex + lock_guard`；并发状态用 `std::atomic`
- **析构**: 吞掉异常，避免析构传播

## 平台限制

仅支持 Linux (SocketCAN)。非 Linux 平台构造时抛出 `CANError`。
