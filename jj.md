# LinkerHand C++ SDK 并发与生命周期契约（供架构评审）

> **文档版本**: 1.0  
> **更新时间**: 2026-01-29  
> **适用范围**: `jiagou/`（`L6/O6`、`CANMessageDispatcher`、各 `*Manager`、`IterableQueue`）

## 0. 目标

1. **考虑边界情况**：并发 close/stop、回调与析构并发、阻塞等待与关闭并发等。
2. **补齐生命周期契约**：`close/stop/析构` 的关系、是否幂等、close 后调用的统一行为（抛错/返回值约定）。
3. **明确并发/回调语义**：回调执行线程、是否允许阻塞、`subscribe/unsubscribe` 并发策略，以及 stop/析构如何避免竞态与回调冲突。

## 1. 组件与线程模型（事实陈述）

### 1.1 关键组件

| 组件 | 职责 | 关键资源/线程 | 代码位置（示例） |
|------|------|---------------|------------------|
| `L6/O6`（Facade） | 聚合各 Manager；统一关闭（停流 + stop dispatcher） | `closed_` 原子；内部持有 dispatcher | `src/hand/l6/l6.cpp`、`src/hand/o6/o6.cpp` |
| `CANMessageDispatcher` | SocketCAN 收发与回调分发 | `socket_fd_`；`recv_thread_` | `src/can_dispatcher.cpp` |
| `*Manager` | 协议处理、最新值缓存、阻塞等待、流式采集 | `subscription_id`；streaming 线程 | `src/hand/**/**_manager.cpp` |
| `IterableQueue<T>` | 线程安全可迭代队列（内部共享状态） | `mutex/cv`；`closed` 标记 | `include/linkerhand/iterable_queue.hpp` |

### 1.2 线程/执行上下文
- `CANMessageDispatcher::recv_thread_`：poll/read CAN socket，然后**顺序执行**所有订阅回调。
- `*Manager::streaming_thread`：周期性 `dispatcher.send(...)`（拉取传感器/状态）。
- `ForceSensorManager`：额外存在 `aggregation_thread` 与若干 `reader_threads`（读取单指队列并聚合输出）。

## 2. 生命周期契约（close/stop/析构）

### 2.1 统一原则（必须对调用方明确）
- **析构与并发调用不兼容**：对象析构期间禁止与任何公开方法并发（包含阻塞 API）。调用方必须先停止自身线程并等待所有 in-flight 调用完成。
- **关闭函数幂等且可并发**：`close()/stop()/stop_streaming()` 允许多线程重复调用，语义等价于“只执行一次”。
- **关闭顺序**：先停 streaming，再停 dispatcher（避免后台线程继续 send 或回调触达已析构状态）。

### 2.2 `L6/O6::close()` 与析构

**关系与顺序**：
- `~L6/~O6`：best-effort 调用 `close()`（吞掉异常）。
- `close()`：内部顺序为：
  1) 调用各 streaming Manager 的 `stop_streaming()`  
  2) 调用 `dispatcher_.stop()`

**幂等与并发**：
- 使用 `closed_.exchange(true, std::memory_order_acq_rel)` 做幂等保护与并发安全。

**close 后统一行为（对外契约）**：
- `is_closed()` 返回 `true`。
- 由于 Manager 为公有成员暴露，close 后仍可能调用 `hand.angle.*`：
  - **涉及 CAN IO 的操作**（`set_*`、`*_blocking`、`stream`）将因 dispatcher stop 而抛 `StateError("CAN dispatcher is stopped")`（或非 Linux 场景抛 `CANError`）。
  - **仅访问缓存的操作**（`get_current_*`）仍可调用，返回 `std::optional`（可能包含最后一次接收值）。

### 2.3 `CANMessageDispatcher::stop()` 与析构

**关系**：
- `~CANMessageDispatcher`：best-effort 调用 `stop()`（吞掉异常）。

**幂等与并发**：
- `stop()` 使用 `running_.exchange(false, ...)` 幂等化；首次 stop 会 join `recv_thread_` 并关闭 `socket_fd_`。

**stop 后统一行为**：
- `send()`：在 stop 后抛 `StateError("CAN dispatcher is stopped")`，并在持锁后再次检查以避免竞态窗口。
- `subscribe/unsubscribe()`：可并发调用（互斥保护订阅列表）。

**重要约束**：
- **禁止在 `recv_thread_`（回调线程）内调用 `stop()`**：可能触发自 join（实现层面可能抛 `std::system_error`）。

### 2.4 `*Manager::stop_streaming()` / `stream()` 与析构

**析构关系（Impl 级别）**：
- `Impl::~Impl()` 执行（best-effort）：
  1) `stop_streaming()`（若支持）  
  2) `dispatcher.unsubscribe(subscription_id)`（解除回调关联）

**stream/stop_streaming 契约（适用于所有 streaming Manager）**：
- `stream(interval_ms, maxsize)`：
  - 参数校验失败：抛 `ValidationError`。
  - dispatcher 已 stop：抛 `StateError("CAN dispatcher is stopped")`（快速失败，避免返回“永不产出且永不关闭”的队列）。
  - 重复启动 streaming：抛 `StateError("Streaming is already active...")`。
  - 健壮性：streaming 线程退出时会 **关闭返回的 `IterableQueue`**，避免消费者永久阻塞（即使后台线程因 send/异常提前退出）。
- `stop_streaming()`：
  - 幂等：未开启时 no-op。
  - 关闭队列并 join 线程：保证 stop_streaming 返回后不会遗留后台线程。

**阻塞等待（`get_*_blocking(timeout_ms)`）契约**：
- `timeout_ms <= 0`：抛 `ValidationError`。
- 如果内部需要发送请求且 send 失败（例如 dispatcher stop），会在抛出异常前清理 waiter 注册，避免内部状态残留。
- close/stop 不会主动取消正在等待的阻塞调用：调用将按 timeout 返回（`TimeoutError`）或正常返回。

## 3. 并发/回调语义

### 3.1 回调执行线程与阻塞约束
- **回调线程**：所有订阅回调均在 `CANMessageDispatcher::recv_thread_` 中执行，单线程顺序执行。
- **是否允许阻塞**：不建议。阻塞会延迟后续消息处理与其他回调，扩大 stop/析构等待时间。
- **异常策略**：回调异常被捕获并写入 stderr，不向上抛出，不中断其他订阅者。

### 3.2 `subscribe/unsubscribe` 的并发策略（避免析构回调冲突）

**问题背景**：快照分发使得 `unsubscribe()` 返回时，回调可能仍在执行；若此时析构回调目标对象会导致 UAF。

**解决方案**：`SubscriberState(active + in_flight)` + 快照分发：
- 分发侧：在执行回调前 `try_enter()`（检查 active 并 `in_flight++`）；回调结束后 `exit()`（`in_flight--`）。
- `unsubscribe(id)`：
  - 先从订阅列表移除，再 `deactivate()`。
  - 若调用线程不是 `recv_thread_`：`wait_for_idle()` 等待 `in_flight==0` 后返回（保证析构安全）。
  - 若在 `recv_thread_` 调用：不等待（避免自锁/死锁），但**禁止在该回调栈内触发目标对象析构**，应转交到业务线程做 teardown。

### 3.3 stop 时如何避免竞态与析构回调冲突
- `stop()`：`running_=false` → join `recv_thread_` → 关闭 `socket_fd_`。因此 stop 返回后不再有回调执行。
- `Manager::Impl` 析构：先停 streaming，再 unsubscribe 等待回调退出（非回调线程调用时），避免析构期 UAF。

## 4. 边界情况（行为定义 + 调用方责任）

| 场景 | 期望行为（统一契约） | 备注/调用方责任 |
|------|----------------------|------------------|
| 多线程并发 `L6/O6::close()` | 幂等，仅一次实际清理 | 无 |
| `close()` 与业务 API 并发 | 业务 API 可能成功或抛 `StateError`（竞态时序决定） | 建议上层加锁串行化“关闭”和“业务调用” |
| `close()` 与 `get_*_blocking()` 并发 | 不会被主动取消；按 timeout 返回或抛 `StateError` | 若需“快速停机”，建议引入 cancel 或缩短 timeout |
| `stream()` 在 dispatcher stop 后调用 | 直接抛 `StateError("CAN dispatcher is stopped")` | 不要在 close 后启动 streaming |
| 回调中调用 `unsubscribe()` | 允许但不等待回调退出 | 回调内禁止析构目标对象；转交业务线程 |
| 回调中调用 `stop()` | 不支持 | 可能自 join；必须在非回调线程 stop |
| `stop()` 与 `send()` 并发 | `send()` 可能成功或抛 `StateError` | 强一致需要上层同步 |
| 消费者主动 `queue.close()` | 生产侧 put 抛 `StateError`；回调侧应吞掉并停止推送 | 关闭队列 ≠ 停止 streaming；需显式 `stop_streaming()` |
| 析构与任意 API 并发 | 未定义行为（潜在 UAF） | 必须保证析构前无 in-flight 调用（join/同步） |

## 5. 统一错误模型（建议作为对外契约）
- `ValidationError`：参数错误（interval/timeout/maxsize 等）。
- `StateError`：状态错误（dispatcher stop、重复 stream、对关闭队列 put 等）。
- `TimeoutError`：阻塞等待超时。
- `QueueFull/QueueEmpty/StopIteration`：队列语义异常或迭代终止。

## 6. 架构评审建议（可选改进项）
- **Facade 状态一致性**：`L6/O6::ensure_open()` 无法覆盖公有 Manager API；如需 close 后统一错误信息与更早失败，可考虑：
  - 将 Manager 变为私有成员并由 Facade 提供包装方法；或
  - Manager 持有 Facade 的共享生命周期状态（例如 `shared_ptr<LifecycleState>`）统一检查。
- **阻塞 API 的可取消性**：当前 close 不会唤醒 `get_*_blocking()` 的等待线程（依赖 timeout）；若业务需要“快速停止”，建议增加 cancel/broadcast 机制。
