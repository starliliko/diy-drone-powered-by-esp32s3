# ESP32 同步机制详解 (Semaphore, Mutex, Spinlock, 中断控制)


---

## 目录

1. [概述与对比](#概述与对比)
2. [Semaphore (信号量)](#semaphore-信号量)
3. [Mutex (互斥锁)](#mutex-互斥锁)
4. [Spinlock (自旋锁)](#spinlock-自旋锁)
5. [中断控制](#中断控制)
6. [核心概念深入](#核心概念深入)
   - [忙等待详解](#忙等待详解)
   - [临界区详解](#临界区详解)
   - [中断禁用机制](#中断禁用机制)
   - [单核与双核对比](#单核与双核对比)
7. [实际应用场景](#实际应用场景)
8. [常见错误与最佳实践](#常见错误与最佳实践)
9. [组会讲解大纲](#组会讲解大纲)

---

## 概述与对比

### 为什么需要同步机制？

在多任务并发系统中，多个任务可能同时访问共享资源（如变量、硬件外设），导致**竞态条件 (Race Condition)**。

**示例：竞态条件**
```c
// 全局变量
volatile int counter = 0;

// Task A
void taskA() {
  counter++;  // 读取 → 加1 → 写回 (3个CPU指令)
}

// Task B
void taskB() {
  counter++;  // 可能在 Task A 执行中间插入！
}

// 结果：counter 可能是 1 而不是期望的 2
```

### 四种机制对比表

| 机制          | 阻塞 | ISR安全     | 优先级继承 | CPU消耗 | 最佳场景             |
| ------------- | ---- | ----------- | ---------- | ------- | -------------------- |
| **Semaphore** | ✓    | ✓ (FromISR) | ❌          | 低      | 事件通知、资源计数   |
| **Mutex**     | ✓    | ❌           | ✓          | 低      | 互斥访问、长临界区   |
| **Spinlock**  | ❌    | ✓           | ❌          | 高      | 极短临界区、共享变量 |
| **中断控制**  | ❌    | ✓           | ❌          | 中等    | 硬件寄存器访问       |

---

## Semaphore (信号量)

### 原理

信号量是一个**计数器**，用于控制对共享资源的访问或进行任务同步。

```
┌─────────────────────────────────────────────────────────────────────┐
│                        信号量工作原理                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   初始值: [3]  (可用资源数)                                          │
│                                                                      │
│   Task1: xSemaphoreTake()  →  [2]  ✓ 获取成功                       │
│   Task2: xSemaphoreTake()  →  [1]  ✓ 获取成功                       │
│   Task3: xSemaphoreTake()  →  [0]  ✓ 获取成功                       │
│   Task4: xSemaphoreTake()  →  计数=0，阻塞等待 ❌                     │
│                                                                      │
│   Task1: xSemaphoreGive()  →  [1]  → Task4 被唤醒                   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 类型

| 类型                   | 计数范围        | 用途               |
| ---------------------- | --------------- | ------------------ |
| **Binary Semaphore**   | 0 或 1          | 事件通知、任务同步 |
| **Counting Semaphore** | 0 到 N          | 资源池管理         |
| **Mutex**              | 0 或 1 + 所有权 | 互斥访问           |

### API

```c
// 创建
SemaphoreHandle_t sem = xSemaphoreCreateBinary();
SemaphoreHandle_t cnt_sem = xSemaphoreCreateCounting(max, initial);

// 静态创建
StaticSemaphore_t semBuffer;
SemaphoreHandle_t sem = xSemaphoreCreateBinaryStatic(&semBuffer);

// 获取 (阻塞)
xSemaphoreTake(sem, portMAX_DELAY);

// 释放
xSemaphoreGive(sem);

// ISR 中使用
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
```

### ESP-Drone 应用：启动同步

**system.c - systemStart 机制**

```c
// 全局信号量
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

// 初始化时创建并立即获取
void systemInit(void) {
  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);  // 阻止系统启动
  // ... 其他初始化 ...
}

// 其他任务等待启动
void systemWaitStart(void) {
  while (!isInit) vTaskDelay(2);                 // 等待systemInit完成
  xSemaphoreTake(canStartMutex, portMAX_DELAY);  // 阻塞等待
  xSemaphoreGive(canStartMutex);                 // 立即释放
}

// 所有初始化完成后释放
void systemStart() {
  xSemaphoreGive(canStartMutex);  // 唤醒所有等待任务
}
```

**时序图**:
```
systemTask          stabilizerTask        extRxTask         pmTask
    │                    │                    │                │
    │ 初始化中...         │ systemWaitStart()  │                │
    │                    ├─ Take (阻塞) ───┐  │                │
    │                    │                 │  ├─ systemWaitStart()
    │                    │                 │  ├─ Take (阻塞) ─┐ │
    │                    │                 │  │               │ ├─ systemWaitStart()
    │                    │                 │  │               │ ├─ Take (阻塞) ─┐
    ▼                    ▼                 │  ▼               │ ▼              │
systemStart()           [等待]             │ [等待]           │ [等待]          │
xSemaphoreGive() ───────┴──────────────────┴──────────────────┴────────────────┘
    │                    │                    │                │
    │                  唤醒                  唤醒             唤醒
    ▼                    ▼                    ▼                ▼
 workerLoop()        开始工作              开始工作         开始工作
```


## Mutex (互斥锁)

### 原理

Mutex 是特殊的 Binary Semaphore，专门用于**互斥访问共享资源**，具有**所有权**和**优先级继承**特性。

```
┌─────────────────────────────────────────────────────────────────────┐
│                      Mutex vs Semaphore                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   Semaphore (信号量):                                               │
│   • 任何任务都可以 Give (无所有权)                                   │
│   • 用于"信号通知"                                                   │
│   • 无优先级继承                                                     │
│                                                                      │
│   Mutex (互斥锁):                                                    │
│   • 只有持有者可以释放 (有所有权)                                     │
│   • 用于"互斥访问"                                                   │
│   • 支持优先级继承，防止优先级反转                                    │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 优先级继承 (Priority Inheritance)

**问题：优先级反转**

```
时间轴 ────────────────────────────────────────────────────→

Task_L (低优先级):
  │
  ├─ 获取 Mutex ─────┬─ 持有 Mutex ──────────────┬─ 释放
  │                  │                           │
Task_M (中优先级):   │                           │
                     │                           │
                  抢占 Task_L ───────────────────┘
                     │
Task_H (高优先级):   │
                     │
              等待 Mutex (被 Task_M 间接阻塞！) ❌ 优先级反转
```

**Mutex 的解决方案**：

```
Task_L 持有 Mutex
    ↓
Task_H 尝试获取 Mutex
    ↓
Task_L 临时提升到 Task_H 的优先级 (继承)
    ↓
Task_L 不会被 Task_M 抢占
    ↓
Task_L 释放 Mutex，优先级恢复
    ↓
Task_H 获取 Mutex ✓
```

### API

```c
// 创建
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

// 静态创建
StaticSemaphore_t mutexBuffer;
SemaphoreHandle_t mutex = xSemaphoreCreateMutexStatic(&mutexBuffer);

// 获取 (阻塞)
xSemaphoreTake(mutex, portMAX_DELAY);

// 释放 (只有持有者可以释放)
xSemaphoreGive(mutex);

// ⚠️ 不能在 ISR 中使用！
```

### 应用场景：I2C 总线互斥访问

```c
// I2C 总线互斥锁
SemaphoreHandle_t i2c_mutex;

void i2c_init() {
  i2c_mutex = xSemaphoreCreateMutex();
}

// 传感器1读取
void read_mpu6050() {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);  // 获取总线控制权
  i2c_master_write_read_device(I2C_NUM_0, MPU6050_ADDR, ...);
  xSemaphoreGive(i2c_mutex);                 // 释放总线
}

// 传感器2读取（不会冲突）
void read_ms5611() {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  i2c_master_write_read_device(I2C_NUM_0, MS5611_ADDR, ...);
  xSemaphoreGive(i2c_mutex);
}

// 传感器3读取
void read_hmc5883l() {
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  i2c_master_write_read_device(I2C_NUM_0, HMC5883L_ADDR, ...);
  xSemaphoreGive(i2c_mutex);
}
```

**时序图**:
```
sensorsTask              i2c_mutex              I2C 总线
    │                        │                      │
    ├─ read_mpu6050()        │                      │
    ├─ Take(mutex) ──────────┤                      │
    │                      [获取]                    │
    ├─ i2c_read() ───────────────────────────────→  │
    │                        │                    读取中
    ├─ Give(mutex) ──────────┤                      │
    │                      [释放]                    │
    │                        │                      │
    ├─ read_ms5611()         │                      │
    ├─ Take(mutex) ──────────┤                      │
    │                      [获取]                    │
    ├─ i2c_read() ───────────────────────────────→  │
    │                        │                    读取中
    ├─ Give(mutex) ──────────┤                      │
```

---

## Spinlock (自旋锁)

### 原理

Spinlock 是一种**忙等待**的锁，获取失败时不会进入阻塞状态，而是在原地循环检查（自旋）。

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Mutex vs Spinlock                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   Mutex (互斥锁):                                                    │
│   ┌──────┐                                                          │
│   │Task  │ → xSemaphoreTake() → 锁被占用 → 进入阻塞 → 让出CPU       │
│   └──────┘                                                          │
│                                                                      │
│   Spinlock (自旋锁):                                                 │
│   ┌──────┐                                                          │
│   │Core0 │ → portENTER_CRITICAL() → 锁被占用 → 原地等待(循环)        │
│   └──────┘       占用CPU，持续检查锁状态                             │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### ESP32 实现：临界区 (Critical Section)

ESP32 使用临界区实现 Spinlock，并**禁用中断**。

```c
// 定义 spinlock
portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

// 方式1: 禁用当前核心的可屏蔽中断
portENTER_CRITICAL(&my_spinlock);
// 临界区代码 (极短！)
portEXIT_CRITICAL(&my_spinlock);

// 方式2: 在 ISR 中使用 (禁用所有中断)
portENTER_CRITICAL_ISR(&my_spinlock);
// 临界区代码
portEXIT_CRITICAL_ISR(&my_spinlock);
```

### 双核场景

```
CPU0 (PRO_CPU)                CPU1 (APP_CPU)
    │                             │
    ├─ portENTER_CRITICAL()       │
    │  获取 spinlock               │
    │                             ├─ portENTER_CRITICAL()
    │                             │  尝试获取 spinlock
    │  [临界区代码]               │  ↓
    │  counter++;                 │  [自旋等待]
    │                             │  while(spinlock) {
    │                             │    // CPU 空转，持续检查
    │                             │  }
    ├─ portEXIT_CRITICAL()        │
    │  释放 spinlock               │  ↓
    │                             │  获取成功！
    │                             ├─ [临界区代码]
    │                             │  counter++;
    │                             ├─ portEXIT_CRITICAL()
```

### 应用场景：共享变量保护

```c
volatile uint32_t shared_counter = 0;
portMUX_TYPE counter_mux = portMUX_INITIALIZER_UNLOCKED;

// 任务中访问
void task_increment() {
  portENTER_CRITICAL(&counter_mux);
  shared_counter++;  // 原子操作，几个CPU周期
  portEXIT_CRITICAL(&counter_mux);
}

// ISR 中访问
void IRAM_ATTR gpio_isr_handler() {
  portENTER_CRITICAL_ISR(&counter_mux);
  shared_counter++;
  portEXIT_CRITICAL_ISR(&counter_mux);
}
```

### 使用场景对比

| 情况       | 使用 Mutex        | 使用 Spinlock        |
| ---------- | ----------------- | -------------------- |
| 临界区时间 | 较长 (>几十μs)    | 极短 (<几μs)         |
| 是否阻塞   | 可以阻塞          | 不能阻塞             |
| CPU消耗    | 低 (让出CPU)      | 高 (持续检查)        |
| 中断上下文 | ❌ 不可用          | ✓ 可用               |
| 任务切换   | 允许              | 禁止                 |
| 典型应用   | I2C总线、文件系统 | 共享变量、硬件寄存器 |

---

## 中断控制

### ESP32 中断层级

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ESP32 中断优先级                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   Level 7 (最高) ─── NMI (不可屏蔽中断)                              │
│   Level 6       ─── 高优先级中断                                     │
│   Level 5       ─── 调试异常                                         │
│   Level 4-3     ─── 中等优先级中断                                   │
│   Level 2-1     ─── 低优先级中断 (FreeRTOS 可屏蔽，Max < 3)          │
│   Level 0       ─── 普通代码执行                                     │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 开关中断的几种方式

#### 方式1: 全局禁用中断 (危险！)

```c
// 禁用所有中断
portDISABLE_INTERRUPTS();
// 临界区代码 (必须极短)
portENABLE_INTERRUPTS();
```

**影响**：所有中断被屏蔽，包括系统时钟，可能导致系统崩溃。

---

#### 方式2: 临界区 (推荐)

```c
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// 任务中使用 - 禁用低优先级中断 (< Level 3)
portENTER_CRITICAL(&mux);
// 临界区代码
portEXIT_CRITICAL(&mux);

// ISR 中使用 - 禁用所有可屏蔽中断
portENTER_CRITICAL_ISR(&mux);
// 临界区代码
portEXIT_CRITICAL_ISR(&mux);
```

**特性**：
- 支持嵌套（计数）
- 双核安全
- 禁用任务调度

---

#### 方式3: 挂起调度器

```c
// 挂起任务调度器（中断仍然运行）
vTaskSuspendAll();
// 不会被其他任务抢占，但中断仍然响应
xTaskResumeAll();
```

**用途**：长时间临界区，但不希望禁用中断。

---

### 对比表

| 方法                       | 禁用中断 | 禁用调度 | 支持嵌套 | ISR安全 | 用途         |
| -------------------------- | -------- | -------- | -------- | ------- | ------------ |
| `portDISABLE_INTERRUPTS()` | ✓ 全部   | ✓        | ❌        | ✓       | 极短临界区   |
| `portENTER_CRITICAL()`     | ✓ 部分   | ✓        | ✓        | ❌       | 任务临界区   |
| `portENTER_CRITICAL_ISR()` | ✓ 全部   | -        | ✓        | ✓       | ISR临界区    |
| `vTaskSuspendAll()`        | ❌        | ✓        | ✓        | ❌       | 长时间临界区 |

---

## 核心概念深入

### 忙等待详解

#### 定义

**忙等待 (Busy Waiting)** 是指进程/线程在等待某个条件满足时，**不断循环检查条件**，而不是让出 CPU 进入休眠/阻塞状态。

#### 代码对比

**忙等待 (Spinlock)**

```c
volatile bool lock = false;

// 获取锁
void acquire_lock() {
  while (lock == true) {
    // 什么都不做，只是循环检查
    // CPU 一直在这里执行，浪费 CPU 周期
  }
  lock = true;  // 获取到锁
}

// 汇编层面
loop:
  LOAD  R1, [lock]      ; 读取 lock 的值
  CMP   R1, #1          ; 比较是否为 1
  BEQ   loop            ; 如果是 1，跳回 loop 继续检查
  MOV   R1, #1          ; 设置为 1
  STORE R1, [lock]      ; 写入
```

**特点**:
- CPU 持续执行指令（循环）
- **不让出 CPU**
- **高 CPU 占用率**

**阻塞等待 (Mutex/Semaphore)**

```c
SemaphoreHandle_t mutex;

// 获取锁
void acquire_lock() {
  xSemaphoreTake(mutex, portMAX_DELAY);  // 阻塞
  // 如果获取不到，任务进入阻塞队列，CPU 执行其他任务
}

// 系统调用流程
xSemaphoreTake() 
  → 检查信号量
  → 如果不可用：
      - 将当前任务加入等待队列
      - 切换到其他任务执行  ← CPU 去执行其他任务
      - 当前任务进入休眠
  → 信号量可用时，调度器唤醒任务
```

**特点**:
- 任务进入阻塞/休眠
- **让出 CPU 给其他任务**
- **低 CPU 占用率**

#### 双核场景对比

```
═══════════════════════════════════════════════════════════════════
Spinlock (忙等待)
═══════════════════════════════════════════════════════════════════

CPU0                          CPU1
 │                             │
 ├─ portENTER_CRITICAL()       │
 │  lock = 1                   │
 │                             ├─ portENTER_CRITICAL()
 │  [临界区代码]               │  while(lock == 1) {
 │  counter++;                 │    // 空循环，CPU 空转 ⚠️
 │                             │  }
 │                             │  ← CPU1 100% 占用，但什么都不做！
 │                             │
 ├─ portEXIT_CRITICAL()        │
 │  lock = 0                   │
 │                             │  lock == 0，退出循环
 │                             ├─ [临界区代码]
 │                             │  counter++;

═══════════════════════════════════════════════════════════════════
Mutex (阻塞等待)
═══════════════════════════════════════════════════════════════════

CPU0                          CPU1
 │                             │
 ├─ xSemaphoreTake()           │
 │  获取 mutex                  │
 │                             ├─ xSemaphoreTake()
 │  [临界区代码]               │  mutex 不可用
 │  i2c_read() (5ms)           │  → 进入阻塞
 │                             │  → CPU1 切换到其他任务 ✓
 │                             │  → Task_B 开始执行
 │                             │  → CPU1 利用率高
 ├─ xSemaphoreGive()           │
 │  释放 mutex                  │
 │                             │  → 调度器唤醒等待任务
 │                             ├─ [临界区代码]
```


#### 为什么还要用忙等待？

虽然浪费 CPU，但在某些场景下**忙等待更快**：

**优势场景：极短临界区**

```
假设临界区只需要 1μs，任务切换需要 10μs

┌─────────────────────────────────────────────────────────────┐
│ Spinlock (忙等待)                                           │
├─────────────────────────────────────────────────────────────┤
│ 等待 1μs → 获取锁 → 执行 → 总耗时 1μs                        │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Mutex (阻塞)                                                │
├─────────────────────────────────────────────────────────────┤
│ 阻塞切换 (10μs) → 等待 → 唤醒切换 (10μs) → 总耗时 20μs+     │
└─────────────────────────────────────────────────────────────┘

结论: 临界区极短时，忙等待反而更快！
```

#### 上下文切换开销详解

**上下文切换 (Context Switch)** 是任务调度的核心开销，包括：

```
┌─────────────────────────────────────────────────────────────┐
│            上下文切换的时间消耗                              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. 保存当前任务上下文 (Save Context)                       │
│     ├─ CPU 寄存器 (R0-R15, PC, SP, LR)                      │
│     ├─ 浮点寄存器 (如果使用 FPU)                            │
│     └─ 任务状态 (TCB 更新)                                  │
│                                                             │
│  2. 调度器决策 (Scheduler Decision)                         │
│     ├─ 遍历就绪队列                                         │
│     ├─ 优先级比较                                           │
│     └─ 选择下一个任务                                       │
│                                                             │
│  3. 恢复目标任务上下文 (Restore Context)                    │
│     ├─ 从 TCB 加载寄存器值                                  │
│     ├─ 恢复堆栈指针                                         │
│     └─ 跳转到目标任务                                       │
│                                                             │
│  4. 缓存/管道影响 (Cache/Pipeline Effects)                  │
│     ├─ 指令缓存失效                                         │
│     ├─ 数据缓存失效                                         │
│     └─ CPU 流水线刷新                                       │
│                                                             │
│  总耗时: ESP32 上约 10-20μs                                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**相关术语**：

| 术语                                     | 英文                          | 说明                   |
| ---------------------------------------- | ----------------------------- | ---------------------- |
| **上下文切换**                           | Context Switch                | 从一个任务切换到另一个 |
| **调度开销**                             | Scheduling Overhead           | 调度器决策和执行的时间 |
| **上下文切换时间/延迟**                  | Context Switch Time / Latency | 完成一次切换所需的时间 |
| **任务控制块**                           | Task Control Block (TCB)      | 存储任务状态的数据结构 |
| **就绪队列**                             | Ready Queue                   | 等待执行的任务队列     |
| **缓存失效** / **流水线刷新** (性能影响) | Cache Miss / Pipeline Flush   | 切换导致的间接性能损失 |

**为什么要考虑上下文切换开销？**

```c
// ❌ 错误示例：频繁切换导致浪费
void busy_task() {
  while(1) {
    do_tiny_work();      // 只需 1μs
    vTaskDelay(1);       // 切换开销 15μs
    // 实际工作 1μs，切换浪费 15μs
    // 效率只有 1/(1+15) = 6.25%
  }
}

// ✓ 正确示例：批量处理减少切换
void efficient_task() {
  while(1) {
    for(int i=0; i<100; i++) {
      do_tiny_work();    // 100μs
    }
    vTaskDelay(10);      // 切换开销 15μs
    // 效率 100/(100+15) = 87%
  }
}
```

#### 总结

| 特性           | 忙等待 (Spinlock)  | 阻塞等待 (Mutex)   |
| -------------- | ------------------ | ------------------ |
| **CPU 占用**   | 持续 100%          | 0% (休眠)          |
| **等待方式**   | 循环检查           | 进入休眠队列       |
| **上下文切换** | 无                 | 有 (10-20μs)       |
| **调度开销**   | 无                 | 有 (2-5μs)         |
| **适用场景**   | 极短临界区 (<10μs) | 长临界区 (>10μs)   |
| **多核影响**   | 一个核心空转       | 核心可执行其他任务 |
| **优势**       | 快速、无切换开销   | 节省 CPU           |
| **劣势**       | 浪费 CPU           | 切换开销           |
| **何时更高效** | 临界区 < 切换时间  | 临界区 > 切换时间  |


### 临界区详解

#### 定义

**临界区 (Critical Section)** 是指访问**共享资源**的一段代码，在同一时刻**只能有一个线程/任务**执行，否则会导致数据不一致。

#### 为什么需要临界区？

**问题：竞态条件 (Race Condition)**

```c
// 共享变量
int balance = 1000;

// 线程A: 取款 500
void withdraw() {
  int temp = balance;     // 1. 读取 balance = 1000
  temp = temp - 500;      // 2. 计算 temp = 500
  balance = temp;         // 3. 写回 balance = 500
}

// 线程B: 取款 300
void withdraw_b() {
  int temp = balance;     // 1. 读取 balance = 1000
  temp = temp - 300;      // 2. 计算 temp = 700
  balance = temp;         // 3. 写回 balance = 700
}
```

**时序问题**：
```
时间轴 ──────────────────────────────────────────────→

线程A:  读(1000)  计算(500)           写(500)
线程B:          读(1000)  计算(700)  写(700)
                  ↑
                交叉执行！

结果: balance = 700 (应该是 200)
问题: 线程B的读取发生在线程A写回之前，覆盖了线程A的结果
```

#### 临界区保护

```c
int balance = 1000;
SemaphoreHandle_t mutex;

// 线程A: 取款 500
void withdraw() {
  // ─────── 临界区开始 ───────
  xSemaphoreTake(mutex, portMAX_DELAY);  // 获取锁
  
  int temp = balance;     // 读取
  temp = temp - 500;      // 计算
  balance = temp;         // 写回
  
  xSemaphoreGive(mutex);                 // 释放锁
  // ─────── 临界区结束 ───────
}

// 线程B: 取款 300
void withdraw_b() {
  xSemaphoreTake(mutex, portMAX_DELAY);  // 等待线程A释放
  
  int temp = balance;
  temp = temp - 300;
  balance = temp;
  
  xSemaphoreGive(mutex);
}
```

**正确时序**：
```
时间轴 ──────────────────────────────────────────────→

线程A:  [获取锁] 读 计算 写 [释放锁]
线程B:          [等待]──────────→[获取锁] 读 计算 写 [释放锁]

结果: balance = 200 ✓
```

#### 临界区的组成部分

```c
┌─────────────────────────────────────────────────────────────┐
│                     临界区结构                               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. 进入区 (Entry Section)                                  │
│     ├─ 检查是否可以进入                                     │
│     └─ 如果不能，等待                                       │
│                                                              │
│  2. 临界区 (Critical Section)                               │
│     └─ 访问共享资源的代码                                   │
│                                                              │
│  3. 退出区 (Exit Section)                                   │
│     └─ 释放临界区，通知其他线程                             │
│                                                              │
│  4. 剩余区 (Remainder Section)                              │
│     └─ 其他不访问共享资源的代码                             │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

#### ESP32 中的临界区实现

**方式1: Mutex (长临界区)**

```c
SemaphoreHandle_t mutex;

void critical_section_mutex() {
  // ── 进入区 ──
  xSemaphoreTake(mutex, portMAX_DELAY);
  
  // ── 临界区 ──
  process_i2c_data();  // 几毫秒
  
  // ── 退出区 ──
  xSemaphoreGive(mutex);
}
```

**方式2: Spinlock (极短临界区)**

```c
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void critical_section_spinlock() {
  // ── 进入区 ──
  portENTER_CRITICAL(&mux);  // 禁用中断 + 获取自旋锁
  
  // ── 临界区 ──
  shared_counter++;  // 几个CPU周期
  
  // ── 退出区 ──
  portEXIT_CRITICAL(&mux);   // 释放锁 + 恢复中断
}
```

#### 临界区的三个要求

```
┌─────────────────────────────────────────────────────────────┐
│              临界区必须满足的三个条件                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. 互斥 (Mutual Exclusion)                                 │
│     同一时刻只有一个线程在临界区内                           │
│                                                              │
│  2. 进步 (Progress)                                         │
│     如果没有线程在临界区，且有线程想进入，                   │
│     则必须能在有限时间内进入                                 │
│                                                              │
│  3. 有限等待 (Bounded Waiting)                              │
│     线程请求进入临界区后，在有限时间内能够进入               │
│     (防止饥饿)                                              │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

#### 临界区设计原则

**1. 尽量短**

```c
// ❌ 错误: 临界区太长
portENTER_CRITICAL(&mux);
process_data();      // 耗时操作
send_to_network();   // 耗时操作
update_counter++;
portEXIT_CRITICAL(&mux);

// ✓ 正确: 只保护必要的部分
process_data();      // 不需要保护
send_to_network();   // 不需要保护

portENTER_CRITICAL(&mux);
update_counter++;    // 只保护共享变量
portEXIT_CRITICAL(&mux);
```

**2. 避免嵌套**

```c
// ❌ 错误: 嵌套临界区（可能死锁）
portENTER_CRITICAL(&mux_a);
  portENTER_CRITICAL(&mux_b);
    // 临界区
  portEXIT_CRITICAL(&mux_b);
portEXIT_CRITICAL(&mux_a);

// ✓ 正确: 合并或重新设计
portENTER_CRITICAL(&mux_combined);
  // 临界区
portEXIT_CRITICAL(&mux_combined);
```

**3. 不要在临界区内调用阻塞函数**

```c
// ❌ 错误
portENTER_CRITICAL(&mux);
vTaskDelay(100);        // 阻塞！其他核心会一直等待
shared_data++;
portEXIT_CRITICAL(&mux);

// ✓ 正确
portENTER_CRITICAL(&mux);
shared_data++;
portEXIT_CRITICAL(&mux);
vTaskDelay(100);        // 在临界区外阻塞
```

---

### 中断禁用机制

#### FreeRTOS 临界区不会禁用所有中断

```c
portENTER_CRITICAL(&mux);
// 临界区代码
portEXIT_CRITICAL(&mux);
```

**实际行为**：
- ❌ **不是**禁用所有中断
- ✓ **只禁用**优先级 ≤ `configMAX_SYSCALL_INTERRUPT_PRIORITY` 的中断
- ✓ 高优先级中断（NMI、Level 6-7）**仍然可以响应**

#### ESP32 中断优先级分层

```
┌─────────────────────────────────────────────────────────────────────┐
│                ESP32 中断优先级与临界区                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│   Level 7 (NMI)        ───  不可屏蔽中断 (永远不会被禁用)            │
│   Level 6-4            ───  高优先级中断 (临界区不影响) ✓            │
│   ═══════════════════════════════════════════════════════            │
│   Level 3              ───  configMAX_SYSCALL_INTERRUPT_PRIORITY     │
│   ═══════════════════════════════════════════════════════            │
│   Level 2-1            ───  低优先级中断 (临界区会禁用) ✗            │
│   Level 0              ───  普通代码                                 │
│                                                                      │
│   portENTER_CRITICAL() 只禁用 Level 1-3 的中断                       │
│   Level 4-7 的中断仍然可以触发                                      │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```


#### 对比：不同函数的中断禁用范围

```c
// 1. portENTER_CRITICAL() - 禁用部分中断
portENTER_CRITICAL(&mux);
// 禁用: Level 1-3
// 保留: Level 4-7 (高优先级中断仍可响应)
portEXIT_CRITICAL(&mux);

// 2. portENTER_CRITICAL_ISR() - 禁用更多中断
portENTER_CRITICAL_ISR(&mux);
// 禁用: Level 1-6
// 保留: Level 7 (NMI)
portEXIT_CRITICAL_ISR(&mux);

// 3. portDISABLE_INTERRUPTS() - 尝试禁用所有（但NMI除外）
portDISABLE_INTERRUPTS();
// 禁用: Level 1-6
// 保留: Level 7 (NMI 不可屏蔽)
portENABLE_INTERRUPTS();
```

#### 为什么不禁用所有中断？

```
如果禁用所有中断：
    ❌ 看门狗定时器无法复位 → 系统重启
    ❌ NMI 用于严重错误处理 → 无法处理
    ❌ 调试器无法工作
    ❌ 系统时钟可能受影响

保留高优先级中断：
    ✓ 关键系统功能可以继续
    ✓ 紧急情况可以处理
    ✓ 只保护 FreeRTOS API 调用
```

#### 配置选项

```c
// FreeRTOS 配置 (ESP32)
// 文件: sdkconfig 或 FreeRTOSConfig.h

#define configMAX_SYSCALL_INTERRUPT_PRIORITY  3

// 含义：
// - 优先级 > 3 的中断不会被 FreeRTOS 管理
// - 这些中断不能调用 FreeRTOS API (如 xSemaphoreGiveFromISR)
// - 但它们可以在临界区期间触发
```

#### 总结表

| 问题                                | 答案                                         |
| ----------------------------------- | -------------------------------------------- |
| `portENTER_CRITICAL` 禁用所有中断？ | ❌ 不是，只禁用低优先级中断                   |
| 哪些中断被禁用？                    | Level 1-3 (FreeRTOS 管理的中断)              |
| 哪些中断不受影响？                  | Level 4-7 (高优先级、NMI)                    |
| NMI 能被禁用吗？                    | ❌ 永远不能（不可屏蔽）                       |
| 为什么这样设计？                    | 保证关键系统功能（看门狗、错误处理）始终可用 |

---

### 单核与双核对比

#### 自旋锁与核心数的关系

**简短回答：不一定**。自旋锁在单核和双核/多核系统中都可以使用，但**意义和效果完全不同**。

#### 单核系统中的自旋锁

**行为**

```c
// 单核系统
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void task_A() {
  portENTER_CRITICAL(&mux);  // 获取锁 + 禁用中断/调度
  shared_data++;
  portEXIT_CRITICAL(&mux);
}

void task_B() {
  portENTER_CRITICAL(&mux);  // 永远不会自旋等待！
  shared_data++;
  portEXIT_CRITICAL(&mux);
}
```

**关键点**：
```
单核系统中：
  ├─ portENTER_CRITICAL() 禁用任务调度
  ├─ Task_A 在临界区时，Task_B 无法运行
  ├─ 所以 Task_B 永远不会"自旋等待"
  └─ "自旋"机制退化为"禁用调度"
```

**时序**：
```
时间轴 ──────────────────────────────────────→

单核CPU:
  │
  ├─ Task_A 运行
  ├─ portENTER_CRITICAL() → 禁用调度
  ├─ [临界区] shared_data++
  ├─ portEXIT_CRITICAL() → 恢复调度
  │
  ├─ 任务切换 → Task_B
  ├─ portENTER_CRITICAL() → 直接获取（无需等待）
  ├─ [临界区] shared_data++
  ├─ portEXIT_CRITICAL()
```

**结论**：单核系统中不会发生真正的"自旋"，因为同一时刻只有一个任务在运行。

#### 双核/多核系统中的自旋锁

**行为**

```c
// 双核系统
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Task_A 在 CPU0 上运行
void task_A() {
  portENTER_CRITICAL(&mux);  // CPU0 获取锁
  shared_data++;              // 执行中...
  portEXIT_CRITICAL(&mux);
}

// Task_B 在 CPU1 上运行
void task_B() {
  portENTER_CRITICAL(&mux);  // CPU1 自旋等待！
  shared_data++;
  portEXIT_CRITICAL(&mux);
}
```

**关键点**：
```
双核系统中：
  ├─ CPU0 执行 Task_A，持有锁
  ├─ CPU1 执行 Task_B，尝试获取锁
  ├─ CPU1 发现锁被占用
  ├─ CPU1 进入自旋循环（忙等待）
  └─ 真正的"自旋"发生在这里
```

**时序**：
```
时间轴 ──────────────────────────────────────→

CPU0:  │─ Task_A ────────────────────────┐
       │  portENTER_CRITICAL()           │
       │  [临界区] shared_data++         │
       │  portEXIT_CRITICAL() ───────────┘

CPU1:  │─ Task_B ─────────────────────────┐
       │  portENTER_CRITICAL()            │
       │  ↓ 锁被占用                      │
       │  [自旋等待] ← CPU1 空转          │
       │  while(lock) { /* 检查 */ }      │
       │  ↓ 锁释放                        │
       │  获取成功                        │
       │  [临界区] shared_data++          │
       │  portEXIT_CRITICAL() ────────────┘
```

**结论**：双核系统才会出现真正的自旋等待，一个核心空转检查锁状态。

#### 为什么单核也用 `portENTER_CRITICAL`？

```c
// ESP32 单核配置下
portENTER_CRITICAL(&mux);
// 临界区
portEXIT_CRITICAL(&mux);
```

**原因**：

1. **代码兼容性**
   - 同一套代码可以在单核/双核配置下编译
   - `portENTER_CRITICAL` 自动适配

2. **中断保护**
   - 单核系统中，主要作用是**禁用中断**
   - 防止中断处理函数访问共享数据

3. **FreeRTOS 实现**
   ```c
   #if configNUM_CORES == 1
     // 单核：只禁用中断，无自旋
     #define portENTER_CRITICAL(mux) taskENTER_CRITICAL()
   #else
     // 多核：禁用中断 + 自旋锁
     #define portENTER_CRITICAL(mux) vPortEnterCritical(mux)
   #endif
   ```

#### 单核系统的临界区保护

```c
volatile uint32_t counter = 0;

// 中断处理函数
void IRAM_ATTR timer_isr() {
  counter++;  // ← 可能与任务冲突
}

// 任务
void task() {
  // 需要保护，防止中断打断
  portENTER_CRITICAL(&mux);  // 禁用中断
  counter++;                  // 现在安全了
  portEXIT_CRITICAL(&mux);
}
```

**单核系统中的"临界区"主要防止**：
- ✓ 中断打断任务
- ✓ 任务切换（调度器禁用）
- ❌ 不存在"另一个核心同时访问"的问题

#### 对比表

| 特性                          | 单核系统            | 双核/多核系统       |
| ----------------------------- | ------------------- | ------------------- |
| **会发生自旋吗？**            | ❌ 不会              | ✓ 会                |
| **主要作用**                  | 禁用中断 + 禁用调度 | 禁用中断 + 自旋同步 |
| **CPU 浪费**                  | 无                  | 有（一个核心空转）  |
| **保护对象**                  | 中断 vs 任务        | 核心0 vs 核心1      |
| **`portENTER_CRITICAL` 实现** | 简化版（无自旋）    | 完整版（有自旋）    |

#### 历史背景：为什么叫"自旋锁"？

```
自旋锁起源于多处理器系统：
  ├─ 1960s: 多处理器计算机出现
  ├─ 需要同步机制保护共享内存
  ├─ 传统锁（阻塞）会导致上下文切换开销大
  ├─ 发明自旋锁：不阻塞，原地"自旋"等待
  └─ 适合临界区极短的场景

单核时代：
  ├─ 不存在真正的"并行"
  ├─ 自旋锁退化为"禁用中断/调度"
  └─ 但保留了名字和 API
```

#### ESP32 配置的影响

```c
// sdkconfig
CONFIG_FREERTOS_UNICORE=y  // 单核模式
// 或
CONFIG_FREERTOS_NUMBER_OF_CORES=2  // 双核模式
```

**单核配置**：
```c
// FreeRTOS/portable/xtensa/portmacro.h
void portENTER_CRITICAL(portMUX_TYPE *mux) {
  // 只禁用中断，无自旋
  portDISABLE_INTERRUPTS();
}
```

**双核配置**：
```c
void portENTER_CRITICAL(portMUX_TYPE *mux) {
  // 1. 禁用中断
  portDISABLE_INTERRUPTS();
  
  // 2. 自旋获取锁
  while (!compare_and_swap(&mux->owner, FREE, current_cpu)) {
    // 自旋等待
  }
}
```

#### 建议

| 系统配置         | 使用建议                                      |
| ---------------- | --------------------------------------------- |
| **单核 ESP32**   | 仍使用 `portENTER_CRITICAL`，主要用于禁用中断 |
| **双核 ESP32**   | 临界区极短用 `portENTER_CRITICAL`，长用 Mutex |
| **中断 vs 任务** | 单核也需要保护（中断会打断任务）              |


## 实际应用场景

### 场景1: 共享变量保护 (Spinlock)

```c
volatile uint32_t flight_time = 0;
portMUX_TYPE time_mux = portMUX_INITIALIZER_UNLOCKED;

// 定时器中断
void IRAM_ATTR timer_isr() {
  portENTER_CRITICAL_ISR(&time_mux);
  flight_time++;  // 1ms 计时
  portEXIT_CRITICAL_ISR(&time_mux);
}

// 任务读取
void display_task() {
  uint32_t time_copy;
  portENTER_CRITICAL(&time_mux);
  time_copy = flight_time;
  portEXIT_CRITICAL(&time_mux);
  printf("Flight time: %lu ms\n", time_copy);
}
```

**为什么用 Spinlock?**
- 操作极短（几个CPU周期）
- 需要 ISR 安全

---

### 场景2: I2C 总线访问 (Mutex)

```c
SemaphoreHandle_t i2c_mutex;

void sensors_init() {
  i2c_mutex = xSemaphoreCreateMutex();
}

void read_all_sensors() {
  // 可能需要几毫秒
  xSemaphoreTake(i2c_mutex, portMAX_DELAY);
  
  read_mpu6050();   // IMU
  read_ms5611();    // 气压计
  read_hmc5883l();  // 磁力计
  
  xSemaphoreGive(i2c_mutex);
}
```

**为什么用 Mutex?**
- I2C 通信需要几毫秒
- Spinlock 会浪费大量 CPU
- 支持优先级继承

---

### 场景3: 任务同步 (Semaphore)

```c
SemaphoreHandle_t data_ready_sem;

// 传感器任务（生产者）
void sensor_task() {
  while(1) {
    read_sensors();
    xSemaphoreGive(data_ready_sem);  // 通知数据就绪
    vTaskDelay(M2T(1));  // 1ms
  }
}

// 控制任务（消费者）
void control_task() {
  while(1) {
    xSemaphoreTake(data_ready_sem, portMAX_DELAY);  // 等待数据
    run_controller();
    output_motors();
  }
}
```

**为什么用 Semaphore?**
- 用于事件通知
- 生产者 Give，消费者 Take
- 不需要所有权

---

## 常见错误与最佳实践

### ❌ 错误1: 在 ISR 中使用 Mutex

```c
SemaphoreHandle_t mutex;

// ❌ 错误！ISR 不能阻塞
void IRAM_ATTR timer_isr() {
  xSemaphoreTake(mutex, 0);  // 可能阻塞！
  // ...
  xSemaphoreGive(mutex);
}
```

**正确做法**：使用 Binary Semaphore + FromISR
```c
SemaphoreHandle_t sem;

void IRAM_ATTR timer_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

---

### ❌ 错误2: Spinlock 临界区太长

```c
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ❌ 错误！其他核心会一直空转
portENTER_CRITICAL(&mux);
vTaskDelay(M2T(100));  // 延时 100ms，CPU 浪费！
portEXIT_CRITICAL(&mux);
```

**正确做法**：用 Mutex
```c
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

xSemaphoreTake(mutex, portMAX_DELAY);
vTaskDelay(M2T(100));  // 阻塞期间让出 CPU
xSemaphoreGive(mutex);
```

---

### ❌ 错误3: 死锁 (Deadlock)

```c
SemaphoreHandle_t mutex_A, mutex_B;

// Task 1
void task1() {
  xSemaphoreTake(mutex_A, portMAX_DELAY);
  vTaskDelay(1);
  xSemaphoreTake(mutex_B, portMAX_DELAY);  // 等待 mutex_B
  // ...
  xSemaphoreGive(mutex_B);
  xSemaphoreGive(mutex_A);
}

// Task 2
void task2() {
  xSemaphoreTake(mutex_B, portMAX_DELAY);  // 顺序相反！
  vTaskDelay(1);
  xSemaphoreTake(mutex_A, portMAX_DELAY);  // 等待 mutex_A
  // ...
  xSemaphoreGive(mutex_A);
  xSemaphoreGive(mutex_B);
}

// 结果：Task1 持有 A 等待 B，Task2 持有 B 等待 A → 死锁！
```

**正确做法**：统一获取顺序
```c
// 两个任务都按 A → B 的顺序获取
void task1() {
  xSemaphoreTake(mutex_A, portMAX_DELAY);
  xSemaphoreTake(mutex_B, portMAX_DELAY);
  // ...
}

void task2() {
  xSemaphoreTake(mutex_A, portMAX_DELAY);  // 顺序一致
  xSemaphoreTake(mutex_B, portMAX_DELAY);
  // ...
}
```

---

### ❌ 错误4: 忘记释放锁

```c
SemaphoreHandle_t mutex;

void process_data() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  
  if (error_condition) {
    return;  // ❌ 忘记释放 mutex！
  }
  
  // 正常处理
  xSemaphoreGive(mutex);
}
```

**正确做法**：确保所有路径都释放
```c
void process_data() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  
  if (error_condition) {
    xSemaphoreGive(mutex);  // ✓ 释放后返回
    return;
  }
  
  // 正常处理
  xSemaphoreGive(mutex);
}
```

---

## 性能对比

### 时间开销

| 操作                    | 时间 (ESP32 @ 240MHz) |
| ----------------------- | --------------------- |
| Spinlock (无竞争)       | ~50ns                 |
| Spinlock (有竞争，自旋) | 几μs 到无限           |
| Mutex Take/Give         | ~2-5μs                |
| Semaphore Take/Give     | ~2-5μs                |
| 任务切换                | ~10-20μs              |

### 选择流程图

```
需要保护共享资源？
        │
        ├─ YES ─→ 在 ISR 中使用？
        │             │
        │             ├─ YES ─→ Spinlock (CRITICAL_ISR)
        │             │
        │             └─ NO ─→ 临界区时间 < 10μs？
        │                         │
        │                         ├─ YES ─→ Spinlock (CRITICAL)
        │                         │
        │                         └─ NO ─→ 需要优先级继承？
        │                                     │
        │                                     ├─ YES ─→ Mutex
        │                                     └─ NO  ─→ Semaphore
        │
        └─ NO ─→ 需要任务同步/通知？
                      │
                      ├─ YES ─→ Semaphore
                      └─ NO  ─→ Queue
```

---

## 组会讲解大纲

### 第一部分：概念介绍 (5分钟)

#### 1. 为什么需要同步机制？
- 多任务并发访问共享资源
- 竞态条件示例演示
- 数据不一致的后果

#### 2. 四种机制概览
- **Semaphore**: 计数器，资源管理/事件通知
- **Mutex**: 互斥锁，优先级继承
- **Spinlock**: 自旋锁，忙等待
- **中断控制**: 禁用抢占

---

### 第二部分：详细原理 (15分钟)

#### 3. Semaphore 工作原理
- Binary vs Counting
- 生产者-消费者模式
- **ESP-Drone 示例**: systemStart 启动同步机制
  ```c
  systemInit() → Take(canStartMutex)
  其他任务 → systemWaitStart() → 阻塞
  初始化完成 → systemStart() → Give
  所有任务唤醒
  ```

#### 4. Mutex 优先级继承
- 优先级反转问题演示
- 优先级继承解决方案
- **ESP-Drone 示例**: I2C 总线互斥访问
  - MPU6050, MS5611, HMC5883L 共享 I2C 总线
  - 使用 Mutex 防止冲突

#### 5. Spinlock 双核场景
- 临界区实现原理
- 双核自旋等待示意图
- ISR 安全性
- **代码示例**: 共享计数器保护

#### 6. 中断控制层级
- ESP32 中断优先级 (Level 0-7)
- 几种开关中断方式对比
- `portENTER_CRITICAL` vs `portENTER_CRITICAL_ISR`

---

### 第三部分：实战对比 (8分钟)

#### 7. 三个实际场景对比

**场景A: 共享变量 (flight_time)**
- 问题：定时器 ISR 和任务都访问
- 方案：Spinlock (CRITICAL_ISR)
- 理由：极短操作，需要 ISR 安全

**场景B: I2C 总线**
- 问题：多个传感器共享总线
- 方案：Mutex
- 理由：耗时几ms，支持优先级继承

**场景C: 传感器-控制同步**
- 问题：传感器任务通知控制任务
- 方案：Semaphore
- 理由：事件通知，无需所有权

#### 8. 性能对比
- 时间开销表
- CPU 占用对比

#### 9. 决策流程图
- 展示选择流程图

---

### 第四部分：常见错误 (5分钟)

#### 10. 四个典型错误
1. ISR 中使用 Mutex
2. Spinlock 临界区太长
3. 死锁（顺序不一致）
4. 忘记释放锁

#### 11. 最佳实践
- 临界区尽量短
- 统一获取顺序
- 所有路径都释放
- 避免嵌套锁

---

### 第五部分：总结 (2分钟)

#### 12. 快速决策表

| 场景                | 推荐方案                |
| ------------------- | ----------------------- |
| 极短操作 + ISR      | Spinlock (CRITICAL_ISR) |
| 极短操作 + 任务     | Spinlock (CRITICAL)     |
| 长时间 + 优先级继承 | Mutex                   |
| 事件通知            | Semaphore               |
| 资源计数            | Counting Semaphore      |

#### 13. Q&A 准备
- Mutex 和 Binary Semaphore 的区别？
- 什么时候会发生优先级反转？
- Spinlock 会导致死循环吗？
- 如何避免死锁？

---

## 附录：ESP-Drone 中的实际应用

### 1. systemStart 启动同步 (Semaphore)

**文件**: `system.c`

```c
// 初始化
canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
xSemaphoreTake(canStartMutex, portMAX_DELAY);

// 等待启动
void systemWaitStart(void) {
  while (!isInit) vTaskDelay(2);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}

// 启动系统
void systemStart() {
  xSemaphoreGive(canStartMutex);
}
```

**调用位置**:
- `pmTask` (pm_esplane.c:329)
- `stabilizerTask` (stabilizer.c)
- `sensorsTask` (sensors.c)
- `extRxTask` (extrx.c)

---

### 2. Worker 任务队列 (Queue + Spinlock)

**文件**: `worker.c`

```c
static xQueueHandle workerQueue;

// 调度任务（任意任务可调用）
int workerSchedule(void (*function)(void*), void *arg) {
  struct worker_work work = {function, arg};
  xQueueSend(workerQueue, &work, 0);  // 非阻塞发送
}

// 执行任务（systemTask 中）
void workerLoop() {
  while (1) {
    xQueueReceive(workerQueue, &work, portMAX_DELAY);
    work.function(work.arg);
  }
}
```

---

### 3. 传感器数据同步 (Queue)

**文件**: `stabilizer.c`

```c
static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;

// sensorsTask 写入
xQueueOverwrite(accelerometerDataQueue, &acc);
xQueueOverwrite(gyroDataQueue, &gyro);

// stabilizerTask 读取
xQueueReceive(accelerometerDataQueue, &accAverage, 0);
xQueueReceive(gyroDataQueue, &gyroAverage, 0);
```

---

## 总结

### 核心要点

1. **Semaphore**: 事件通知、资源计数，无所有权
2. **Mutex**: 互斥访问、优先级继承，有所有权
3. **Spinlock**: 极短临界区、ISR 安全，忙等待
4. **中断控制**: 硬件寄存器、临界区保护

### 选择原则

- **临界区 < 10μs** → Spinlock
- **临界区 > 10μs** → Mutex/Semaphore
- **ISR 中使用** → Spinlock (CRITICAL_ISR) 或 Semaphore (FromISR)
- **优先级继承** → Mutex
- **事件通知** → Semaphore
- **资源计数** → Counting Semaphore

### 注意事项

- Spinlock 临界区必须极短
- Mutex 不能在 ISR 中使用
- 避免死锁（统一获取顺序）
- 所有路径都要释放锁
- 优先使用 FreeRTOS 提供的机制，少用全局禁中断

---

