# ESP32-S3 VS Code 调试指南

## 概述

本文档介绍如何在 VS Code 中使用 JTAG 硬件调试器对 ESP32-S3 进行源码级调试。调试系统由三部分组成：

```
硬件调试器 ←→ OpenOCD ←→ GDB ←→ VS Code
    ↓
 ESP32-S3
```

- **硬件调试器**：通过 JTAG 接口与芯片通信（如 ESP-Prog、J-Link）
- **OpenOCD**：片上调试服务器，翻译 GDB 命令为 JTAG 信号
- **GDB**：GNU 调试器，提供断点、单步、变量查看等功能
- **VS Code**：可视化调试界面

## 硬件准备

### 1. JTAG 调试器选择

支持的调试器：
- **ESP-Prog**（推荐）：Espressif 官方调试器，原生支持
- **J-Link**：SEGGER 调试器，性能更好但需额外配置
- **FT2232H**：基于 FTDI 芯片的通用调试器

### 2. JTAG 引脚连接

ESP32-S3 默认 JTAG 引脚：

| JTAG 信号 | ESP32-S3 GPIO |
| --------- | ------------- |
| MTDO/TDO  | GPIO39        |
| MTDI/TDI  | GPIO40        |
| MTCK/TCK  | GPIO41        |
| MTMS/TMS  | GPIO42        |
| GND       | GND           |
| VCC       | 3.3V (可选)   |

**注意**：
- 确保调试器和 ESP32-S3 共地（GND）
- VCC 连接可选，取决于调试器是否需要检测目标电压
- 这些引脚不能用于其他功能（如 GPIO、PWM）

### 3. 硬件配置检查

在 `main/Kconfig.projbuild` 中确保没有禁用 JTAG：

```kconfig
# 不要设置这个选项，否则 JTAG 会被禁用
# CONFIG_ESP_SYSTEM_DISABLE_JTAG=y
```

## 软件配置

### 1. ESP-IDF 扩展安装

1. 在 VS Code 中安装 **ESP-IDF** 扩展
2. 配置 ESP-IDF 路径（通常在 `C:\Espressif\esp-idf` 或 `~/.espressif/esp-idf`）
3. 验证工具链安装：
   ```powershell
   idf.py --version
   ```

### 2. 调试配置文件

项目的 `.vscode/launch.json` 已配置好 JTAG 调试：

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "GDB",
      "type": "cppdbg",
      "request": "launch",
      "MIMode": "gdb",
      "miDebuggerPath": "${command:espIdf.getToolchainGdb}",
      "program": "${workspaceFolder}/build/${command:espIdf.getProjectName}.elf",
      "windows": {
        "program": "${workspaceFolder}\\build\\${command:espIdf.getProjectName}.elf"
      },
      "cwd": "${workspaceFolder}",
      "setupCommands": [
        {
          "text": "target remote :3333"
        },
        {
          "text": "set remote hardware-watchpoint-limit 2"
        },
        {
          "text": "mon reset halt"
        },
        {
          "text": "thb app_main"
        },
        {
          "text": "flushregs"
        }
      ],
      "externalConsole": false,
      "logging": {
        "engineLogging": true
      }
    }
  ]
}
```

#### 配置说明

| 配置项                                   | 说明                                            |
| ---------------------------------------- | ----------------------------------------------- |
| `miDebuggerPath`                         | 自动获取 Xtensa GDB 路径（位于 ESP-IDF 工具链） |
| `program`                                | ELF 文件路径，包含调试符号                      |
| `target remote :3333`                    | 连接 OpenOCD 的 GDB 服务器（端口 3333）         |
| `set remote hardware-watchpoint-limit 2` | ESP32-S3 只支持 2 个硬件观察点                  |
| `mon reset halt`                         | 复位芯片并立即暂停                              |
| `thb app_main`                           | 在 `app_main()` 设置临时断点                    |
| `flushregs`                              | 刷新寄存器缓存                                  |

### 3. OpenOCD 配置

ESP-IDF 扩展会自动管理 OpenOCD，但也可以手动启动：

```powershell
# 使用 ESP-Prog 调试器
openocd -f board/esp32s3-builtin.cfg

# 使用 J-Link
openocd -f interface/jlink.cfg -f target/esp32s3.cfg
```

OpenOCD 启动后会监听：
- **端口 3333**：GDB 服务器
- **端口 4444**：Telnet 命令接口

## 调试流程

### 1. 编译固件（带调试符号）

```powershell
# 确保启用调试符号
idf.py menuconfig
# Component config → Compiler options → Optimization Level → Debug (-Og)

# 编译
idf.py build

# 烧录（首次调试需要烧录固件）
idf.py -p COM3 flash
```

**重要**：优化级别选择：
- **Debug (-Og)**：最佳调试体验，但性能较低
- **Release (-O2)**：性能优先，调试时变量可能被优化掉

### 2. 启动 OpenOCD

**方法 A：使用 ESP-IDF 扩展（推荐）**

1. 按 `Ctrl+Shift+P` 打开命令面板
2. 输入 `ESP-IDF: OpenOCD Manager`
3. 选择 `Start OpenOCD`
4. 扩展会自动选择正确的配置文件

**方法 B：手动启动**

在终端运行：
```powershell
openocd -f board/esp32s3-builtin.cfg
```

成功启动后会看到类似输出：
```
Info : [esp32s3] Target halted, PC=0x40043abc
Info : starting gdb server for esp32s3 on 3333
```

### 3. 开始调试

1. **连接硬件**：确保 JTAG 调试器已连接到 ESP32-S3
2. **打开文件**：在 VS Code 中打开要调试的源文件（如 `main/main.c`）
3. **设置断点**：在代码行号左侧点击设置断点
4. **启动调试**：
   - 按 `F5` 或点击 `运行 → 启动调试`
   - 选择 `GDB` 配置
5. **等待停在断点**：程序会在 `app_main()` 函数自动停止

### 4. 调试操作

| 操作     | 快捷键          | 说明                   |
| -------- | --------------- | ---------------------- |
| 继续执行 | `F5`            | 运行到下一个断点       |
| 单步跳过 | `F10`           | 执行当前行，不进入函数 |
| 单步进入 | `F11`           | 进入函数内部           |
| 单步跳出 | `Shift+F11`     | 跳出当前函数           |
| 重启调试 | `Ctrl+Shift+F5` | 复位芯片并重新开始     |
| 停止调试 | `Shift+F5`      | 结束调试会话           |

### 5. 查看变量和内存

- **变量窗口**：自动显示局部变量和全局变量
- **监视窗口**：添加自定义表达式（如 `sensor.accel_x`）
- **调用栈**：查看函数调用链
- **寄存器**：查看 CPU 寄存器值
- **内存查看器**：右键变量 → `View Memory`

## 高级技巧

### 1. 调试 FreeRTOS 任务

查看所有任务状态：

在 GDB 调试控制台（`调试控制台`）输入：
```gdb
-exec info threads
```

切换到特定任务：
```gdb
-exec thread 2
```

### 2. 硬件断点限制

ESP32-S3 只有 **2 个硬件断点**，超过会报错。解决方法：
- 使用条件断点减少断点数量
- 禁用不必要的断点
- 使用软件断点（在 Flash 中运行的代码）

### 3. 观察点（Watchpoint）

监控变量变化：

在调试控制台输入：
```gdb
-exec watch sensor.accel_x
```

当 `sensor.accel_x` 值改变时程序会暂停。

### 4. 实时变量修改

在调试时修改变量值：
1. 在 `变量` 窗口右键变量
2. 选择 `Set Value`
3. 输入新值并回车

### 5. 调试优化代码

如果必须调试 `-O2` 优化的代码：

1. 在 `menuconfig` 中启用 `-fno-inline`：
   ```
   Component config → Compiler options → 
   Disable inlining functions
   ```

2. 特定函数禁用优化：
   ```c
   __attribute__((optimize("O0")))
   void critical_function(void) {
       // 这个函数不会被优化
   }
   ```

## 常见问题

### 1. OpenOCD 连接失败

**错误**：`Error: libusb_open() failed with LIBUSB_ERROR_NOT_FOUND`

**解决**：
- 检查调试器 USB 驱动（ESP-Prog 需要 WinUSB 驱动）
- 使用 Zadig 工具安装正确驱动
- 确保只有一个 OpenOCD 实例在运行

### 2. GDB 连接超时

**错误**：`target remote :3333 timeout`

**解决**：
- 确保 OpenOCD 已启动并监听 3333 端口
- 检查防火墙是否阻止了连接
- 尝试手动启动 OpenOCD 查看错误信息

### 3. 断点无效

**现象**：设置断点后程序不停止

**原因**：
- 代码被编译器优化掉了（检查 `-O2` 优化）
- 代码在 Flash 中且未启用软件断点
- 超过硬件断点限制（最多 2 个）

**解决**：
- 使用 `-Og` 编译
- 减少同时激活的断点数量

### 4. 变量显示 `<optimized out>`

**原因**：编译器优化导致变量被优化到寄存器或删除

**解决**：
- 使用 `-Og` 优化级别
- 将关键变量声明为 `volatile`
- 在函数上添加 `__attribute__((optimize("O0")))`

### 5. 调试时芯片不断重启

**原因**：
- 看门狗未被喂狗
- JTAG 引脚配置冲突

**解决**：
- 在 `menuconfig` 中禁用调试时的看门狗：
  ```
  Component config → ESP System Settings →
  Interrupt watchdog timeout (ms) = 0
  ```
- 确保 JTAG 引脚未被其他功能占用

### 6. 调试 WiFi 或 Bluetooth 代码时卡死

**原因**：某些外设在调试暂停时会超时

**解决**：
- 在关键代码段禁用看门狗
- 使用日志而非断点调试实时代码
- 在中断处理函数中不要设置断点

## 性能对比

| 调试方式       | 优势                                 | 劣势                     | 适用场景               |
| -------------- | ------------------------------------ | ------------------------ | ---------------------- |
| **JTAG 调试**  | 硬件断点、实时内存查看、无需修改代码 | 需要硬件调试器、速度较慢 | 复杂 Bug、底层驱动开发 |
| **串口日志**   | 简单、无需额外硬件、实时性好         | 无法暂停、需修改代码     | WiFi/BLE、高频中断     |
| **Trace 跟踪** | 高速采样、不影响实时性               | 需要专业工具             | 性能分析、协议分析     |

## 最佳实践

1. **开发阶段使用 `-Og`**：保证调试体验
2. **发布前测试 `-O2`**：确保优化后功能正常
3. **关键变量标记 `volatile`**：防止被优化
4. **合理使用断点**：不要超过 2 个硬件断点
5. **调试实时代码用日志**：JTAG 会影响时序
6. **定期清理编译产物**：`idf.py fullclean` 避免调试信息不同步

## 参考资料

- [ESP-IDF JTAG 调试文档](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/api-guides/jtag-debugging/)
- [OpenOCD 用户手册](http://openocd.org/doc/html/index.html)
- [GDB 调试指南](https://sourceware.org/gdb/current/onlinedocs/gdb/)
- ESP-Drone 项目文档：`文档/` 目录

## 项目特定配置

### ESP-Drone 调试要点

1. **双核调试**：ESP32-S3 是双核，GDB 会默认调试 PRO_CPU（核心 0）
2. **FreeRTOS 任务**：使用 `info threads` 查看所有任务
3. **关键任务**：
   - `sensorsTask`：传感器数据读取（500Hz）
   - `stabilizerTask`：控制循环（1000Hz）
   - `commanderTask`：指令处理
   - `wifilink`：WiFi 通信

4. **调试传感器**：在 `sensors_mpu6050.c` 中设置断点查看原始数据
5. **调试控制算法**：在 `controller_pid.c` 中查看 PID 输出
6. **调试估计器**：在 `estimator_kalman.c` 中查看状态估计

---

**最后更新**：2025年12月25日
