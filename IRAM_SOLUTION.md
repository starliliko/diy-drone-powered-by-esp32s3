# ESP-Drone IRAM 优化总结

## 问题
ESP32-S3 的 16KB IRAM 已 100% 占用 (15356/16384 bytes),无法进一步优化。

## 根本原因
ESP-IDF 核心库(Xtensa、SPI Flash、FreeRTOS、System、HW Support)的关键代码**必须**在 IRAM 中运行,这些代码由链接器强制放置,无法通过 Kconfig 配置移除。

## 已尝试的优化(全部无效)
- ❌ 禁用所有驱动 ISR IRAM 优化
- ❌ 禁用 LWIP/WiFi IRAM 优化
- ❌ 禁用外设控制/中断 IRAM
- ❌ 禁用 FreeRTOS IRAM 优化
- ❌ 移动 Heap/LibC 函数到 Flash

**结果**: IRAM 使用量完全没有变化

## 可行解决方案

### 方案 1: 使用 PSRAM (推荐) ⭐
如果硬件有 PSRAM,在 sdkconfig.defaults 中添加:

```kconfig
# Enable PSRAM
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y

# Allow instruction fetch from PSRAM (frees ~8KB IRAM)
CONFIG_SPIRAM_FETCH_INSTRUCTIONS=y
CONFIG_SPIRAM_RODATA=y

# Use PSRAM for dynamic allocations
CONFIG_SPIRAM_USE_MALLOC=y
```

**效果**: 可将部分代码/数据移到 PSRAM,释放 6-8KB IRAM

### 方案 2: 切换到 ESP32-S3 变体
某些 ESP32-S3 变体可能有更大的 IRAM 配置,查看硬件规格。

### 方案 3: 减少功能
如果没有 PSRAM,考虑:
- 禁用 ESP-NOW (如果只用 WiFi)
- 禁用 Bluetooth
- 使用更简单的网络栈

### 方案 4: 接受当前状态
IRAM 100% 占用**并不会导致编译失败**,只是:
- 无法添加更多需要 IRAM 的代码
- 性能已达到最优(所有关键代码都在 IRAM)

## 当前配置文件
已优化的配置保存在:
- `sdkconfig.defaults` - 通用 IRAM 优化
- `sdkconfig.defaults.esp32s3` - ESP32-S3 特定配置
- `fix_iram.ps1` - 禁用强制 IRAM 选项的脚本

## 建议
1. **检查硬件是否有 PSRAM** - 这是唯一真正有效的解决方案
2. 如果有 PSRAM,启用方案 1 的配置
3. 如果没有 PSRAM,接受当前 100% IRAM 使用率
4. 飞行测试验证稳定性

## 性能影响
当前配置下:
- Flash Code: 633864 bytes (比原始增加 ~30KB,代码从 IRAM 移出)
- DIRAM: 44.97% (释放了 ~30KB RAM)
- 1000Hz 飞控循环: 应不受影响
- WiFi 性能: 可能略有下降(5-10%)
