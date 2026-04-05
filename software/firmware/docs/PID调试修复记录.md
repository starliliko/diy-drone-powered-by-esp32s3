# PID 控制系统调试与修复记录

**日期**: 2026-02-28  
**平台**: ESP32-S3 + ESP-IDF v5.5.1  
**固件基础**: Crazyflie 移植版  
**飞行构型**: X 型四旋翼  

---

## 一、问题现象

桌面测试时出现：**一个电机满转、对角电机输出为 0**，且两个电机完全不同步，无法形成正常飞行姿态修正。

---

## 二、根本原因分析

经逐层排查，发现三个独立问题叠加导致了该现象：

| #   | 问题              | 位置                         | 影响                 |
| --- | ----------------- | ---------------------------- | -------------------- |
| 1   | Pitch 符号错误    | `controller_pid.c`           | 正反馈 → 失控        |
| 2   | Rate PID 积分饱和 | `pid.h`                      | PID 输出恒为最大值   |
| 3   | Mixer 无修正缩放  | `power_distribution_stock.c` | 低油门时电机极差悬殊 |

---

## 三、修复详情

### 3.1 Pitch 符号错误修复

**文件**: `components/core/crazyflie/modules/src/controller_pid.c`

**问题**: 原始代码中有 `control->pitch = -control->pitch`，注释称"鼻子上仰 = pitch 为负"。经四元数严格推导该假设错误，导致 Pitch 轴形成**正反馈**。

**四元数推导过程**:
```
鼻子上仰 → 绕 Y 轴旋转 -θ (右手系)
q = (cos(θ/2), 0, -sin(θ/2), 0)

gravX = 2*(qx*qz - qw*qy)
      = 2*(0 - cos(θ/2)*(-sin(θ/2)))
      = sin(θ) > 0

pitch = asin(gravX) → 鼻子上仰时 pitch > 0 ✓
```

**修复**: 删除 `control->pitch = -control->pitch` 这一行。保留 `control->yaw = -control->yaw`（X 构型 CW 电机系数与 + 构型方向相反，需取反）。

**修复后符号链路**:
```
Roll:  飞机右倾(roll>0) → PID输出负R → r<0 → 右侧电机(M1,M4)加速 → roll减小 ✓
Pitch: 鼻子下压(pitch<0) → PID输出正P → p>0 → 前方电机(M1,M2)加速 → 鼻子上仰 ✓
Yaw:   取反后符合 X 构型 CW/CCW 电机方向 ✓
```

---

### 3.2 PID 积分饱和修复

**文件**: `components/core/crazyflie/modules/interface/pid.h`

**问题**: 原始 Rate PID 参数中 `KI=440`，`iLimit=33.3`，最大积分输出 = 440 × 33.3 ≈ **14652**，远超典型油门值（10000~25000），导致电机之间差值极大。

**修复策略 — 首飞安全参数**:

| 参数               | 原始值 | 修复后   | 说明                     |
| ------------------ | ------ | -------- | ------------------------ |
| Roll Rate KI       | 440    | **0**    | 纯 PD，消除积分饱和      |
| Pitch Rate KI      | 440    | **0**    | 纯 PD，消除积分饱和      |
| Roll Rate KP       | 250    | 250      | 保持不变                 |
| Roll Rate KD       | 2.5    | 2.5      | 保持不变                 |
| Roll KP (外环)     | 3.5    | **6.0**  | 提高姿态响应速度         |
| Roll KI (外环)     | 2.5    | **1.0**  | 降低积分贡献             |
| Roll iLimit (外环) | 20.0   | **10.0** | 限制最大积分输出 = 10°/s |
| Pitch 同 Roll      | —      | —        | 与 Roll 相同             |
| Yaw KP             | 6.0    | 6.0      | 保持                     |
| Yaw KI             | 1.0    | 1.0      | 保持                     |
| Yaw KD             | 0      | 0.35     | 加少量微分抑制震荡       |

**最终参数**（`#ifdef CONFIG_TARGET_ESP32_S2_DRONE_V1_2`）:
```c
// 内环 Rate PID
#define PID_ROLL_RATE_KP  250.0
#define PID_ROLL_RATE_KI  0.0      // 首飞: 纯PD
#define PID_ROLL_RATE_KD  2.5
#define PID_ROLL_RATE_INTEGRATION_LIMIT  33.3

#define PID_PITCH_RATE_KP  250.0
#define PID_PITCH_RATE_KI  0.0
#define PID_PITCH_RATE_KD  2.5
#define PID_PITCH_RATE_INTEGRATION_LIMIT  33.3

#define PID_YAW_RATE_KP  120.0
#define PID_YAW_RATE_KI  16.7
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_INTEGRATION_LIMIT  166.7

// 外环 Attitude PID
#define PID_ROLL_KP   6.0
#define PID_ROLL_KI   1.0
#define PID_ROLL_KD   0.0
#define PID_ROLL_INTEGRATION_LIMIT  10.0

#define PID_PITCH_KP   6.0
#define PID_PITCH_KI   1.0
#define PID_PITCH_KD   0.0
#define PID_PITCH_INTEGRATION_LIMIT  10.0

#define PID_YAW_KP   6.0
#define PID_YAW_KI   1.0
#define PID_YAW_KD   0.35
#define PID_YAW_INTEGRATION_LIMIT  20.0
```

---

### 3.3 推力比例输出限幅

**文件**: `components/core/crazyflie/modules/src/controller_pid.c`

**问题**: 即使 PID 参数合理，极端情况下 R/P/Y 输出之和仍可能超过推力 T，导致某个电机输出为负（被 clamp 到 0）。

**修复**: 在 mixer 之前，根据当前推力值动态限制 R/P/Y 输出范围：

```
|R| ≤ 0.6T
|P| ≤ 0.6T
|Y| ≤ 0.2T
→ 最差电机 ≥ T - 0.3T - 0.3T - 0.2T = 0.2T（至少保留 20% 推力）
```

同时修复了一个**整数溢出 Bug**：

```c
// ❌ 错误：T > 54612 时 thr*0.6 > 32767，int16_t溢出变负数
int16_t rpLim = (int16_t)(thr * 0.6f);

// ✅ 修复：用int32_t中间变量，再clamp到INT16_MAX
int32_t rpLim32 = (int32_t)(thr * 0.6f);
int16_t rpLim = (rpLim32 > INT16_MAX) ? INT16_MAX : (int16_t)rpLim32;
```

当 T = 60000（满油门）时，若不修复，`thr * 0.6f = 36000` 截断为 `int16_t` 后变为负数（约 -29536），导致限幅逻辑反向，R 和 P 都被钳制到 -29536。修复后限幅值正确取 INT16_MAX = 32767。

---

### 3.4 Mixer 修正缩放

**文件**: `components/core/crazyflie/modules/src/power_distribution_stock.c`

**问题**: 低油门时若 `|r|+|p|+|y| > T`，某些电机计算值为负，被 `limitThrust` 强制截断到 0，产生非线性死区。

**修复**: 在 mixer 执行前等比例缩放修正量：

```c
int32_t totalCorrection = absR + absP + absY;
if (thrust > 0 && totalCorrection > thrust)
{
    float scale = (float)thrust / (float)totalCorrection;
    r = (int16_t)(r * scale);
    p = (int16_t)(p * scale);
    y = (int16_t)(y * scale);
}
```

---

## 四、陀螺仪 Y 轴取反说明

**文件**: `controller_pid.c` Line ~130

```c
attitudeControllerCorrectRatePID(
    sensors->gyro.x,
    -sensors->gyro.y,  // Y轴取反
    sensors->gyro.z,
    ...
);
```

BMI088 传感器坐标系中 gyro.y 的正方向与飞控定义的 Pitch 速率正方向相反，因此在传入 Rate PID 之前将 gyro.y 取反，确保内环误差方向正确。

---

## 五、测试结果

### 修复前（原始参数）
```
T=15000  R=-24000  P=+24000  Y=+15000
M1=0  M2=65535  M3=0  M4=0   ← 极端失衡
```

### 修复后（现行参数）
```
T=24500  R=-4000  P=+3500  Y=+200
M1=26500  M2=26000  M3=18400  M4=27000  ← 分布合理
```

输出特征：
- R 和 P 与推力成比例，有界
- 各电机差值不超过 ±30%
- 不再出现单电机满转/归零的极端情况

---

## 六、后续调参建议

### 首飞后（确认方向正确）

1. **逐步加入 Rate KI**：从 50 开始，观察抗扰能力提升
   ```c
   #define PID_ROLL_RATE_KI  50.0   // 尝试值
   #define PID_ROLL_RATE_INTEGRATION_LIMIT  20.0
   ```

2. **如响应迟钝**：适当提高 Rate KP（250 → 300）

3. **Yaw 积分漂移**：桌面测试中 Y 会缓慢累积到 ~2000~5000，飞行时重力约束会改善，若仍漂移可降低 `PID_YAW_RATE_KI`

### 调参优先级

```
1. 确认 Roll/Pitch 方向 → 2. Rate KP 粗调 → 3. 加入 Rate KI → 4. 外环 KP 细调
```

---

## 七、修改文件汇总

| 文件                                                               | 修改内容                                                         |
| ------------------------------------------------------------------ | ---------------------------------------------------------------- |
| `components/core/crazyflie/modules/src/controller_pid.c`           | 删除 Pitch 取反；添加推力比例限幅；修复 int16 溢出；添加诊断日志 |
| `components/core/crazyflie/modules/interface/pid.h`                | Rate KI 归零；外环 KP=6/KI=1/iLimit=10；Yaw KD=0.35              |
| `components/core/crazyflie/modules/src/power_distribution_stock.c` | 添加等比例修正缩放；添加诊断日志                                 |
