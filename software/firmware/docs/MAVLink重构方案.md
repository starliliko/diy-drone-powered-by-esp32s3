# MAVLink 重构方案（飞控 <-> 地面站）

> **状态更新**：本文原为重构设计讴商。到 2026-04 为止，本文描述的地面站侧与固件侧双栈都已实现：
>
> - 地面站：`groundstation/src/mavlink.js` + `DroneClient.js` 双栈解析
> - 固件：`components/core/crazyflie/modules/src/remote_server.c` 完整实现 MAVLink v2 帧识别、CRC、`PARAM_SET` / `MANUAL_CONTROL` / `COMMAND_LONG` 下行与 `HEARTBEAT` / `SYS_STATUS` / `ATTITUDE` / `SERVO_OUTPUT_RAW` / `LOCAL_POSITION_NED` / `HIGHRES_IMU` 上行
> - 协议模式：`PROTOCOL_MODE = legacy | mavlink | dual`，默认 `mavlink`
>
> 第 4 节“固件端最小改造清单”仅作为历史参考，实际实现以代码为准。

## 1. 目标

将当前自定义二进制协议（magic=0xABCD）迁移到标准 MAVLink v2，并在迁移期支持双栈：

- `legacy`：仅旧协议
- `mavlink`：仅 MAVLink
- `dual`：双协议兼容（推荐灰度模式）

## 2. 当前已完成（地面站）

已在 groundstation 侧完成 MAVLink 双栈基础能力：

- 新增 MAVLink 最小编解码模块：
  - `groundstation/src/mavlink.js`
- `DroneClient` 支持 legacy + MAVLink 双栈解析与协议自动识别：
  - `groundstation/src/DroneClient.js`
- 新增协议模式开关：
  - `groundstation/src/config.js` -> `PROTOCOL_MODE`

## 3. 消息映射（建议）

### 3.1 飞控上行遥测

1. 姿态：`ATTITUDE (30)`
- roll/pitch/yaw（rad）
- rollspeed/pitchspeed/yawspeed（rad/s）

2. 电池：`SYS_STATUS (1)`
- `voltage_battery`（mV）
- `battery_remaining`（%）

3. 位置与速度：`LOCAL_POSITION_NED (32)`
- x/y/z（m）
- vx/vy/vz（m/s）

4. IMU 原始量：`HIGHRES_IMU (105)`
- xacc/yacc/zacc（m/s^2）
- xgyro/ygyro/zgyro（rad/s）

5. 心跳：`HEARTBEAT (0)`
- armed 状态放到 `base_mode` 的 `MAV_MODE_FLAG_SAFETY_ARMED`
- flight mode 通过 `custom_mode` 扩展（可选）

### 3.2 地面站下行控制

1. 手动姿态/油门：`MANUAL_CONTROL (69)`
- x/y/z/r 分别映射 pitch/roll/thrust/yaw

2. 解锁/上锁：`COMMAND_LONG (76)` + `MAV_CMD_COMPONENT_ARM_DISARM (400)`
- param1=1 解锁
- param1=0 上锁

3. 降落：`COMMAND_LONG (76)` + `MAV_CMD_NAV_LAND (21)`

4. 急停（兼容方案）
- 仍使用 `MAV_CMD_COMPONENT_ARM_DISARM`，param1=0，param2=21196（force disarm）

## 4. 固件端最小改造清单（已落地，仅备查）

> 以下所有项在 [`remote_server.c`](../components/core/crazyflie/modules/src/remote_server.c) 中都已完成。

1. ✅ 在 `remote_server.c` 增加 MAVLink 帧解析器
- 识别 `0xFD/0xFE` 起始字节
- 校验 X25 CRC + extra CRC
- 分发 `MANUAL_CONTROL` / `COMMAND_LONG`

2. ✅ 在 `handleControlPacket` 之外新增 MAVLink 控制入口
- `MANUAL_CONTROL -> commanderSetSetpoint(COMMANDER_PRIORITY_REMOTE)`
- `COMMAND_LONG(ARM/DISARM/LAND) -> vehicle_state` 对应接口

3. ✅ 在遥测任务中发送 MAVLink 消息
- 每个周期发送 `ATTITUDE`
- 低频发送 `SYS_STATUS`
- 视带宽发送 `LOCAL_POSITION_NED` / `HIGHRES_IMU`

4. ✅ 迁移期间保留 legacy 通道
- 由 Kconfig 控制：`REMOTE_PROTOCOL_MODE = LEGACY / MAVLINK / DUAL`

## 5. 风险与建议

1. 带宽与周期
- MAVLink 消息拆分后包数增多，建议对不同消息设置不同发送频率。

2. 坐标系与符号
- NED 与当前机体系/世界系存在符号差异，必须统一（尤其 z、yaw、roll）。

3. CRC 与 extra CRC
- 若消息 extra CRC 不匹配，会导致静默丢包。建议在固件加入统计计数器。

4. 回滚策略
- 默认先 `dual`，飞测稳定后切 `mavlink`。

## 6. 验收标准

1. 地面站显示稳定
- roll/pitch/yaw、电池、高度、速度正确更新

2. 控制正确
- ARM/DISARM/LAND/RPYT 可用，无方向反转

3. 故障恢复
- 断链重连后恢复正常

4. 兼容性
- `dual` 模式下 legacy 与 MAVLink 均可接入
