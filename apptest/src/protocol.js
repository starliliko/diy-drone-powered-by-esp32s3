/**
 * ESP-Drone 通信协议定义
 * ==========================
 * 定义与飞控通信的所有协议常量和数据结构
 */

// 协议基础常量
const MAGIC = [0xAB, 0xCD];
const PROTOCOL_VERSION = 0x01;
const HEADER_SIZE = 8;

// 发现协议魔数 "ESPD"
const DISCOVERY_MAGIC = Buffer.from([0x45, 0x53, 0x50, 0x44]);

// UDP 遥测包魔数 "ESPU" + 类型字节
// 格式: [0x45 0x53 0x50 0x55] [1字节类型=PacketType.TELEMETRY] [1字节序号] [遥测payload]
const UDP_TELEMETRY_MAGIC = Buffer.from([0x45, 0x53, 0x50, 0x55]);
const UDP_TELEMETRY_HEADER_SIZE = 6; // magic(4) + type(1) + seq(1)

// 数据包类型
const PacketType = {
    HEARTBEAT: 0x00,
    TELEMETRY: 0x01,
    CONTROL: 0x02,
    CRTP: 0x03,
    ACK: 0x04,
    CONFIG: 0x05,
    LOG: 0x06
};

// 控制指令类型
const ControlCmdType = {
    RPYT: 0x00,
    VELOCITY: 0x01,
    POSITION: 0x02,
    HOVER: 0x03,
    LAND: 0x04,
    EMERGENCY: 0x05,
    ARM: 0x06,
    DISARM: 0x07,
    SET_CONTROL_MODE: 0x10
};

// 远程控制模式
const RemoteControlMode = {
    DISABLED: 0,    // 服务器控制禁用
    ENABLED: 1,     // 服务器控制启用
    SHARED: 2       // 共享控制
};

const RemoteControlModeNameCN = {
    0: '禁用',
    1: '启用',
    2: '协同'
};

// 目标飞行模式（前端发送的期望模式）
const FlightMode = {
    0: 'MANUAL',
    1: 'STABILIZE',
    2: 'ALTITUDE',
    3: 'POSITION',
    4: 'MISSION',
    5: 'RTL',
    6: 'LAND',
    7: 'TAKEOFF',
    8: 'OFFBOARD'
};

const FlightModeNameCN = {
    0: '手动模式',
    1: '自稳模式',
    2: '定高模式',
    3: '定点模式',
    4: '任务模式',
    5: '返航模式',
    6: '降落模式',
    7: '起飞模式',
    8: '外部控制'
};

// 实际飞行姿态模式（由传感器能力决定）
const ActualFlightMode = {
    0: 'STABILIZE',
    1: 'ALTHOLD',
    2: 'POSHOLD'
};

const ActualFlightModeNameCN = {
    0: '自稳模式',
    1: '定高模式',
    2: '定点模式'
};

// 控制来源
const ControlSource = {
    0: 'NONE',
    1: 'CRTP',
    2: 'REMOTE',
    3: 'SBUS'
};

const ControlSourceNameCN = {
    0: '无控制',
    1: 'WiFi控制',
    2: '地面站',
    3: '遥控器'
};

// PX4风格解锁状态
const ArmingState = {
    0: 'INIT',
    1: 'STANDBY',
    2: 'ARMED',
    3: 'STANDBY_ERROR',
    4: 'SHUTDOWN',
    5: 'IN_AIR_RESTORE'
};

const ArmingStateNameCN = {
    0: '初始化',
    1: '待机',
    2: '已解锁',
    3: '待机错误',
    4: '关机',
    5: '空中恢复'
};

// 飞行阶段
const FlightPhase = {
    0: 'ON_GROUND',
    1: 'TAKEOFF',
    2: 'IN_AIR',
    3: 'LANDING',
    4: 'LANDED'
};

const FlightPhaseNameCN = {
    0: '地面待机',
    1: '起飞中',
    2: '空中飞行',
    3: '降落中',
    4: '已降落'
};

// 故障安全状态
const FailsafeState = {
    0: 'NONE',
    1: 'LOW_BATTERY',
    2: 'CRITICAL_BATTERY',
    3: 'RC_LOSS',
    4: 'GCS_LOSS',
    5: 'SENSOR_FAILURE',
    6: 'GEOFENCE'
};

// 状态标志位
const STATUS_FLAGS = {
    ARMED: 0x01,
    FLYING: 0x02,
    EMERGENCY: 0x04,
    RC_CONNECTED: 0x08,
    GCS_CONNECTED: 0x10,
    BATTERY_LOW: 0x20,
    TUMBLED: 0x40
};

module.exports = {
    MAGIC,
    PROTOCOL_VERSION,
    HEADER_SIZE,
    DISCOVERY_MAGIC,
    UDP_TELEMETRY_MAGIC,
    UDP_TELEMETRY_HEADER_SIZE,
    PacketType,
    ControlCmdType,
    RemoteControlMode,
    RemoteControlModeNameCN,
    FlightMode,
    FlightModeNameCN,
    ActualFlightMode,
    ActualFlightModeNameCN,
    ControlSource,
    ControlSourceNameCN,
    ArmingState,
    ArmingStateNameCN,
    FlightPhase,
    FlightPhaseNameCN,
    FailsafeState,
    STATUS_FLAGS
};
