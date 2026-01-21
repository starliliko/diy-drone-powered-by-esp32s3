#!/usr/bin/env python3
"""
ESP-Drone 远程服务器示例
========================

这是一个简单的 TCP 服务器示例，用于与 ESP-Drone 进行双向通信。

功能：
1. 接收无人机遥测数据
2. 发送控制指令
3. 心跳保活

使用方法：
    python remote_server_demo.py

默认监听 0.0.0.0:8080，可通过命令行参数修改。

协议说明：
- 包头：0xAB 0xCD + version(1) + type(1) + seq(2) + length(2) = 8 bytes
- 包体：根据 type 不同有不同结构
"""

import socket
import struct
import threading
import time
import sys
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Callable
import argparse


# ============================================================================
# 协议定义
# ============================================================================

MAGIC_0 = 0xAB
MAGIC_1 = 0xCD
PROTOCOL_VERSION = 0x01
HEADER_SIZE = 8


class PacketType(IntEnum):
    HEARTBEAT = 0x00
    TELEMETRY = 0x01
    CONTROL = 0x02
    CRTP = 0x03
    ACK = 0x04
    CONFIG = 0x05
    LOG = 0x06


class ControlCmdType(IntEnum):
    RPYT = 0x00
    VELOCITY = 0x01
    POSITION = 0x02
    HOVER = 0x03
    LAND = 0x04
    EMERGENCY = 0x05
    ARM = 0x06
    DISARM = 0x07


@dataclass
class PacketHeader:
    magic0: int
    magic1: int
    version: int
    pkt_type: int
    seq: int
    length: int

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['PacketHeader']:
        if len(data) < HEADER_SIZE:
            return None
        magic0, magic1, version, pkt_type, seq, length = struct.unpack('<BBBBHH', data[:HEADER_SIZE])
        if magic0 != MAGIC_0 or magic1 != MAGIC_1:
            return None
        return cls(magic0, magic1, version, pkt_type, seq, length)

    def to_bytes(self) -> bytes:
        return struct.pack('<BBBBHH', 
                          self.magic0, self.magic1, 
                          self.version, self.pkt_type, 
                          self.seq, self.length)


@dataclass
class TelemetryData:
    """遥测数据结构"""
    roll: float        # 度
    pitch: float       # 度
    yaw: float         # 度
    gyro_x: float      # deg/s
    gyro_y: float      # deg/s
    gyro_z: float      # deg/s
    acc_x: float       # mg
    acc_y: float       # mg
    acc_z: float       # mg
    pos_x: float       # m
    pos_y: float       # m
    pos_z: float       # m
    vel_x: float       # m/s
    vel_y: float       # m/s
    vel_z: float       # m/s
    batt_voltage: float  # V
    batt_percent: int    # %
    flight_mode: int
    is_armed: bool
    is_low_battery: bool
    timestamp: int       # ms

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['TelemetryData']:
        if len(data) < 40:  # 最小尺寸
            return None
        
        # 解析原始数据
        values = struct.unpack('<hhhhhhhhhiiihhhHBBBBxxI', data[:44])
        
        return cls(
            roll=values[0] / 100.0,
            pitch=values[1] / 100.0,
            yaw=values[2] / 100.0,
            gyro_x=values[3] / 10.0,
            gyro_y=values[4] / 10.0,
            gyro_z=values[5] / 10.0,
            acc_x=values[6],
            acc_y=values[7],
            acc_z=values[8],
            pos_x=values[9] / 1000.0,
            pos_y=values[10] / 1000.0,
            pos_z=values[11] / 1000.0,
            vel_x=values[12] / 1000.0,
            vel_y=values[13] / 1000.0,
            vel_z=values[14] / 1000.0,
            batt_voltage=values[15] / 1000.0,
            batt_percent=values[16],
            flight_mode=values[17],
            is_armed=bool(values[18]),
            is_low_battery=bool(values[19]),
            timestamp=values[20]
        )


@dataclass
class ControlCommand:
    """控制指令结构"""
    cmd_type: ControlCmdType
    roll: float = 0.0       # 度
    pitch: float = 0.0      # 度
    yaw: float = 0.0        # deg/s
    thrust: int = 0         # 0-65535
    mode: int = 0

    def to_bytes(self) -> bytes:
        return struct.pack('<BhhhHBxxx',
                          self.cmd_type,
                          int(self.roll * 100),
                          int(self.pitch * 100),
                          int(self.yaw * 10),
                          self.thrust,
                          self.mode)


# ============================================================================
# 客户端连接处理
# ============================================================================

class DroneClient:
    """处理单个无人机连接"""
    
    def __init__(self, sock: socket.socket, addr: tuple, server: 'DroneServer'):
        self.sock = sock
        self.addr = addr
        self.server = server
        self.running = False
        self.tx_seq = 0
        self.rx_buffer = b''
        
        # 统计信息
        self.rx_count = 0
        self.tx_count = 0
        self.last_heartbeat = time.time()
        self.connect_time = time.time()
        
        # 最新遥测
        self.latest_telemetry: Optional[TelemetryData] = None
    
    def start(self):
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
    
    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except:
            pass
    
    def _rx_loop(self):
        """接收循环"""
        self.sock.settimeout(1.0)
        
        while self.running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    print(f"[{self.addr}] Connection closed by drone")
                    break
                
                self.rx_buffer += data
                self._process_buffer()
                
            except socket.timeout:
                # 检查心跳超时
                if time.time() - self.last_heartbeat > 10:
                    print(f"[{self.addr}] Heartbeat timeout")
                    break
            except Exception as e:
                print(f"[{self.addr}] RX error: {e}")
                break
        
        self.running = False
        self.server.remove_client(self)
    
    def _process_buffer(self):
        """处理接收缓冲区"""
        while len(self.rx_buffer) >= HEADER_SIZE:
            # 查找包头
            header = PacketHeader.from_bytes(self.rx_buffer)
            if header is None:
                # 同步丢失，跳过一个字节
                self.rx_buffer = self.rx_buffer[1:]
                continue
            
            total_len = HEADER_SIZE + header.length
            if len(self.rx_buffer) < total_len:
                # 数据不完整
                break
            
            # 提取包
            payload = self.rx_buffer[HEADER_SIZE:total_len]
            self.rx_buffer = self.rx_buffer[total_len:]
            
            self.rx_count += 1
            self._handle_packet(header, payload)
    
    def _handle_packet(self, header: PacketHeader, payload: bytes):
        """处理接收到的包"""
        pkt_type = PacketType(header.pkt_type)
        
        if pkt_type == PacketType.HEARTBEAT:
            self.last_heartbeat = time.time()
            # 回复心跳
            self._send_heartbeat()
            
        elif pkt_type == PacketType.TELEMETRY:
            telemetry = TelemetryData.from_bytes(payload)
            if telemetry:
                self.latest_telemetry = telemetry
                self.server.on_telemetry(self, telemetry)
                
        elif pkt_type == PacketType.CRTP:
            self.server.on_crtp(self, payload)
            
        elif pkt_type == PacketType.LOG:
            try:
                log_msg = payload.decode('utf-8', errors='replace')
                print(f"[{self.addr}] LOG: {log_msg}")
            except:
                pass
    
    def _send_heartbeat(self):
        """发送心跳响应"""
        timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        self.send_packet(PacketType.HEARTBEAT, struct.pack('<I', timestamp))
    
    def send_packet(self, pkt_type: PacketType, payload: bytes = b''):
        """发送数据包"""
        header = PacketHeader(
            magic0=MAGIC_0,
            magic1=MAGIC_1,
            version=PROTOCOL_VERSION,
            pkt_type=pkt_type,
            seq=self.tx_seq,
            length=len(payload)
        )
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        
        try:
            self.sock.sendall(header.to_bytes() + payload)
            self.tx_count += 1
        except Exception as e:
            print(f"[{self.addr}] TX error: {e}")
    
    def send_control(self, cmd: ControlCommand):
        """发送控制指令"""
        self.send_packet(PacketType.CONTROL, cmd.to_bytes())


# ============================================================================
# 服务器
# ============================================================================

class DroneServer:
    """无人机远程服务器"""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 8080):
        self.host = host
        self.port = port
        self.running = False
        self.clients: list[DroneClient] = []
        self.clients_lock = threading.Lock()
        
        # 回调
        self.telemetry_callback: Optional[Callable[[DroneClient, TelemetryData], None]] = None
    
    def start(self):
        """启动服务器"""
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)
        
        print(f"Server listening on {self.host}:{self.port}")
        
        self.running = True
        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.accept_thread.start()
    
    def stop(self):
        """停止服务器"""
        self.running = False
        with self.clients_lock:
            for client in self.clients:
                client.stop()
            self.clients.clear()
        
        try:
            self.server_sock.close()
        except:
            pass
    
    def _accept_loop(self):
        """接受连接循环"""
        self.server_sock.settimeout(1.0)
        
        while self.running:
            try:
                client_sock, addr = self.server_sock.accept()
                print(f"New connection from {addr}")
                
                client = DroneClient(client_sock, addr, self)
                with self.clients_lock:
                    self.clients.append(client)
                client.start()
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Accept error: {e}")
    
    def remove_client(self, client: DroneClient):
        """移除客户端"""
        with self.clients_lock:
            if client in self.clients:
                self.clients.remove(client)
                print(f"Client {client.addr} disconnected")
    
    def on_telemetry(self, client: DroneClient, data: TelemetryData):
        """遥测数据回调"""
        if self.telemetry_callback:
            self.telemetry_callback(client, data)
    
    def on_crtp(self, client: DroneClient, data: bytes):
        """CRTP 数据回调"""
        # 可以转发到其他系统
        pass
    
    def broadcast_control(self, cmd: ControlCommand):
        """向所有客户端广播控制指令"""
        with self.clients_lock:
            for client in self.clients:
                client.send_control(cmd)


# ============================================================================
# 主程序
# ============================================================================

def print_telemetry(client: DroneClient, data: TelemetryData):
    """打印遥测数据"""
    print(f"\n[{client.addr}] Telemetry @ {data.timestamp}ms:")
    print(f"  Attitude: R={data.roll:.1f}° P={data.pitch:.1f}° Y={data.yaw:.1f}°")
    print(f"  Position: X={data.pos_x:.2f}m Y={data.pos_y:.2f}m Z={data.pos_z:.2f}m")
    print(f"  Battery: {data.batt_voltage:.2f}V ({data.batt_percent}%)")
    print(f"  Armed: {data.is_armed}, Mode: {data.flight_mode}")


def interactive_control(server: DroneServer):
    """交互式控制"""
    print("\n=== ESP-Drone Remote Control ===")
    print("Commands:")
    print("  h - hover")
    print("  l - land")
    print("  e - emergency stop")
    print("  w/s/a/d - pitch/roll control")
    print("  q/r - yaw left/right")
    print("  t - telemetry status")
    print("  x - exit")
    print()
    
    while True:
        try:
            cmd = input("> ").strip().lower()
            
            if cmd == 'x':
                break
            elif cmd == 'h':
                server.broadcast_control(ControlCommand(ControlCmdType.HOVER, thrust=1000))
                print("Hover command sent")
            elif cmd == 'l':
                server.broadcast_control(ControlCommand(ControlCmdType.LAND))
                print("Land command sent")
            elif cmd == 'e':
                server.broadcast_control(ControlCommand(ControlCmdType.EMERGENCY))
                print("EMERGENCY STOP sent!")
            elif cmd == 'w':
                server.broadcast_control(ControlCommand(ControlCmdType.RPYT, pitch=-5.0, thrust=30000))
            elif cmd == 's':
                server.broadcast_control(ControlCommand(ControlCmdType.RPYT, pitch=5.0, thrust=30000))
            elif cmd == 'a':
                server.broadcast_control(ControlCommand(ControlCmdType.RPYT, roll=-5.0, thrust=30000))
            elif cmd == 'd':
                server.broadcast_control(ControlCommand(ControlCmdType.RPYT, roll=5.0, thrust=30000))
            elif cmd == 'q':
                server.broadcast_control(ControlCommand(ControlCmdType.RPYT, yaw=-30.0, thrust=30000))
            elif cmd == 'r':
                server.broadcast_control(ControlCommand(ControlCmdType.RPYT, yaw=30.0, thrust=30000))
            elif cmd == 't':
                with server.clients_lock:
                    for c in server.clients:
                        if c.latest_telemetry:
                            print_telemetry(c, c.latest_telemetry)
                        else:
                            print(f"[{c.addr}] No telemetry yet")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    parser = argparse.ArgumentParser(description='ESP-Drone Remote Server')
    parser.add_argument('--host', default='0.0.0.0', help='Server host')
    parser.add_argument('--port', type=int, default=8080, help='Server port')
    parser.add_argument('--verbose', '-v', action='store_true', help='Print all telemetry')
    args = parser.parse_args()
    
    server = DroneServer(args.host, args.port)
    
    if args.verbose:
        server.telemetry_callback = print_telemetry
    
    try:
        server.start()
        interactive_control(server)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.stop()


if __name__ == '__main__':
    main()
