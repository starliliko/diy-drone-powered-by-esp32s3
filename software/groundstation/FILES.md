# 项目文件组织说明

## 文件清单

本项目已完成**模块化架构重构**，从单一文件升级为多模块设计。

### 📁 核心文件

#### 服务器入口
- **server.js** - 新的服务器主入口文件
  - 启动 DroneServer
  - 处理信号和异常
  - 输出启动信息

#### 后端模块 (src/)
- **src/config.js** - 全局配置常量
  - TCP/HTTP 端口定义
  - 协议参数
  - 超时时间设置

- **src/protocol.js** - 协议定义和常量
  - PacketType (0x00-0x06)
  - FlightMode (0-5)
  - ControlSource (0-3)
  - ControlCmdType (0-7)

- **src/parser.js** - 数据包编解码
  - `parseHeader()` - 解析 8 字节包头
  - `parseTelemetry()` - 解析 44 字节遥测数据
  - `createHeader()` - 创建包头
  - `createControlCommand()` - 创建控制命令包

- **src/DroneClient.js** - 单个飞控 TCP 连接
  - 处理 TCP 接收数据
  - 心跳机制
  - 遥测数据缓存

- **src/DroneServer.js** - 主服务器类
  - TCP 服务器 (port 8080)
  - HTTP 服务器 (port 3000)
  - WebSocket 服务器
  - REST API 路由 (/api/status, /api/control)
  - 多客户端管理

#### 前端静态文件 (public/)

**HTML 模板**
- **public/index.html** - 完整的 HTML 页面结构
  - 导航栏和标签页
  - 左侧遥测面板
  - 中间 3D 渲染区域
  - 右侧控制面板
  - 底部日志控制台
  - 电机测试页面

**样式表**
- **public/css/style.css** - QGC 风格的完整样式
  - 深色主题 (QGroundControl 风格)
  - 响应式布局
  - 自定义控件样式

**JavaScript 模块** (public/js/)
- **app.js** - 主入口，模块初始化
  - 导入所有子模块
  - 页面导航函数
  - 全局函数暴露

- **visualization.js** - Three.js 3D 可视化
  - 场景初始化
  - 飞机模型创建
  - 姿态角更新

- **websocket.js** - WebSocket 连接管理
  - 连接/重连
  - 消息处理
  - 心跳保活
  - 日志输出

- **control.js** - 飞行控制命令
  - ARM/DISARM
  - TAKEOFF/LAND
  - RTL/HOVER
  - 远程控制模式设置

- **motorTest.js** - 电机测试工具
  - 单个电机推力控制
  - 顺序测试
  - 全部停止

- **dashboard.js** - 实时数据仪表盘
  - 遥测数据显示
  - 状态指示器更新
  - 电池显示

#### 配置文件
- **package.json** - Node.js 项目配置
  - 依赖: express, ws, bonjour-service
  - 启动脚本: npm start, npm run dev
  - 项目元数据

#### 文档文件
- **README.md** - 项目说明和快速开始
  - 功能特性
  - 快速开始步骤
  - 飞控配置
  - 协议说明
  - API 文档
  - 故障排除

- **ARCHITECTURE.md** - 详细架构设计文档
  - 系统架构图
  - 模块层次结构
  - 数据流程
  - 前端模块详解
  - 通信协议
  - 扩展点
  - 部署指南

- **MIGRATION.md** - 模块化迁移指南
  - 迁移概述
  - 文件结构变化
  - 启动命令变化
  - API 兼容性说明
  - 常见问题解答

- **FILES.md** - 本文件（文件清单说明）

### 📊 代码统计

| 模块               | 文件数 | 代码行数  | 用途               |
| ------------------ | ------ | --------- | ------------------ |
| 后端 (src/)        | 5      | ~400      | TCP/HTTP/WS 服务器 |
| 前端 (public/js/)  | 6      | ~800      | 前端交互和可视化   |
| 前端 (public/css/) | 1      | ~400      | UI 样式            |
| HTML               | 1      | ~300      | 页面结构           |
| 配置和文档         | 5      | ~1000     | 说明和配置         |
| **总计**           | **18** | **~2900** | -                  |

### 🗑️ 删除的文件

- ❌ `app.js` - 旧的 2136 行单文件版本 (已分解为模块)
- ❌ `app-legacy.js` - 备份文件 (如存在)

### 📦 依赖关系

```
server.js (入口)
    ↓
src/DroneServer.js (主服务器)
    ├── src/config.js (配置)
    ├── src/protocol.js (协议)
    ├── src/parser.js (解析)
    └── src/DroneClient.js (TCP 客户端)

public/index.html (前端模板)
    ├── public/css/style.css (样式)
    └── public/js/app.js (主入口)
        ├── visualization.js
        ├── websocket.js
        ├── control.js
        ├── motorTest.js
        └── dashboard.js
```

## 🚀 使用指南

### 启动服务器
```bash
# 方式 1: 直接运行
node server.js

# 方式 2: npm 命令
npm start

# 方式 3: 开发模式（热重载）
npm run dev
```

### 访问前端
```
http://localhost:3000
```

### 添加新功能

#### 添加后端 API
1. 在 `src/DroneServer.js` 的 `setupRoutes()` 中添加路由
2. 在 `src/parser.js` 中添加数据处理逻辑

#### 添加前端功能
1. 在 `public/js/` 中创建新模块
2. 在 `public/js/app.js` 中导入和初始化
3. 在 `public/index.html` 中添加 UI 元素
4. 在 `public/css/style.css` 中添加样式

#### 添加新飞控命令
1. 在 `src/protocol.js` 中定义命令类型
2. 在 `src/parser.js` 中实现编码
3. 在 `public/js/control.js` 中添加发送函数

## 📝 版本信息

- **当前版本**: 2.0 (模块化架构)
- **上一版本**: 1.x (单文件)
- **迁移完成**: 2026年2月2日
- **状态**: ✅ 生产就绪

## 🔄 维护方式

### 后端修改
- 编辑 `src/` 下的相应模块
- 无需重启（可使用 `npm run dev` 热重载）
- 更新后自动应用

### 前端修改
- 编辑 `public/js/` 下的 JavaScript 模块
- 刷新浏览器自动加载最新代码
- 可在浏览器开发者工具查看

### 样式修改
- 编辑 `public/css/style.css`
- 按 F5 刷新浏览器

### HTML 修改
- 编辑 `public/index.html`
- 刷新浏览器查看

## 🔒 文件权限

所有文件建议的权限设置：

| 文件类型   | 权限 | 说明           |
| ---------- | ---- | -------------- |
| .js        | 644  | 代码文件，可读 |
| .json      | 644  | 配置文件       |
| .html/.css | 644  | 前端文件       |
| .md        | 644  | 文档文件       |
| 目录       | 755  | 可执行目录     |

## 📦 备份建议

重要文件备份：
- 定期备份 `src/` 和 `public/` 目录
- 提交到版本控制系统 (git)
- 保存 `package.json` 和 `package-lock.json`

## 🔗 相关资源

- [README.md](README.md) - 快速开始
- [ARCHITECTURE.md](ARCHITECTURE.md) - 架构详解
- [MIGRATION.md](MIGRATION.md) - 迁移指南

---

**最后更新**: 2026年2月2日
