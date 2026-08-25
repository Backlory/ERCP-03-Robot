# Robot UDP V3 与 Beckhoff 兼容边界

## 三端通讯端口总览

当前三端（Master / Cloud / Robot）之间共存的全部网络通道如下。除 Robot UDP V3
（31001~31004）外，其余通道**不属于 V3 契约**，修改它们不需要动 V3 的黄金 fixture。

| 端口 | 方向 | 传输 | 协议/内容 | 归属 |
|---:|---|---|---|---|
| 8990 | Master → Cloud | TCP/HTTP | JSON 控制面：模块/连接/属性/频率/流订阅（Cloud 侧 `01-Cloud-cmake/src/Server/server.cpp`） | Cloud 控制面，非 V3 |
| 12001~12100 | Cloud → Master | UDP | SplitPacket 分片媒体流：VideoFrame(H.264) / MatFrame(FloatMatrix)；Master 经 `/port/require` 订阅，Cloud `Transmit` 推流 | 媒体流，非 V3 |
| 31001 | Robot → Master | UDP | V3 状态（1200B 完整状态包） | V3 状态通道 |
| 31002 | Master → Robot | UDP | V3 命令（224B 命令包）；Robot 侧对非回环地址按配置 `basic.master` 的 IP 过滤对端（M4，`YunSBot.ControlChannel.cpp`） | V3 命令通道 |
| 31003 | Robot → Cloud | UDP | V3 状态；**仅 loopback**：Cloud 与 Robot 同机部署，Cloud `robot_status_receiver` 绑 127.0.0.1 | V3 状态通道 |
| 31004 | Cloud → Robot | UDP | V3 命令；**仅 loopback**：Robot `situaware` 通道绑 127.0.0.1 | V3 命令通道 |
| 7998 | Master/其它 → Robot | TCP/HTTP | JSON 控制接口（`RobotSystem/Net/server.*`，Robot 进程入口 `RobotSystem/main.cpp` 承载） | Robot 控制 HTTP，非 V3 |
| 14001 / 14002 | Cloud ↔ 外部进程 | UDP | 回环媒体流（同款 VideoFrame/MatFrame 信封），Cloud 本地 `ExternalProcModule` 专用 | Cloud 本地，非三端协议 |
| 48898 | Robot → PLC | TCP/ADS | Beckhoff ADS over TCP（AMS runtime 端口 851），即下文"当前生产 ADS 契约" | 设备侧，非 V3 |

部署约束：Cloud 与 Robot **必须同机部署**，31003/31004 强制走 127.0.0.1 回环，不可拆机；
Master 与 Robot 之间的 31001/31002 为跨机通道，31002 接收端按配置的 Master IP 做对端过滤。

## 网络协议

端口保持不变：

| 端口 | 方向 |
|---:|---|
| 31001 | Robot → Master 状态 |
| 31002 | Master → Robot 命令 |
| 31003 | Robot → Cloud 状态 |
| 31004 | Cloud → Robot 命令 |

公共包头为 48B，网络字节序为 big-endian，当前协议版本为 `3.1`（`version_major=3`、
`version_minor=1`）。

V3 完整状态包固定为 1200B，各组大小依次为：

`24 / 312 / 40 / 24 / 80 / 448 / 64 / 24`

V3.1 完整命令包固定为 224B，payload 为 176B。它可以承载：

- 原连续 10 路 `LREAL` 和 6 个开关；
- Robot action（`-1` 不操作，`0..3` 对应待机/折叠/展开/跟随）；
- ERCP operate/cooperate、6D handle、3 buttons；
- 双注射器速度、目标位置和使能；
- payload 偏移 170 的 `uint16 emergency_stop`（完整包偏移 218），取值只能是 0/1；
  只有 Master 来源允许置 1，偏移 172~175 必须为零。

V3.1 只接受 major version=3、minor version=1、固定包长和固定字段布局；任何版本、长度或字段布局
不一致的输入都会被严格解码器拒绝。

### 急停来源合并

- 面板急停由 Master 作为普通 31002 控制包的 `emergency_stop=1` 发送，解除时由新鲜
  Master 包显式发送 `emergency_stop=0`。
- Robot 独立检查 Master 通道的急停字段，不把它交给自动模式下的 Cloud/Master 普通运动源仲裁；
  Cloud 包的急停字段必须为零，不能断言或解除 Master 急停。
- Robot 将 Master UDP 急停与兼容 HTTP `/robot/emergency-stop` 请求合并；只有两个来源都释放后
  才向 `MAIN.Emergency_Stop_FromMaster` 写入 0。
- Master 包过期、UDP 丢包、Cloud 命令切换和普通零命令都不会自动解除已断言的急停。

### 命令保活、过期与源仲裁契约

V3 命令链路的在线与安全行为由三端共同保证：

- **Cloud 命令发送（`01-Cloud-cmake/nodes/robot_command`）**：收到首个合法命令前不主动
  发送；此后按 ≥20Hz 保活——距上次发送超 50ms 重发上一命令，真实输入超 200ms 未刷新
  则改发安全零命令。命令包固定 `source=Cloud`。
- **Robot 命令接收（`robot_udp_v3_runtime.hpp` 的 `CommandReceiver`）**：命令接收时间
  超过 100ms 视为过期，控制周期按零命令下发（`Motions.Cycles.cpp`）。
- **控制源仲裁**：手动模式固定使用 Master；自动模式下 Master 命令在 200ms 优先窗口内
  可覆盖 Cloud 命令（开关 `basic.master_priority`，见 `control_cycle_policy.hpp` /
  `Motions.Cycles.cpp`），`active_source` 如实回显当前生效来源。
- **方向契约**：只有 Robot 能发布状态（`source=Robot`），只有 Master/Cloud 能发命令
  （`source=Master/Cloud`）；`Test(255)` 不作为任何包来源被接受。状态包内的
  active-source / applied-command source 属于 Robot 的事实回显，允许 `{0,1,2,3,255}`。

### 三端实现与权威源

wire 定义唯一权威源在仓库根 `shared-wire/robot_udp_v3.hpp`（配套
`schema/robot_udp_v3.yaml` 金标准 schema 与 `golden/*.hex` 黄金字节 fixture）。三端持有
逐字节相同的 C++ 副本，由 `sync.bat` 整文件分发，当前 `SYNC-VERSION=8`；C# 侧
`02-Master/ErcpApp/Interface/RobotUdpV3.cs` 无法与 C++ 共文件，手工对齐后由三端黄金测试
锁定字节语义（`robot_control_224.hex` / `robot_status_1200.hex`）。

任何 V3 修改必须依次执行：改权威源并同步 `SYNC-VERSION` → 重生成 golden →
`sync.bat` 分发副本 → 三端黄金测试全绿（详见 `shared-wire/README.md`）。

## 当前生产 ADS 契约

`260729-开发人员通信协议金标准.md` 是目标接口描述，不是现有生产 TwinCAT
工程已经部署该接口的证据。当前代码唯一可静态验证的生产 ADS 基线是：

生产 ADS 符号表和 ABI 由 Robot 部署环境提供；当前仓库只保留静态协议与边界验证，
不依赖已删除的 simulator 工程。

因此 Robot 必须保持以下边界：

- Robot 对机器人本体和 ERCP 台车均按金标准的叶子符号读写，不以 Markdown
  行顺序推断 TwinCAT `STRUCT` 的声明顺序、对齐或总长度；
- `Axes_Pos` 按金标准读取 21 路，V3 固定网络布局发送前 19 路；
- `DriveErrorState`/`MotorErrorState` 分别按 22/19 个 `BOOL` 读取，
  `DriveErrorState_ERCP`/`MotorErrorState_ERCP` 分别按 13/11 个 `BOOL` 读取；
- 公共状态有效性只依赖原有
  `MAIN.Status_Feedback_ToMaster` 和 `MAIN.Info_Feedback_ToMaster`；
- 连续控制逐项写
  `Cmd_Follow_Comp_Joy_FromMaster`、`Cmd_Operator_Joy_FromMaster[9]`、
  `Cmd_Home_Joy_FromMaster[3]` 和 `Cmd_IO_Joy_FromMaster[3]`；
- Robot action 只写生产快照已有的 `MAIN.Status_Command_FromMaster`；
- 金标准不定义旧版 `MAIN.IEncoder`/`MAIN.ISensor`，Robot 不得查询这些符号。
  为保持 V3 的 8 组/1200B 线格式，第 3 组改为固定全零的保留组；
- `MAIN_ERCP.*` 属于可选台车。Robot 以
  `POU_Ercp_CycleExecute.Ercp_Ready_State` 叶子符号探测接口；读取成功代表接口存在，
  BOOL 值仅表示当前是否 ready。接口不可用时清除 ERCP 有效组且不影响机器人本体；
  每秒只读重探测，支持晚接入；
- ERCP 连续量只受网络命令新鲜度影响，即使台车 offline 或未 ready 也按原值写入
  `MAIN.Follow_Control_Cmd`；operate/cooperate、6D handle、3 buttons 和双注射器
  离散命令仍仅在台车 online 且 ready 时写入，否则实际写入/回显为零；
- 注射状态为完成（11）时，相应注射使能强制为 `FALSE`。
- Balloon pressure、operator position。

这些字段在 V3 状态中暂按零值/未知值上报，在 applied-command 中不得伪报为已经写入。
只有获得生产 TwinCAT 符号表和精确 ABI（符号名、类型、数组下标、结构对齐与总长度），
并新增生产契约回归后，才能逐项启用。

静态门禁：

```powershell
.\tests\BeckhoffProductionContractRegression.ps1
```

该门禁拒绝新增生产快照之外的 ADS 符号，并校验 ERCP feedback 字段布局及错误数组长度。
