# ERCP Robot UDP V2 线协议

状态：V2.0 wire contract。本文只定义 UDP 数据报中的字节，不定义任何 C++/C# 内存布局。

## 1. 传输边界

| 端口 | 方向 | `message_type` | 地址 |
|---:|---|---:|---|
| 31001 | Robot -> Master | 2 (`RobotStatus`) | 现有 Master LAN 地址 |
| 31002 | Master -> Robot | 1 (`RobotControl`) | 现有 Robot LAN 地址 |
| 31003 | Robot -> Cloud | 2 (`RobotStatus`) | `127.0.0.1` |
| 31004 | Cloud -> Robot | 1 (`RobotControl`) | `127.0.0.1` |

- 一个消息必须完整放入一个 UDP 数据报，不允许 SplitPacket。
- 数据报硬上限是 1200 字节。V2.0 全量状态包固定为 1024 字节。
- UDP 发送线程只能序列化 Robot 已发布的不可变 Beckhoff 快照，不得发起 ADS 读取。
- 10+6 连续控制字不包含折叠、展开、跟随等离散动作；这些动作继续使用现有 action/ADS
  状态机，直到另行定义新的 message type。

## 2. 基本编码规则

- 所有整数均为定宽整数，按网络字节序（big-endian）编码。
- 所有 `binary64` 均为 IEEE-754 double；将其 64 位 bit pattern 按网络字节序编码。
- `bool` 不上线；单个逻辑量使用位图，必要时使用 `uint8`。
- 发送端逐字段写入字节缓冲区，接收端逐字段读取并检查边界。禁止发送 native struct、
  `memcpy(struct)`、`Marshal.StructureToPtr`、`#pragma pack` 或依赖 padding。
- 所有命名为 `reserved` 或 `extension` 的字节在 V2.0 必须由发送端置零；接收端若发现
  非零值必须拒绝，防止未协商语义被静默接受。
- 当前 PLC 工程尚未随代码提供。反馈量按 PLC native engineering unit 原样传递；字段单位和
  合法范围必须在 TwinCAT schema 到位后补充，不能从 C++ 类型或历史 UI 猜测。

## 3. 公共包头

Magic 为 ASCII `ERCP`，数值 `0x45524350`。版本为 `2.0`。公共头固定 48 字节。

| Offset | Size | Wire type | 字段 | 约束 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32` | `magic` | `0x45524350` |
| 4 | 2 | `uint16` | `version_major` | 2 |
| 6 | 2 | `uint16` | `version_minor` | 0 |
| 8 | 2 | `uint16` | `header_size` | 48 |
| 10 | 2 | `uint16 enum` | `message_type` | Control=1, Status=2 |
| 12 | 2 | `uint16 enum` | `source` | Robot=1, Master=2, Cloud=3, Test=255 |
| 14 | 2 | `uint16 bitmask` | `flags` | V2.0 必须为 0 |
| 16 | 4 | `uint32` | `payload_size` | 必须等于数据报长度减 48 |
| 20 | 8 | `uint64` | `session_id` | 进程启动时生成的非零随机数 |
| 28 | 8 | `uint64` | `sequence` | 每 `(source,message_type,session_id)` 从 0 单调递增 |
| 36 | 8 | `uint64` | `sent_at_unix_ns` | 消息产生时 UTC Unix ns |
| 44 | 4 | `uint32` | `reserved` | 必须为 0 |

合法来源：Control 只接受 Master、Cloud 或 Test；Status 只接受 Robot 或 Test。生产端口不得
接受 Test 来源。端点必须在 codec 校验后再检查端口方向要求。

发送进程重启时必须生成新的 `session_id` 并将 sequence 归零。接收端按
`(source,message_type,session_id)` 统计 duplicate、out-of-order 和 gap；新 session 不计为丢包。
sequence 按无符号 64 位递增，V2.0 不允许在同一 session 内回绕。

## 4. RobotControl

Control payload 固定 104 字节，完整数据报固定 152 字节。10 个数值必须全部是 finite；
NaN 和正负 Inf 均拒绝。V2.0 暂不在 codec 中猜测量程，Robot 在来源仲裁后负责按设备安全
策略做 sanitize/clamp。过期判断只使用 Robot monotonic clock，不使用包内 Unix 时间。

| Payload offset | Size | Wire type | 字段 |
|---:|---:|---|---|
| 0 | 8 | `binary64` | `follow_compensation` |
| 8 | 8 | `binary64` | `scope_move` |
| 16 | 8 | `binary64` | `scope_rotate` |
| 24 | 8 | `binary64` | `scope_bend_lr` |
| 32 | 8 | `binary64` | `scope_bend_ud` |
| 40 | 8 | `binary64` | `pincer` |
| 48 | 8 | `binary64` | `cutter_feed` |
| 56 | 8 | `binary64` | `cutter_rotate` |
| 64 | 8 | `binary64` | `cutter_bend` |
| 72 | 8 | `binary64` | `guide_wire_feed` |
| 80 | 2 | `uint16 bitmask` | `switches` |
| 82 | 6 | bytes | `reserved`，必须全 0 |
| 88 | 16 | bytes | `extension`，必须全 0 |

`switches` bit：0 home rotate；1 home bend LR；2 home bend UD；3 water；4 gas；5 suction。
bit 6..15 必须为 0。

## 5. RobotStatus 容器

Status payload 先放 8 字节目录，然后顺序放置 length-delimited groups。

| Payload offset | Size | Wire type | 字段 |
|---:|---:|---|---|
| 0 | 2 | `uint16` | `group_count` |
| 2 | 2 | `uint16` | `schema_flags`，V2.0 必须为 0 |
| 4 | 4 | `uint32` | `reserved`，必须为 0 |

每个 group header 固定 16 字节：`group_id:uint16`、`group_version:uint16`、
`group_size:uint32`、`sampled_at_unix_ns:uint64`。未知 group 按 `group_size` 跳过；已知 group 的
版本或长度不匹配、同一已知 group 重复、越界、group 数与实际不符时拒绝整个包。

V2.0 全量状态按 ID 1..8 顺序编码。业务 payload 总计 840 字节，8 个 group header 128 字节，
目录 8 字节，公共头 48 字节，合计 **1024 字节**。

这份预算已经包含“两份完整 applied command + 32 字节扩展”，因此同时满足目标 1024 和
1200 字节硬门禁，不需要删除状态事实或改为分包。

### 5.1 RobotRuntimeGroup (ID=1, version=1, size=24)

| Offset | Type | 字段 |
|---:|---|---|
| 0 | `uint8 enum` | lifecycle：Unknown=0, Stopped=1, Starting=2, Running=3, Stopping=4, Fault=5 |
| 1 | `uint8 enum` | mode：Unknown=0, Manual=1, Automatic=2 |
| 2 | `uint16 enum` | active control source；None=0 或公共头 Source |
| 4 | `uint32 bitmask` | bit0 Beckhoff connected；1 logging；2 command fresh；3 emergency stop |
| 8 | `uint64` | lifecycle changed Unix ns |
| 16 | `uint64` | accepted command received Unix ns；无命令为 0 |

PLC move state 不在这里重复，它只在 BeckhoffCommonGroup 中出现。

### 5.2 BeckhoffCommonGroup (ID=2, version=1, size=296)

| Offset | Type | 字段 |
|---:|---|---|
| 0 | `uint32 enum/raw` | PLC move state；未知值保留 raw |
| 4 | `uint16 bitmask` | 实际输出：bit0 water, bit1 gas, bit2 suction |
| 6 | `int16` | power level，PLC native unit |
| 8 | `binary64[36]` | 下表顺序 |

36 个值依次为：follow length、big wheel、small wheel、force sensor 0..9、lifter、deliver force、
rotate degree、follow force、axis position 0..18。全部为 PLC native engineering unit。

PLC move state 当前具名值：Initial=0、Folding=10、Folded=11、Opening=20、Opened=21、
Following=30、Followed=31、OneClickFollowing=40、OneClickFollowed=41。线类型始终是 `uint32`；
接收端不得因为值不在当前名称表中而拒绝或改写它。

### 5.3 RawIoGroup (ID=3, version=1, size=40)

Offset 0 为 `int32 IEncoder[5]`，offset 20 为 `int16 ISensor[7]`，offset 34 的 6 个 reserved
字节必须为 0。

### 5.4 ErcpStateGroup (ID=4, version=1, size=24)

| Offset | Type | 字段 |
|---:|---|---|
| 0 | `uint16 bitmask` | bit0 online；bit1 ready；bit2 load direction；逐路错误仅由下方错误位图表达 |
| 2 | `uint16` | reserved，必须为 0 |
| 4 | `uint16 bitmask` | 14 路 drive error，bit 0..13 |
| 6 | `uint16 bitmask` | 12 路 motor error，bit 0..11 |
| 8 | `int32 enum/raw` | ERCP type |
| 12 | `int32 enum/raw` | ERCP move status |
| 16 | 8 bytes | reserved，必须为 0 |

ERCP type 当前具名值：Cutter=0、Basket=1、Balloon=2、DrainageTube=3。ERCP move status 当前
具名值：Idle=0、Folding=10、Folded=11、Opening=20、Opened=21、Following=30、Followed=31、
Loading=40、Loaded=41、Exchanging=50、Exchanged=51。两者线类型始终是 `int32`。
这些名称仍需用 TwinCAT 权威 schema 确认；未知底层值必须无损保留，消费者应显示 Unknown 及
raw value，不得拒绝未知值，也不得从多个重叠 bool 反推状态。

### 5.5 ErcpFeedbackGroup (ID=5, version=1, size=104)

offset 0 的 12 个 binary64 依次为：deliver force、guide-wire force、clamp force、follow force
01/02、bow force、inject force 01/02、deliver pos、guide-wire pos、inject current pos 01/02。
offset 96、100 为 `int32 inject_state_01/02`。

Injector state 当前具名值：Idle=0、Injecting=10、Completed=11。线类型始终是 `int32`；名称仍需
用 TwinCAT 权威 schema 确认，未知底层值必须无损保留。

### 5.6 AppliedCommandGroup (ID=6, version=1, size=256)

包含两个 128 字节 record：offset 0 为 `latest_write_attempt`，offset 128 为
`last_successful_write`。失败尝试绝不能覆盖后一条记录。

每个 record：offset 0 为 10 个 Control binary64；offset 80 为 switches；offset 82 为 source；
offset 84 为 result；offset 86 两字节 reserved；offset 88 为 command session；offset 96 为
command sequence；offset 104 为 received Unix ns；offset 112 为 write attempt/success Unix ns；
offset 120 为 `uint32 ADS error`；offset 124 四字节 reserved。result：None=0, Attempted=1,
Succeeded=2, Failed=3, Rejected=4, TimedOutToZero=5。

这里的 command 是 Robot 完成来源仲裁、freshness 检查和 sanitize 后实际尝试写入
`MAIN.Follow_Control_Cmd` 的 10+6，而不是收到的原始网络对象。

### 5.7 AdsDiagnosticsGroup (ID=7, version=1, size=64)

| Offset | Type | 字段 |
|---:|---|---|
| 0 | `uint64` | snapshot sequence |
| 8 | `uint64` | poll start Unix ns |
| 16 | `uint64` | poll complete Unix ns |
| 24 | `uint64` | snapshot publish Unix ns |
| 32 | `uint8 enum` | Disconnected=0, Connecting=1, Running=2, Degraded=3 |
| 33 | `uint8 bitmask` | valid：common/raw IO/ERCP state/ERCP feedback bits 0..3 |
| 34 | `uint8 bitmask` | stale：同上 |
| 35 | `uint8` | reserved，必须为 0 |
| 36 | `uint32` | consecutive failed polls |
| 40 | `uint32` | overall ADS error |
| 44 | `uint32` | common ADS error |
| 48 | `uint32` | raw IO ADS error |
| 52 | `uint32` | ERCP state ADS error |
| 56 | `uint32` | ERCP feedback ADS error |
| 60 | `uint32` | latest command write ADS error |

状态年龄由接收时间或公共头时间减 publish time 得到，不重复上线。

### 5.8 ExtensionGroup (ID=8, version=1, size=32)

32 字节必须全 0。后续优先增加新 group 或提升 group version，不复用已有字段的 reserved offset。

## 6. 拒绝和兼容规则

- 长度小于 48、超过 1200、magic/version/header size/message/source/flags 不合法、声明长度与
  数据报不符、session 为 0、reserved 非零时拒绝。
- magic 等于 `ERCP` 的畸形包属于 V2 错包，绝不能回退按 V1 native struct 解释。
- Control 长度必须恰好 152；未知 switch bit、NaN/Inf、reserved 非零时拒绝。
- Status 可跳过未知 group，但整个数据报仍受 1200 上限和严格边界检查；已知 group 版本必须
  精确匹配。V2.0 编码器始终发出 8 个已知 group。
- major 不同直接拒绝。minor 大于接收端版本时 V2.0 直接拒绝；未来只有在兼容矩阵明确允许
  后才能放宽。
- 接收新 session 时清空该来源旧 session 的 gap 状态，并将已切换出的 session 标记为 retired。
  延迟到达的 retired session 直接拒绝，不能把 active session 切回旧进程；相同 sequence 是
  duplicate；更小是 out-of-order；更大且差值大于 1 时 gap 增加 `delta-1`。
- 本机 command freshness 使用 `steady_clock`。Unix 时间只用于跨端日志关联，系统时钟回拨
  不得使旧命令重新变 fresh。

## 7. 迁移和回滚

| Robot 接收 | Robot 状态发送 | 用途 |
|---|---|---|
| V1 + V2 严格 magic 分流 | 按目的端 V1/V2 | 迁移期 |
| V2 only | V2 only | soak 与现场验收后最终态 |

当前软件状态：Robot、Master、Cloud 的正常网络路径均已使用 V2-only；31003/31004 固定为
`127.0.0.1`，控制命令路径不再使用 SplitPacket/Boost archive，13005、`YunSBot.Visual.Local.cpp`
和 V1 网络结构也已删除。通用 SplitPacket 仍服务于视频/矩阵数据，不能将其作为控制协议使用。

原计划要求先双栈、完成现场门禁后再物理删除；实际实现已提前删除旧路径，因此在 8 小时 loopback、
2 小时 Master LAN、PLC/TwinCAT 字段核对、真机动作、真机故障注入和 V1/V2 同期语义对照完成前，
不得把当前状态称为最终生产发布，也不存在可直接启用的 V1 软件回滚。
