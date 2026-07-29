# Robot UDP V3（Beckhoff 金标准）

端口保持不变：

| 端口 | 方向 |
|---:|---|
| 31001 | Robot → Master 状态 |
| 31002 | Master → Robot 命令 |
| 31003 | Robot → Cloud 状态 |
| 31004 | Cloud → Robot 命令 |

公共包头 48B，网络字节序为 big-endian，`version_major=3`。

## 状态包

完整状态包固定 1200B，组大小依次为：

`24 / 312 / 40 / 24 / 80 / 448 / 64 / 24`

ERCP feedback 的 80B PLC 原生布局：

| Offset | 字段 | PLC 类型 |
|---:|---|---|
| 0 | ERCP_Deliver_Force | LREAL |
| 8 | GuideWire_Force | LREAL |
| 16 | Bow_Force | LREAL |
| 24 | ERCP_Deliver_Pos | LREAL |
| 32 | GuideWire_Pos | LREAL |
| 40 | Inject_CurPos_01 | LREAL |
| 48 | Inject_CurPos_02 | LREAL |
| 56 | Inject_State_01 | DINT |
| 60 | Inject_State_02 | DINT |
| 64 | Balloon_Pressure | INT |
| 72 | Operator_Pos | LREAL |

V3 不存在夹紧力、ERCP 跟随力 01/02 和注射器力 01/02。

Robot common 组新增 `iPrepare_State`、Robot 驱动/电机错误汇总与 22/19 路位图，以及
`type_of_scope`。ERCP 错误位图为 13/11 路。

## 命令包

完整命令包固定 224B，payload 176B，包括：

- 原连续 10 路 LREAL 和 6 个开关；
- Robot action（`-1` 不操作，`0..3` 对应待机/折叠/展开/跟随）；
- ERCP operate/cooperate、6D handle、3 buttons；
- 双注射器速度、目标位置和使能。

Robot 在 ERCP offline/not-ready 时将 ERCP 离散命令安全清零。`operate` 只响应显式命令，
不再由 online/ready 状态自动写入。命令 100ms 不新鲜时连续量归零。
