# Robot UDP V3 与 Beckhoff 兼容边界

端口保持不变：

| 端口 | 方向 |
|---:|---|
| 31001 | Robot → Master 状态 |
| 31002 | Master → Robot 命令 |
| 31003 | Robot → Cloud 状态 |
| 31004 | Cloud → Robot 命令 |

公共包头为 48B，网络字节序为 big-endian，`version_major=3`。

## 网络协议

V3 完整状态包固定为 1200B，各组大小依次为：

`24 / 312 / 40 / 24 / 80 / 448 / 64 / 24`

V3 完整命令包固定为 224B，payload 为 176B。它可以承载：

- 原连续 10 路 `LREAL` 和 6 个开关；
- Robot action（`-1` 不操作，`0..3` 对应待机/折叠/展开/跟随）；
- ERCP operate/cooperate、6D handle、3 buttons；
- 双注射器速度、目标位置和使能。

V3 只接受 major version=3、固定包长和固定字段布局；任何版本、长度或字段布局
不一致的输入都会被严格解码器拒绝。

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
  `ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Force` 叶子符号探测接口，
  不可用时清除 ERCP 有效组且不影响机器人本体；每秒只读重探测，支持晚接入；
- ERCP 连续量、operate/cooperate、6D handle、3 buttons 和双注射器命令
  仅在网络命令新鲜、台车 online 且 ready 时写入，否则实际写入/回显均为零；
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
