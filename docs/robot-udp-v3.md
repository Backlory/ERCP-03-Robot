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

V3 与 V2 的 major version、包长和字段布局都不同。Master 与 Robot 必须同步升级；
V2/V3 混部会被双方的严格解码器拒绝。

## 当前生产 ADS 契约

`260729-开发人员通信协议金标准.md` 是目标接口描述，不是现有生产 TwinCAT
工程已经部署该接口的证据。当前代码唯一可静态验证的生产 ADS 基线是：

`04-simulator/beckhoff-GT/生产线上的beckhoff驱动`

因此 Robot 必须保持以下边界：

- `MAIN.Info_Feedback_ToMaster` 保持 304B；
- `MAIN_ERCP.ERCP_Info_Feedback_ToMaster` 保持 104B 旧布局；
- `DriveErrorState_ERCP` 和 `MotorErrorState_ERCP` 分别按 14、12 个 `BOOL` 读取；
- 公共状态有效性只依赖原有
  `MAIN.Status_Feedback_ToMaster` 和 `MAIN.Info_Feedback_ToMaster`；
- 连续控制仍只写生产快照已有的 `MAIN.Follow_Control_Cmd`（88B）；
- Robot action 只写生产快照已有的 `MAIN.Status_Command_FromMaster`；
- ERCP operate 保留原有 online/ready 状态迁移写入
  `MAIN_ERCP.bERCP_Operate_State_FromMaster`。

以下 V3 网络字段尚无现有生产 PLC 接口证据，当前不得读写 Beckhoff：

- Robot prepare、22 路驱动错误、19 路电机错误；
- ERCP cooperate、6D handle、3 buttons；
- 双注射器速度、位置和使能；
- Balloon pressure、operator position。

这些字段在 V3 状态中暂按零值/未知值上报，在 applied-command 中不得伪报为已经写入。
只有获得生产 TwinCAT 符号表和精确 ABI（符号名、类型、数组下标、结构对齐与总长度），
并新增生产契约回归后，才能逐项启用。

静态门禁：

```powershell
.\tests\BeckhoffProductionContractRegression.ps1
```

该门禁拒绝新增生产快照之外的 ADS 符号，并校验 ERCP feedback 字段布局及错误数组长度。
