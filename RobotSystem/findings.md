# 审查发现

## 当前状态

已完成 Robot ADS 入口与核心布局文件的第一轮盘点。

## 第一轮源代码证据

### Robot 侧已经具备的能力

- `03-Robot/plugins/lib/drivers/src/beckhoff_driver.hpp` 暴露了按符号名读取、批量读取、符号信息查询、符号大小验证和 ADS 原始读写接口。
- `03-Robot/tests/BeckhoffInterfaceProbe/main.cpp` 使用 `ADSIGRP_SYM_INFOBYNAMEEX` 查询在线符号的 index group、offset、size 和类型，然后按在线 size 读取；这证明 ADS 本身可以先查元数据，再按实际字段长度读取，而不是只能猜结构总长度。
- Robot 驱动实现包含 `ADSIGRP_SUMUP_READ` 批量读取路径，当前代码为公共状态和 ERCP 状态分别准备多条 `AdsReadRequest`；因此“一次 ADS 请求携带多个字段”可行，但它仍然是多个固定符号读项的聚合，不是 ADS 自动返回键值对。

### 当前仍存在的 320 字节硬约束

- `03-Robot/include/include/beckhoff_feedback_layout.hpp` 定义 `RobotFeedbackBlockSize = 320`，`RobotFeedbackData` 使用 `static_assert(sizeof(...) == 320)` 和一组固定 offset。
- 同一文件的 `DecodeRobotFeedbackBlock` 只接受 `blockSize == 320`；因此只要完整父结构变成 280 字节，该路径必然拒绝，即使其中部分叶子符号仍然可读。
- 生产契约测试和接口探针仍把 `MAIN.Info_Feedback_ToMaster` 的 320 字节父结构作为当前部署 ABI；这说明 320 是当前这台 PLC 的真实在线结构尺寸，但不能因此推导为 ADS 协议的普遍限制。

### 初步判断

- 320 字节来自 `MAIN.Info_Feedback_ToMaster` 这个 PLC 结构的当前在线布局，而不是 ADS 单次传输的固定上限。
- 目标应是把“ADS 传输/符号元数据”和“Robot 本地 typed snapshot”之间做深的 Adapter seam：块读取可作为已验证的快速路径，叶子符号/批量符号读取作为版本兼容路径。
- 需要继续确认：当前状态线程在何处选择块路径或叶子路径、失败后是否保留旧值、每个字段是否有独立有效性/错误码，以及 Beckhoff DUT 的真实声明顺序和 pack/alignment。

## Robot 状态线程与 ADS 批量机制

- `Beckhoff_Motor::OpenConn` 先打开 ADS 端口、读取 ADS/PLC 状态，然后调用 `ValidateRobotFeedbackLayout()`；只有父结构大小和每个叶子 offset/size 都匹配时，才启用 320 字节块读取。
- `ReadData`/`WriteData` 通过 `ADSIGRP_SYM_HNDBYNAME` 获取符号句柄，再用 `ADSIGRP_SYM_VALBYHND` 读写；句柄错误后会释放并从缓存删除。
- `ReadDataBatch` 把每个请求编码为 `(ADSIGRP_SYM_VALBYHND, handle, length)` 三元组，通过 `ADSIGRP_SUMUP_READ` 一次返回“每项错误码 + 每项数据”；若目标不支持 Sum Read，则退化为逐项 ADS 读。它已经是一次通讯读取多个状态的实现基础。
- `StateUpdateThread` 每 20 ms 轮询。公共状态在块路径下请求 `MAIN.Info_Feedback_ToMaster` 320 字节；否则按叶子请求 `Follow_Length`、3 个 BOOL、轮、传感器数组、功率、lifter、力/角度/跟随力和轴数组。ERCP 状态与 ERCP 反馈分别是 9 项和 11 项 Sum Read。
- 当前聚合结果只保留 `common_ads_error`、`ercp_state_ads_error`、`ercp_feedback_ads_error` 等组级错误码，虽然 Sum Read 已生成 `itemErrors`，但状态快照没有保存每个字段的错误码/在线 size/类型。
- 当前组提交采用全组成功才更新 typed snapshot；失败时标记 group stale，保留上一轮快照内容。这是安全的基础，但还不能区分“字段不存在”“长度变化”“单个可选字段失败”。

## 已观察到的当前版本差异

- 已有在线探针记录显示：当前机器人本体 PLC 的 `MAIN.Info_Feedback_ToMaster` 为 320 字节，`Axes_Pos` 为 21 个 LREAL（168 字节），`MotorErrorState` 为 21 个 BOOL；而金标准/Robot 本地部分声明使用 19 个轴。
- 探针还记录：文档中的 `MAIN.Status_Comand_FromMaster` 不存在，而 `MAIN.Status_Command_FromMaster` 存在；三个本体错误符号以及所有 `MAIN_ERCP.*` 在这台仅本体 PLC 上不存在。
- 这证明“符号存在性、在线长度和设备 profile”比“把协议表直接等同于一个固定父结构”更可靠；也证明不能把 ERCP 小车字段缺失当成本体 Robot ADS 连接整体失败。

## Beckhoff-LingMiProject-无台车的底层证据

- `LingMiProject/LingMiMotorCtrl/DUTs/Info_Feedback_ToMaster.TcDUT` 的 PLC 声明顺序是：`Follow_Length`、3 个 BOOL、`Axes_Pos`、大小拨轮、10 个力传感器、`Power_level`、`lifter`、`Deliver_Force`、`Rotate_Degree`、`Follow_Force`。
- `MAIN.TcPOU` 将 `Info_Feedback_ToMaster` 声明为 `AT%I*` 输入结构，将 `Status_Command_FromMaster`、`type_of_scope` 等写入量声明为 `AT%Q*`；`MotorErrorState` 也是按 `iMaxAxisNum` 的数组。
- 当前 Beckhoff 工程的 `iMaxAxisNum` 由配置/符号信息确定为 21，因此当前结构自然形成 320 字节布局；这不是 TwinCAT/ADS 规定的固定包长，而是本工程 DUT 成员和数组维度共同决定的 ABI。
- 工程源码中未发现 `MAIN_ERCP`、ERCP cart 反馈 DUT 或 ERCP 命令定义；这些符号属于另一类/另一台 PLC profile。Robot 侧把 ERCP 作为可选设备探测，是合理的设备边界处理。
- `MAIN` 的真实 PLC 变量名是 `Status_Command_FromMaster`，与金标准表中疑似 OCR/拼写的 `Status_Comand_FromMaster` 不同；按符号名访问时拼写差异会直接变成 `ADS_SYMBOL_NOT_FOUND`。

## 当前实现的关键缺口

- 叶子回退仍用编译期数组尺寸：`feedback.Axes_Pos` 为 21 个、`mainMotorErrors` 为 19 个。若未来 PLC 数组维度变化，叶子读取仍可能因请求长度不等于在线符号长度而失败；当前只是绕过了父结构 320 校验，尚未实现真正的可变数组兼容。
- `ReadData`/`ReadDataBatch` 本身接收调用方给出的长度，不会自动用 `QuerySymbolInfo` 的在线 size 校正或拒绝；所以动态兼容必须在更高层先建立字段 capability/schema，再按在线 size 解码。
- `ValidateRobotFeedbackLayout` 的固定 offset 校验适合作为“快速块读取是否安全”的判定，但不应作为所有版本的唯一入口；块路径与叶子路径应共享同一套能力模型和字段解码规则。
- 当前 `StateUpdateThread` 只有组级错误码和 stale/valid 位。要满足“字段成功则写本地结构、失败则写错误码”的目标，需要扩展本地快照的字段状态（至少存在性、在线类型/size、ADS 错误、是否 stale、采样时间），并定义必需/可选字段策略。

## 金标准对照

- 金标准是“叶子符号变量表”，列出了本体 `MAIN.*`、本体 `Info_Feedback_ToMaster.*`、可选 `MAIN_ERCP.*` 及其读写方向；它没有规定 PLC `STRUCT` 的总字节数，也没有授权用 Markdown 行顺序推断结构 offset。
- 金标准中的本体 `Axes_Pos` 为 19 路，而只读 Beckhoff 工程配置 `iMaxAxisNum := 21`；因此 Robot 应采用“在线数组长度 + 本地发布容量”的映射策略，而不是把 19 或 21 写死为 ADS 请求长度。
- `Status_Comand_FromMaster` 与 Beckhoff 工程/Robot 当前实际使用的 `Status_Command_FromMaster` 拼写不同。兼容层需要支持显式 alias，并对 alias 的在线类型做验证；不能通过改变结构 offset 解决符号名差异。
- ERCP 字段属于可选设备 profile；本体 PLC 缺少 `MAIN_ERCP` 时，只应将 ERCP 字段标记为 unavailable/stale，不应让本体公共状态失效。

## 写入侧风险

- Robot 的写入目前大多以 C++ `sizeof` 直接发送固定长度，`WriteData` 不先比对在线 symbol size/type。例如 9 路 LREAL、3 路 BOOL 和单个 DINT 在 PLC 版本变化后可能返回 `ADS_INVALID_SIZE`，但本地只获得一个聚合命令错误。
- 急停、运动状态、镜体类型等写入属于高风险字段；动态兼容不能“尽量写入”或自动截断，应在 capability discovery 阶段确认存在、类型和精确长度，运行时 mismatch 时 fail-closed，并保留明确的拒绝原因。
- 当前 `GoldDiscreteCommandResult` 已对 ERCP 命令做差分写入和 online 探测，这是后续接入字段 capability 校验的合适 seam。

## ADS 通讯边界补充

- 当前 `StateUpdateThread` 是 Robot 主动轮询，不是 Beckhoff 主动推送固定 320 字节包；周期约为 20 ms。ADS notification 可以另行设计，但不是解决布局兼容的必要条件。
- `ADSIGRP_SUMUP_READ` 的“一次”只适用于同一个 ADS target（同一 `AmsAddr`/runtime）。如果机器人本体和 ERCP 台车实际位于不同 PLC、AMS Net ID 或端口，必须各自建立连接和批量请求；不能把两个设备强行拼成一个 Sum Read。
- 当前 `Beckhoff_Motor` 只有一个 `m_Addr`，且用同一地址探测 `MAIN_ERCP`；因此未来若 ERCP 台车独立部署，设备 profile/多连接能力必须在改造前确认。

## 基线验证

- `03-Robot/tests/BeckhoffProductionContractRegression.ps1` 当前通过，但该测试明确把 320 字节父结构和块读取当作生产契约；它证明当前版本未偏离现有 ABI，不证明可变长度兼容已经实现。

## 2026-08-11 用户修订后的方案决策

以下内容取代前面关于 capability discovery、金标准运行时绑定、320 字节快速路径、字段级启动门控和写入校验的建议；前面的内容保留为历史审查证据，不再作为实施方案。

- 兼容目标限定为当前版与已知旧版机器人本体，不建立在线能力发现、版本协商或 schema/capability 模型。
- 不把金标准 Markdown 作为运行时/编译时依赖。Robot 侧使用已有代码和已知当前/旧版符号形成静态叶子读取表。
- 不再读取 `MAIN.Info_Feedback_ToMaster` 父结构；320 字节不再是任何状态读取路径的前置条件。
- 叶子读取也不应把 C++ 数组 `sizeof` 当作远端数组请求长度。数组应优先拆成固定标量元素符号读取，例如 `Axes_Pos[i]`、`Force_Sensor[i]`、`MotorErrorState[i]`；旧版缺少的元素单独 unavailable，不影响其他元素。
- 现有 `ADSIGRP_SUMUP_READ` 作为多个叶子读取的主要传输方式；因为只有一个 ADS target，可将固定字段清单组织为一个或少量固定批次。
- 不新增 RobotSystem 内的启动禁止、required/optional 策略或安全门控。单字段失败只标记 unavailable，保持其他模块已有的启动判断。
- 本轮不改写入路径，不把写入的 `sizeof` 问题纳入本次实现范围。
- 不扩展完整本地状态模型；优先复用现有 snapshot、stale/unavailable/error 表达和日志通道。只有现有表示无法区分 unavailable 时，才做最小范围的表达补充。
- 旧版缺失大量数组元素时必须限频记录错误，避免 20 ms 轮询造成日志洪水。

## 新增证据：数组元素符号

- 当前 Beckhoff 工程的 `LingMiProject.tsproj` 明确包含 `MAIN.MotorErrorState[1]` 至 `MAIN.MotorErrorState[21]` 等元素级符号/链接，说明按元素读取是当前工程可行的方向。
- `Axes_Pos`、`Force_Sensor` 等数组元素仍需在具体旧版 PLC 上用现有接口探针确认导出名称；该确认属于测试/现场验收，不是运行时能力发现机制。

## 证据记录规则

- 记录文件路径、符号名、类型/长度、读写方向和调用链。
- 区分“代码直接证明”“由 TwinCAT/ADS 机制推断”和“必须现场符号验证”。
- 不把 UDP wire 布局与 ADS PLC 结构布局混为同一层协议。

## 关键待证问题

1. Robot 是否通过 `AdsSyncReadReq` 读取完整 PLC 结构，还是逐符号读取。
2. 320 字节是某个 ADS 结构的 `sizeof`、请求长度、UDP 状态包长度，还是多者混淆。
3. Robot 当前是否能在读失败后保留旧快照并记录字段级错误。
4. Beckhoff 工程是否暴露了稳定的在线符号名，是否存在结构成员级符号访问。
