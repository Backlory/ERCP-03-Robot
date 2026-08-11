# Robot 旧版 ADS 兼容改造计划（修订版）

## 目标

只修改 `03-Robot` 的状态读取适配，使当前 RobotSystem 同时兼容：

- 当前 Beckhoff 机器人本体接口：存在 320 字节 `Info_Feedback_ToMaster` 父结构；
- 旧版机器人本体接口：没有 320 字节父结构，或父结构布局不同。

`BECKHOFF-LingMiProject-无台车` 是只读底层接口，绝不修改。通讯只有一个 ADS target。本轮不改写入路径、不增加启动禁止逻辑、不改变外部 UDP 状态协议。

## 明确排除

- 不建立在线能力发现、版本协商或 schema/capability 模型。
- 不依赖任何 Markdown 协议文档作为运行时或编译时输入。
- 不再读取 `Info_Feedback_ToMaster` 父结构，不再保留 320 字节快速路径。
- 不在 RobotSystem 内判断某个字段是否允许启动；字段失败只标记 unavailable，其他模块继续按既有规则处理。
- 本轮不改 ADS 写入路径；写入兼容另立任务。
- 不引入多 ADS 设备/多连接设计。

## 实施阶段

- [completed] 1. 完成当前 Robot 与 Beckhoff ADS 机制的只读盘点
- [completed] 2. 确认 320 字节来自当前 PLC 结构 ABI，而非 ADS 固定限制
- [completed] 3. 根据用户约束收敛为纯叶子读取方案
- [completed] 4. 建立静态叶子字段读取表，覆盖当前版与已知旧版符号
- [completed] 5. 删除父结构读取、320 字节解码和编译期远端数组长度依赖
- [completed] 6. 用 Sum Read 在一次 ADS 轮询中读取全部状态，并按单项结果更新本地快照
- [completed] 7. 建立旧版/缺字段/单项失败/20 ms 周期回归验证

## 实施结果

- 本体状态全部通过固定标量叶子读取；`Axes_Pos`、`Force_Sensor`、`MotorErrorState` 及 ERCP 错误数组按元素读取。
- 删除 `Info_Feedback_ToMaster` 父结构读取、320 字节 ABI 校验、`DecodeRobotFeedbackBlock` 和父结构回退路径。
- Sum Read 支持同批次部分符号不存在；成功字段继续更新，失败字段清零并通过本地错误/日志表达 unavailable。
- 对连接内稳定的 `ADS_SYMBOL_NOT_FOUND` 做负结果缓存，避免旧数组缺失元素在每个 20 ms 周期重复申请句柄。
- 写入路径、UDP 固定协议、Beckhoff 工程和 RobotSystem 启动判断未改动。

## 设计要点

### 1. 一个深的 Robot ADS 读取模块

在 `Beckhoff_Motor` 内部建立一个固定的叶子读取模块：调用方只看到已有的 `BeckhoffSnapshot`，不感知 PLC 父结构、offset 或 320 字节布局。

每个读取项由代码静态定义：

- 逻辑字段；
- 当前版或已知旧版的符号名；
- 固定的 PLC 标量类型和字节数；
- 本地写入位置；
- 字段失败时的 unavailable 处理。

这不是在线能力发现，而是 Robot 已知接口的静态读取清单。

### 2. 所有状态只读取叶子符号

- 标量直接读取 `LREAL`、`DINT`、`INT`、`BOOL` 叶子符号。
- 数组不再以整体数组长度读取；优先按元素读取，例如 `Axes_Pos[1]`、`Axes_Pos[2]`、`Force_Sensor[1]`、`MotorErrorState[1]`。
- 元素读取使用固定标量长度，例如 LREAL=8、DINT=4、INT=2、BOOL=1，不能使用 C++ 数组的 `sizeof` 作为远端请求长度。
- 旧版少于当前版元素数量时，超出范围的单项读取失败并标记 unavailable；已成功读取的元素仍然更新。
- 新版多出的元素按 RobotSystem 现有本地容量处理，不改变既有上层接口。

### 3. 一次 Sum Read 读取多个叶子

同一 ADS target 下，将固定读取项组成一个或少量固定批次，通过现有 `ADSIGRP_SUMUP_READ` 完成轮询。每个批次返回的 item error 单独对应到字段：

- 成功项更新本地字段；
- 失败项不覆盖其他成功项；
- 失败项仅标记 unavailable，并沿用现有错误/日志通道；
- 不因单个字段失败阻止 RobotSystem 启动或改变其他模块的启动判断。

批次结构只为满足 20 ms 性能和代码可读性，不再以父结构大小为依据。

### 4. 删除 320 字节路径

后续实现应移除或不再使用：

- `RobotFeedbackBlockSize`；
- `RobotFeedbackData` 的 320 字节 `static_assert`；
- `DecodeRobotFeedbackBlock`；
- `ValidateRobotFeedbackLayout`；
- `m_common_block_read_enabled`；
- `feedbackBlock` 父结构请求。

同时删除所有“父结构失败后再回退叶子”的双路径逻辑，叶子读取直接成为唯一状态读取路径。

### 5. 本地行为保持克制

- 不新增 required/optional 启动门控。
- 不改变 `BeckhoffSnapshot` 的业务字段和现有对外接口，除非为了表达 unavailable 确有必要。
- 不把 ADS 读取失败转换成“有效的 0”；失败只走现有 unavailable/stale/error 表达方式。
- 不修改 UDP V3 的固定包布局；本轮关注 ADS 到本地快照这一层。

## 测试与验收

需要覆盖：

1. 当前版 320 字节父结构存在，但 Robot 不读取父结构，所有叶子值正确更新。
2. 旧版没有父结构，但叶子符号存在，状态仍可更新。
3. 旧版数组元素数量少于当前版，已有元素成功、超出元素 unavailable。
4. 单个叶子符号不存在或读取失败时，其他字段仍更新，不触发启动禁止。
5. 当前版/旧版符号名差异使用已知静态别名处理，不进行在线 schema 探测。
6. Sum Read 不支持时，沿用现有逐项读取 fallback，并验证 20 ms 周期行为。
7. 连续运行时错误日志限频，避免旧版缺失数组元素造成日志洪水。
8. 现场使用当前版和旧版 PLC 分别验证状态值、unavailable 表达和周期耗时。

## 回滚

改造只集中在 `03-Robot` ADS 状态读取模块及其测试。若现场性能或符号元素访问不满足预期，回滚 Robot 侧提交即可；不触碰 Beckhoff 工程。
