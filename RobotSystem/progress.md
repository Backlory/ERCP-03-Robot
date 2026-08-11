# 审查进度

## 2026-08-11

- 已读取 `codebase-design` 与 `planning-with-files-zh` 技能说明。
- 已确认当前工作区已有历史协议升级计划，本次使用独立审查文件，不覆盖历史记录。
- 已确认本轮约束：不修改 Beckhoff 工程，也不修改任何代码。
- 已完成 `03-Robot` 的 ADS 调用链、320 字节布局、Sum Read、状态快照和写入路径盘点。
- 已完成 Beckhoff DUT、`MAIN` 变量声明、`iMaxAxisNum := 21` 和 ERCP profile 边界盘点。
- 已完成金标准 UTF-8 版本逐项对照。
- 已确认基线静态契约测试通过：`BeckhoffProductionContractRegression.ps1`。
- 已确认 `03-Robot` 和 Beckhoff 工程源代码工作区没有被本轮修改；本工程新增的仅是本审查的三个 Markdown 规划文件。
- 历史结论：曾考虑动态能力发现与 320 字节优化路径；该方向已由用户在本轮修订中明确取消。

## 用户修订后的计划（2026-08-11）

- 用户明确取消：能力发现、金标准 MD 绑定、320 字节快速路径、RobotSystem 启动门控、写入路径改造、多 ADS 设备设计。
- 计划已改为：仅在 `03-Robot` 中使用静态叶子字段表，全部状态通过叶子符号读取。
- 数组兼容优先使用元素级符号读取，避免父结构长度和 C++ 数组 `sizeof`；旧版缺少元素只标记 unavailable。
- 继续复用现有 `ADSIGRP_SUMUP_READ` 和本地 snapshot/error 表达，保持 20 ms 轮询目标与其他模块行为不变。
- 下一阶段才进入代码修改；本轮仍未修改任何源代码或 Beckhoff 工程。
- 根据用户补充，规划文件已从 `01-Cloud-cmake` 移动至 `03-Robot/RobotSystem`；Cloud 工程不再保留副本。
- 路径核对中一次 PowerShell `Get-ChildItem -Name` 数组参数失败，改用文件枚举后完成验证；未造成数据变更。
- 已完成静态叶子读取改造：本体和 ERCP 状态的数组均按元素请求，所有 ADS 读长度使用固定 PLC 标量大小。
- 已删除 320 字节父结构布局、`DecodeRobotFeedbackBlock`、在线父结构校验和双路径回退；`ReadDataBatch` 支持跳过缺失句柄后继续读取有效项。
- 已增加连接内 `ADS_SYMBOL_NOT_FOUND` 负结果缓存，避免旧版缺失数组元素在 20 ms 轮询中重复申请句柄；重连时清空缓存。
- 已同步更新接口探针和静态契约测试；本地旧版叶子缺失模拟验证了成功字段保留、失败字段 unavailable。
- 最终验证通过：静态契约检查、`RobotUdpV2Regression.exe`、`robot_device.dll` 构建和 `BeckhoffInterfaceProbe.exe` 构建。
- `robot_device` 首次构建曾被 vcpkg 用户属性权限阻断，改用显式 `VCLibPackagePath` 绕过该本机环境问题后构建成功；未修改该用户文件。
- 全程未修改 Beckhoff 工程；未改写入路径、UDP 协议或启动门控逻辑。
