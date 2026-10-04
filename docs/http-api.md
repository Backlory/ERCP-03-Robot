# Robot HTTP 接口

RobotSystem 默认监听 7998 端口，支持 `/local` 路径前缀。JSON 请求使用 `Content-Type: application/json`。

| 路径 | 行为 |
| --- | --- |
| `/robot/status` | 返回实际运行状态、模块状态、步骤及任务失败诊断；`running` 为 1 表示运行中，0 表示未运行 |
| `/robot/init` | 执行机器人初始化 |
| `/robot/close` | 执行机器人关闭 |
| `/robot/action` | 执行指定模块动作；请求包含 `type` 和 `action` |
| `/robot/log` | 切换机器人日志记录 |

`/robot/action` 的 `type` 为 `arm`，支持 `suspend`、`resume`、`clear`、`fold`、`open`、`follow`、`exit` 和 `auto`。展开、折叠和跟随通过 ArmModule 调用 Beckhoff 实现。

响应包含 `req`、`uri`、`status`、`info`，有业务数据时包含 `data`。客户端需要检查 JSON 的 `status`；路径不存在和请求校验失败同样使用 HTTP 200 包装错误响应。

`/robot/status` 的 `data.errors.arm` 包含 `active`、`time_unix_ns`（Unix 纳秒）、`task`、`step`、`reason` 和 `report`。`report` 按步骤名称记录执行结果：0 待执行、1 执行中、2 成功、3 返回失败、-1 异常。失败后保留首个未清除的错误，结束任务并废弃该批次的其他请求。Master 周期查询该接口，显示新发生的失败步骤和原因。

`clear` 仅清除软件错误并恢复接受请求；执行中的旧任务尚未结束时返回失败。清除操作不发送 PLC 复位或运动命令，不重放失败任务，也不恢复旧队列。新动作必须由新的显式请求创建，并使用独立的空报告。`resume` 不能清除错误；`exit` 在错误状态仍可请求退出跟随，失败诊断继续保留。`suspend` 终止当前任务批次。

`/robot/info`、`/robot/forcerecord`、`/robot/emergency-stop`、`/settings`、`/settings/data`、`/settings/update`、`/sensio`、`/beckhoff/isopen` 的注册代码带有 TODO 注释，等待确认使用场景；当前 HTTP 路由不开放这些接口。

## HTTP 回归验证

在配置好 MSBuild 的终端中，从 Robot 工程根目录执行：

```powershell
MSBuild.exe robot.sln /p:Configuration=Release /p:Platform=x64 /m
.\build\Release\RobotSystem\HttpApiRegression.exe
```

测试直接执行生产任务执行器，验证失败报告、错误状态、队列失效、停止和清除后的新请求；并通过本机临时端口执行 90 次真实 HTTP 请求，检查路径拒绝及请求校验。设备操作路由使用不支持的内容类型，在进入设备调用之前完成校验。
