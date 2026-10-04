# Robot HTTP 接口

RobotSystem 默认监听 7998 端口，支持 `/local` 路径前缀。JSON 请求使用 `Content-Type: application/json`。

| 路径 | 行为 |
| --- | --- |
| `/robot/info` | 返回机器人名称、版本、位置和模块名称 |
| `/robot/status` | 返回实际运行状态、模块状态和步骤；`running` 为 1 表示运行中，0 表示未运行 |
| `/robot/init` | 执行机器人初始化 |
| `/robot/close` | 执行机器人关闭 |
| `/robot/action` | 执行指定模块动作；请求包含 `type` 和 `action` |
| `/robot/emergency-stop` | 设置专用急停信号；请求包含布尔字段 `active` |
| `/robot/log` | 切换机器人日志记录 |
| `/robot/forcerecord` | 记录力反馈和位置 |
| `/settings` | 返回配置字段定义 |
| `/settings/data` | 返回当前 YAML 配置 |
| `/settings/update` | 更新配置；请求中的 `data` 为 YAML 字符串 |
| `/sensio` | 返回传感器及 GPIO 通道目录 |
| `/beckhoff/isopen` | 查询 Beckhoff 连接状态 |

`/robot/action` 的 `type` 为 `arm`，支持 `suspend`、`resume`、`clear`、`fold`、`open`、`follow`、`exit` 和 `auto`。展开、折叠和跟随通过 ArmModule 调用 Beckhoff 实现。

响应包含 `req`、`uri`、`status`、`info`，有业务数据时包含 `data`。客户端需要检查 JSON 的 `status`；路径不存在和请求校验失败同样使用 HTTP 200 包装错误响应。`/beckhoff/isopen` 在设备离线时返回校验失败。

## HTTP 回归验证

在配置好 MSBuild 的终端中，从 Robot 工程根目录执行：

```powershell
MSBuild.exe robot.sln /p:Configuration=Release /p:Platform=x64 /m
.\build\Release\RobotSystem\HttpApiRegression.exe
```

测试编译生产 HTTP 路由、处理器和设备实现，通过本机临时端口执行 74 次真实请求，检查路径拒绝、机器人信息、通道目录以及请求校验。设备操作路由使用不支持的内容类型，在进入设备调用之前完成校验。
