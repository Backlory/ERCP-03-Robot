# RobotTimingProbe

这是一个不启动真实 Master、也不连接现场 TwinCAT 的 RobotSystem 时序探针。
它在进程内启动最小 fake ADS TCP 服务，并用 Robot UDP V3.1 控制包伪造 Master，实际运行 RobotSystem 的控制线程和 Beckhoff 适配器。

## 构建

```powershell
& 'C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe' `
  '.\tests\RobotTimingProbe\RobotTimingProbe.vcxproj' `
  /m /p:Configuration=Debug /p:Platform=x64 `
  /p:SolutionDir='G:\CPPprojs\260402-ERCP-CLOUD\260717-rethink\03-Robot\' /v:minimal
```

## 运行

必须从本目录启动，因为 `robot_device.dll` 在进入 `main()` 前读取当前目录的 `config.yaml`：

```powershell
Push-Location 'G:\CPPprojs\260402-ERCP-CLOUD\260717-rethink\03-Robot\tests\RobotTimingProbe'
& '..\..\build\Debug\RobotSystem\RobotTimingProbe.exe' `
  --duration-ms 3000 --frequency-hz 125 `
  --ads-delay-ms 0 --ads-delay-scope none `
  --wheel both --pattern step
Pop-Location
```

`--ads-delay-scope` 可以设置为 `none`、`writes`、`reads` 或 `all`。用同一组参数分别运行基线和延迟注入即可观察 ADS 对控制周期的影响。`--csv PATH` 可保存原始观测。

## 重点指标

- `master_tx_interval`：伪 Master 发包周期，用来确认输入源是否稳定。
- `master_rx`：RobotSystem 收到的包、接受数、乱序和丢序统计。
- `follow_write_interval`：RobotSystem 向 `Cmd_Follow_Comp_Joy_FromMaster` 写入的间隔。
- `ads_write_service`：fake ADS 对同步写请求的服务时间。
- `robot_status_interval`：RobotSystem 状态包周期；状态读取与控制写入共享 ADS 锁时会受到影响。
- `command_applied_age`：RobotSystem 从收到控制包到开始 ADS 写入的时间；不包含写请求完成后的网络服务时间。

该工具只用于定位边界：fake ADS 的结果可以证明 RobotSystem/ADS 调用路径对人为延迟的敏感性，不能替代现场 PLC 的实际网络测量。
