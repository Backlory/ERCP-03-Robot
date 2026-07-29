# Beckhoff interface probe

This console program audits the PLC symbols listed in
`260729-开发人员通信协议金标准.md` without starting RobotSystem.

Default mode is read-only:

```powershell
BeckhoffInterfaceProbe.exe --net-id 169.254.213.62.1.1 --port 851
```

Safe zero-write mode writes only continuous controls, buttons, and enable flags:

```powershell
BeckhoffInterfaceProbe.exe --net-id 169.254.213.62.1.1 --port 851 --write-safe-zero
```

The following action-like symbols are intentionally never written:

- `MAIN.Status_Comand_FromMaster` / `MAIN.Status_Command_FromMaster`
- `MAIN.Emergency_Stop_FromMaster`
- `MAIN.type_of_scope`

For each symbol the probe reports online ADS type and byte size, expected type and
size, read result, and (when enabled) zero-write result. Exit code is non-zero when
any required symbol is absent, has a size mismatch, cannot be read, or a requested
safe zero write fails.
