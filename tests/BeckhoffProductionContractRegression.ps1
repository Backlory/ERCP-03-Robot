param(
    [string]$RobotRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$driver = Join-Path $RobotRoot 'plugins\lib\drivers\src\beckhoff_driver.cpp'
$source = Get-Content -LiteralPath $driver -Raw
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text([string]$text, [string]$description) {
    if (-not $source.Contains($text)) {
        $failures.Add("Missing gold-standard contract: $description ($text)")
    }
}

foreach ($field in @(
    'Follow_Length', 'Switch_Water', 'Switch_Gas', 'Switch_Suck',
    'Big_Wheel', 'Small_Wheel', 'Force_Sensor', 'Power_level', 'lifter',
    'Deliver_Force', 'Rotate_Degree', 'Follow_Force', 'Axes_Pos')) {
    Require-Text $field "robot feedback leaf $field"
}

foreach ($field in @(
    'ERCP_Deliver_Force', 'GuideWire_Force', 'Bow_Force',
    'ERCP_Deliver_Pos', 'GuideWire_Pos', 'Inject_CurPos_01',
    'Inject_CurPos_02', 'Inject_State_01', 'Inject_State_02',
    'Balloon_Pressure', 'Operator_Pos')) {
    Require-Text $field "ERCP feedback leaf $field"
}

foreach ($symbol in @(
    'Cmd_Follow_Comp_Joy_FromMaster', 'Cmd_Operator_Joy_FromMaster',
    'Cmd_Home_Joy_FromMaster', 'Cmd_IO_Joy_FromMaster',
    'bERCP_Operate_State_FromMaster', 'bErcp_Cooperate_Enable',
    'Cmd_6Dhandle_Joy_FromMaster', 'Cmd_Button_Joy_FromMaster',
    'Inject_Vel_01', 'Inject_Vel_02', 'Inject_Pos_01', 'Inject_Pos_02',
    'Inject_Enable_01', 'Inject_Enable_02')) {
    Require-Text $symbol "command leaf $symbol"
}

foreach ($declaration in @('bool driveErrors[13]{}', 'bool motorErrors[11]{}',
        'bool mainDriveErrors[22]{}', 'bool mainMotorErrors[19]{}')) {
    Require-Text $declaration "gold error-array length"
}

foreach ($forbidden in @(
    'WriteData("MAIN.Follow_Control_Cmd",',
    'ReadData("MAIN.Info_Feedback_ToMaster",',
    'ReadData("MAIN_ERCP.ERCP_Info_Feedback_ToMaster",')) {
    if ($source.Contains($forbidden)) {
        $failures.Add("Raw TwinCAT STRUCT access is forbidden without an explicit ABI: $forbidden")
    }
}

if ($failures.Count -ne 0) {
    Write-Error ($failures -join "`n")
    exit 1
}

Write-Output 'PASS: Beckhoff boundary uses gold-standard leaf symbols and exact array lengths.'
