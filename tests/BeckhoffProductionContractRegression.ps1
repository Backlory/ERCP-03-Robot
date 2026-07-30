param(
    [string]$RobotRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$driver = Join-Path $RobotRoot 'plugins\lib\drivers\src\beckhoff_driver.cpp'
$source = Get-Content -LiteralPath $driver -Raw
$layoutHeader = Join-Path $RobotRoot 'include\include\beckhoff_feedback_layout.hpp'
$layout = Get-Content -LiteralPath $layoutHeader -Raw
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text([string]$haystack, [string]$text, [string]$description) {
    if (-not $haystack.Contains($text)) {
        $failures.Add("Missing gold-standard contract: $description ($text)")
    }
}

foreach ($field in @(
    'Follow_Length', 'Switch_Water', 'Switch_Gas', 'Switch_Suck',
    'Big_Whell', 'Small_Whell', 'Force_Sensor', 'Power_level', 'lifter',
    'Deliver_force', 'Rotate_Deqree', 'Follow_Force', 'Axes_Pos')) {
    Require-Text $layout $field "robot feedback ABI field $field"
}

foreach ($field in @(
    'ERCP_Deliver_Force', 'GuideWire_Force', 'Bow_Force',
    'ERCP_Deliver_Pos', 'GuideWire_Pos', 'Inject_CurPos_01',
    'Inject_CurPos_02', 'Inject_State_01', 'Inject_State_02',
    'Balloon_Pressure', 'Operator_Pos')) {
    Require-Text $source $field "ERCP feedback leaf $field"
}

foreach ($symbol in @(
    'Cmd_Follow_Comp_Joy_FromMaster', 'Cmd_Operator_Joy_FromMaster',
    'Cmd_Home_Joy_FromMaster', 'Cmd_IO_Joy_FromMaster',
    'bERCP_Operate_State_FromMaster', 'bErcp_Cooperate_Enable',
    'Cmd_6Dhandle_Joy_FromMaster', 'Cmd_Button_Joy_FromMaster',
    'Inject_Vel_01', 'Inject_Vel_02', 'Inject_Pos_01', 'Inject_Pos_02',
    'Inject_Enable_01', 'Inject_Enable_02')) {
    Require-Text $source $symbol "command leaf $symbol"
}

foreach ($declaration in @('bool driveErrors[13]{}', 'bool motorErrors[11]{}',
        'bool mainMotorErrors[19]{}')) {
    Require-Text $source $declaration "gold error-array length"
}

foreach ($forbidden in @(
    'WriteData("MAIN.Follow_Control_Cmd",',
    'ReadData("MAIN_ERCP.ERCP_Info_Feedback_ToMaster",')) {
    if ($source.Contains($forbidden)) {
        $failures.Add("Raw TwinCAT STRUCT access is forbidden without an explicit ABI: $forbidden")
    }
}

Require-Text $layout 'constexpr std::size_t RobotFeedbackBlockSize = 320;' `
    '320-byte robot feedback block size'
Require-Text $layout 'static_assert(sizeof(RobotFeedbackData) == RobotFeedbackBlockSize' `
    '320-byte robot feedback ABI size guard'
Require-Text $source 'm_common_block_read_enabled = ValidateRobotFeedbackLayout();' `
    'online feedback size-and-offset validation'
Require-Text $source 'constexpr std::array<ExpectedField, 13> fields' `
    'all online feedback field offsets are validated'
Require-Text $source 'm_common_block_read_enabled = false;' `
    'block-read failures safely fall back until reconnect'
Require-Text $source 'addCommon("MAIN.Info_Feedback_ToMaster",' `
    'single 320-byte Common feedback read'
Require-Text $source 'ADSIGRP_SUMUP_READ' 'ADS Sum Read transport'
Require-Text $source 'const std::array<AdsReadRequest, 9> ercpStateRequests' `
    'single ERCP-state Sum Read group'
Require-Text $source 'const std::array<AdsReadRequest, 11> ercpFeedbackRequests' `
    'single ERCP-feedback Sum Read group'

if ($source.Contains('READ_MAIN_FEEDBACK')) {
    $failures.Add('Common feedback must not be expanded into per-leaf ADS requests.')
}

if ($failures.Count -ne 0) {
    Write-Error ($failures -join "`n")
    exit 1
}

Write-Output 'PASS: Beckhoff boundary uses the validated 320-byte Common ABI and exact remaining contracts.'
