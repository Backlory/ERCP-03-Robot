param(
    [string]$RobotRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$driver = Join-Path $RobotRoot 'plugins\lib\drivers\src\beckhoff_driver.cpp'
$source = Get-Content -LiteralPath $driver -Raw -Encoding UTF8
$layoutHeader = Join-Path $RobotRoot 'include\include\beckhoff_feedback_layout.hpp'
$layout = Get-Content -LiteralPath $layoutHeader -Raw -Encoding UTF8
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text([string]$haystack, [string]$text, [string]$description) {
    if (-not $haystack.Contains($text)) {
        $failures.Add("Missing leaf-read contract: $description ($text)")
    }
}

foreach ($field in @(
    'Follow_Length', 'Switch_Water', 'Switch_Gas', 'Switch_Suck',
    'Big_Wheel', 'Small_Wheel', 'Force_Sensor', 'Power_level', 'lifter',
    'Deliver_Force', 'Rotate_Degree', 'Follow_Force', 'Axes_Pos')) {
    Require-Text $source $field "robot feedback leaf $field"
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

Require-Text $layout 'struct RobotFeedbackLeaves' 'local leaf value model'
Require-Text $layout 'using RobotFeedbackLeafErrors' 'per-leaf unavailable errors'
Require-Text $layout 'ApplyRobotFeedback(const RobotFeedbackLeaves' 'partial leaf application'
Require-Text $layout 'kRobotPublishedAxisCount = 19' 'local 19-axis publication capacity'
Require-Text $layout 'kAdsLrealBytes = 8' 'fixed ADS LREAL request size'
Require-Text $source 'MakeIndexedSymbolNames' 'element-symbol name construction'
Require-Text $source 'MAIN.MotorErrorState[' 'main motor error element symbols'
Require-Text $source 'MAIN.Info_Feedback_ToMaster.Axes_Pos[' 'axis element symbols'
Require-Text $source 'MAIN.Info_Feedback_ToMaster.Force_Sensor[' 'force sensor element symbols'
Require-Text $source 'ADSIGRP_SUMUP_READ' 'ADS Sum Read transport'
Require-Text $source 'std::array<AdsReadRequest, kErcpStateRequestCount>' `
    'ERCP state element Sum Read group'
Require-Text $source 'const std::array<AdsReadRequest, 11> ercpFeedbackRequests' `
    'ERCP feedback Sum Read group'
Require-Text $source 'std::vector<std::size_t> validIndices' `
    'partial Sum Read when one symbol is absent'

foreach ($forbidden in @(
    'RobotFeedbackBlockSize', 'RobotFeedbackData', 'DecodeRobotFeedbackBlock',
    'ValidateRobotFeedbackLayout', 'm_common_block_read_enabled',
    'feedbackBlock', 'addCommon("MAIN.Info_Feedback_ToMaster",',
    'sizeof(mainMotorErrors)', 'sizeof(feedback.', 'sizeof(driveErrors)',
    'sizeof(motorErrors)')) {
    if ($source.Contains($forbidden) -or $layout.Contains($forbidden)) {
        $failures.Add("320-byte parent/remote-array dependency remains: $forbidden")
    }
}

if ($failures.Count -ne 0) {
    Write-Error ($failures -join "`n")
    exit 1
}

Write-Output 'PASS: Beckhoff state reads use static scalar/element leaves and tolerate missing symbols.'
