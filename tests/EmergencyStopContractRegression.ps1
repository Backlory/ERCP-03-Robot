param(
    [string]$RobotRoot = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = 'Stop'
$driverPath = Join-Path $RobotRoot 'plugins\lib\drivers\src\beckhoff_driver.cpp'
$cyclePath = Join-Path $RobotRoot 'RobotSystem\Robot\Motions\Motions.Cycles.cpp'
$basePath = Join-Path $RobotRoot 'RobotSystem\Robot\YunSBot.Base.cpp'
$driver = Get-Content -LiteralPath $driverPath -Raw -Encoding UTF8
$cycle = Get-Content -LiteralPath $cyclePath -Raw -Encoding UTF8
$base = Get-Content -LiteralPath $basePath -Raw -Encoding UTF8
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text([string]$haystack, [string]$text, [string]$description) {
    if (-not $haystack.Contains($text)) {
        $failures.Add("Missing emergency-stop contract: $description ($text)")
    }
}

function Require-NotText([string]$haystack, [string]$text, [string]$description) {
    if ($haystack.Contains($text)) {
        $failures.Add("Forbidden emergency-stop side effect: $description ($text)")
    }
}

function Get-Section([string]$haystack, [string]$startMarker, [string]$endMarker) {
    $start = $haystack.IndexOf($startMarker)
    $end = $haystack.IndexOf($endMarker, $start)
    if ($start -lt 0 -or $end -lt 0) {
        return $null
    }
    return $haystack.Substring($start, $end - $start)
}

$emergency = Get-Section $driver 'bool Beckhoff_Motor::EmergencyStop' 'BeckhoffSnapshot Beckhoff_Motor::Snapshot()'
if ($null -eq $emergency) {
    $failures.Add('Unable to isolate Beckhoff_Motor::EmergencyStop')
} else {
    Require-Text $emergency 'int emergencyStopSignal = bIsStop ? 1 : 0;' `
        'PLC project contract is 1=emergency, 0=release'
    Require-Text $emergency 'WriteData("MAIN.Emergency_Stop_FromMaster", 4, &emergencyStopSignal)' `
        'the emergency operation writes the dedicated PLC signal'
    Require-NotText $emergency 'ArmOperation(' `
        'release must not write Status_Command_FromMaster through ArmOperation'
    Require-NotText $emergency 'MoveState(' `
        'release must not inspect the robot motion state'
    Require-NotText $emergency 'Status_Command_FromMaster' `
        'emergency handling must not address the motion-state PLC signal'
}

Require-NotText $cycle 'if (EmergencyStopActive())' `
    'the control cycle must not gate ordinary motion commands on the emergency flag'
Require-NotText $cycle 'command.emergency_stop = 1;' `
    'the control cycle must not rewrite ordinary commands during emergency stop'
Require-NotText $base 'm_master_udp_emergency_stop' `
    'Master emergency state must not be held in a second request latch'
Require-NotText $base 'm_rpc_emergency_stop' `
    'RPC emergency state must not be merged into the Master signal path'

if ($failures.Count -ne 0) {
    Write-Error ($failures -join "`n")
    exit 1
}

Write-Output 'PASS: emergency stop only writes MAIN.Emergency_Stop_FromMaster.'
