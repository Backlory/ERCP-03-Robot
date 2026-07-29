param(
    [string]$RobotRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$WorkspaceRoot = (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
)

$ErrorActionPreference = 'Stop'

$productionSnapshotRoot = Join-Path $WorkspaceRoot '04-simulator\beckhoff-GT'
$productionCpp = (Get-ChildItem -LiteralPath $productionSnapshotRoot -Recurse -File `
    -Filter 'beckhoff_driver.cpp' | Select-Object -First 1).FullName
$productionHpp = Join-Path (Split-Path -Parent $productionCpp) 'beckhoff_driver.hpp'
$currentCpp = Join-Path $RobotRoot 'plugins\lib\drivers\src\beckhoff_driver.cpp'
$currentHpp = Join-Path $RobotRoot 'plugins\lib\drivers\src\beckhoff_driver.hpp'

foreach ($path in @($productionCpp, $productionHpp, $currentCpp, $currentHpp)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Required contract source is missing: $path"
    }
}

function Get-AdsSymbols([string]$path) {
    $source = Get-Content -LiteralPath $path -Raw
    $matches = [regex]::Matches(
        $source,
        '(?s)\b(?:ReadData|WriteData)\s*\(\s*"(?<symbol>[^"]+)"')
    return @($matches | ForEach-Object { $_.Groups['symbol'].Value } | Sort-Object -Unique)
}

function Get-StructFields([string]$path, [string]$structName) {
    $source = Get-Content -LiteralPath $path -Raw
    $match = [regex]::Match(
        $source,
        "(?s)\bstruct\s+$([regex]::Escape($structName))\s*\{(?<body>.*?)\};")
    if (-not $match.Success) {
        throw "Cannot find struct $structName in $path"
    }

    $withoutComments = [regex]::Replace($match.Groups['body'].Value, '//.*', '')
    return @(($withoutComments -split ';') |
        ForEach-Object { ([regex]::Replace($_, '\s+', ' ')).Trim() } |
        Where-Object { $_ })
}

$failures = [System.Collections.Generic.List[string]]::new()
$productionSymbols = Get-AdsSymbols $productionCpp
$currentSymbols = Get-AdsSymbols $currentCpp
$unsupportedSymbols = @($currentSymbols | Where-Object { $_ -notin $productionSymbols })
if ($unsupportedSymbols.Count -ne 0) {
    $failures.Add(
        "Robot introduced ADS symbols absent from the production snapshot: " +
        ($unsupportedSymbols -join ', '))
}

$productionFeedbackFields = Get-StructFields $productionHpp 'ERCPFeedbackData'
$currentFeedbackFields = Get-StructFields $currentHpp 'ERCPFeedbackData'
if (($productionFeedbackFields -join "`n") -ne ($currentFeedbackFields -join "`n")) {
    $failures.Add(
        "ERCPFeedbackData no longer matches the production snapshot.`n" +
        "Expected: $($productionFeedbackFields -join '; ')`n" +
        "Actual:   $($currentFeedbackFields -join '; ')")
}

$currentSource = Get-Content -LiteralPath $currentCpp -Raw
foreach ($expectedDeclaration in @('bool driveErrors[14]{}', 'bool motorErrors[12]{}')) {
    if ($currentSource -notmatch [regex]::Escape($expectedDeclaration)) {
        $failures.Add("Production ADS array length changed; missing declaration: $expectedDeclaration")
    }
}

if ($failures.Count -ne 0) {
    Write-Error ($failures -join "`n`n")
    exit 1
}

Write-Output 'PASS: Robot ADS symbols, ERCP feedback layout, and error-array lengths match the production snapshot.'
