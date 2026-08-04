[CmdletBinding()]
param(
    [ValidateSet("check", "apply")]
    [string]$Mode = "check"
)

$ErrorActionPreference = "Stop"
$expectedVersion = "22.1.3"
$sourceExtensions = @(".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx")

function Resolve-ClangFormat {
    if ($env:ERCP_CLANG_FORMAT_EXE) {
        return (Resolve-Path -LiteralPath $env:ERCP_CLANG_FORMAT_EXE).Path
    }

    $command = Get-Command clang-format -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    $visualStudioPattern = Join-Path $env:ProgramFiles `
        "Microsoft Visual Studio\*\*\VC\Tools\Llvm\x64\bin\clang-format.exe"
    $bundled = Get-ChildItem -Path $visualStudioPattern -File -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending |
        Select-Object -First 1
    if ($bundled) {
        return $bundled.FullName
    }

    throw "clang-format $expectedVersion was not found. Set ERCP_CLANG_FORMAT_EXE to its executable."
}

function Get-FormatFiles {
    $ignorePatterns = @(Get-Content -LiteralPath (Join-Path $PSScriptRoot ".clang-format-ignore") |
        ForEach-Object { $_.Trim().Replace("\", "/") } |
        Where-Object { $_ -and -not $_.StartsWith("#") })

    $listed = @(& git -c core.quotepath=false -C $PSScriptRoot `
        ls-files --cached --others --exclude-standard)
    if ($LASTEXITCODE -ne 0) {
        throw "git ls-files failed in $PSScriptRoot"
    }

    foreach ($relativePath in $listed) {
        $normalized = $relativePath.Replace("\", "/")
        if ($sourceExtensions -notcontains [IO.Path]::GetExtension($normalized).ToLowerInvariant()) {
            continue
        }

        $ignored = $false
        foreach ($pattern in $ignorePatterns) {
            if ($normalized -like $pattern) {
                $ignored = $true
                break
            }
        }
        if (-not $ignored) {
            Join-Path $PSScriptRoot $relativePath
        }
    }
}

$clangFormat = Resolve-ClangFormat
$versionText = (& $clangFormat --version 2>&1 | Out-String).Trim()
if ($versionText -notmatch "clang-format version\s+$([regex]::Escape($expectedVersion))(?:\s|$)") {
    throw "Expected clang-format $expectedVersion, but found: $versionText"
}

$files = @(Get-FormatFiles | Sort-Object -Unique)
if ($files.Count -eq 0) {
    throw "No project-owned C/C++ files were found."
}

Write-Host "clang-format ${expectedVersion}: $Mode $($files.Count) project-owned C/C++ files"
$failed = $false
for ($offset = 0; $offset -lt $files.Count; $offset += 40) {
    $last = [Math]::Min($offset + 39, $files.Count - 1)
    $batch = @($files[$offset..$last])
    if ($Mode -eq "apply") {
        $output = & $clangFormat --style=file -i @batch 2>&1
    } else {
        $output = & $clangFormat --style=file --dry-run --Werror @batch 2>&1
    }
    if ($LASTEXITCODE -ne 0) {
        $failed = $true
    }
    if ($output) {
        $output | ForEach-Object { Write-Host $_ }
    }
}

if ($failed) {
    exit 1
}
