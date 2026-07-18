[CmdletBinding()]
param(
    [string] $OutputPath,
    [switch] $ConfirmBestPerformanceAndIdle,
    [switch] $SkipEnvironmentChecks
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not $OutputPath) {
    $OutputPath = Join-Path $repoRoot "target/modeling-profile.json"
}
$outputFullPath = [System.IO.Path]::GetFullPath($OutputPath)
$environmentQualification = if ($SkipEnvironmentChecks) {
    "smoke_only"
}
else {
    "reference_conditions_verified"
}

if (-not $SkipEnvironmentChecks) {
    if (-not $ConfirmBestPerformanceAndIdle) {
        throw (
            "Set Windows power mode to 'Best performance', close unrelated workloads, " +
            "then rerun with -ConfirmBestPerformanceAndIdle."
        )
    }

    if (-not ("ZeroCadPowerStatus" -as [type])) {
        Add-Type @"
using System.Runtime.InteropServices;
using System;

public static class ZeroCadPowerStatus {
    [StructLayout(LayoutKind.Sequential)]
    public struct SystemPowerStatus {
        public byte ACLineStatus;
        public byte BatteryFlag;
        public byte BatteryLifePercent;
        public byte SystemStatusFlag;
        public uint BatteryLifeTime;
        public uint BatteryFullLifeTime;
    }

    [DllImport("kernel32.dll")]
    public static extern bool GetSystemPowerStatus(out SystemPowerStatus status);

    [DllImport("powrprof.dll", EntryPoint = "PowerGetEffectiveOverlayScheme")]
    public static extern int PowerGetEffectiveOverlayScheme(out Guid value);
}
"@
    }
    $status = New-Object ZeroCadPowerStatus+SystemPowerStatus
    if (-not [ZeroCadPowerStatus]::GetSystemPowerStatus([ref] $status)) {
        throw "Windows did not provide system power status."
    }
    if ($status.ACLineStatus -ne 1) {
        throw "The modeling profiler must run on AC power."
    }

    $effectiveOverlay = [Guid]::Empty
    $overlayStatus = [ZeroCadPowerStatus]::PowerGetEffectiveOverlayScheme(
        [ref] $effectiveOverlay
    )
    if ($overlayStatus -ne 0) {
        throw "Windows did not provide the effective power-mode overlay."
    }
    $bestPerformanceOverlay = [Guid] "ded574b5-45a0-4f42-8737-46345c09c238"
    if ($effectiveOverlay -ne $bestPerformanceOverlay) {
        throw (
            "Windows power mode must be 'Best performance'. " +
            "The current overlay is $effectiveOverlay."
        )
    }
}
else {
    Write-Warning (
        "Environment checks are disabled. This output is smoke data, not release evidence."
    )
}

Push-Location $repoRoot
try {
    $output = @(& cargo run --quiet --release -p zerocad-core --example modeling_profiler)
    $profilerExitCode = $LASTEXITCODE
}
finally {
    Pop-Location
}

$json = $output -join [Environment]::NewLine
try {
    $report = $json | ConvertFrom-Json
}
catch {
    throw "The modeling profiler did not emit valid JSON: $($_.Exception.Message)"
}
$report | Add-Member -NotePropertyName environment_qualification -NotePropertyValue (
    $environmentQualification
)

$outputDirectory = Split-Path -Parent $outputFullPath
if ($outputDirectory) {
    New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
}
[System.IO.File]::WriteAllText(
    $outputFullPath,
    ($report | ConvertTo-Json -Depth 8),
    [System.Text.UTF8Encoding]::new($false)
)
Write-Host "Modeling profile written to $outputFullPath"

foreach ($workload in @($report.workloads)) {
    Write-Host ((
        "{0}: p50 {1:N3} ms, p95 {2:N3} ms, budget < {3:N3} ms" -f
            $workload.name, $workload.p50_ms, $workload.p95_ms, $workload.budget_ms
    ))
}

if ($profilerExitCode -ne 0 -or -not [bool] $report.passed) {
    throw "The release-mode modeling performance gate failed."
}
