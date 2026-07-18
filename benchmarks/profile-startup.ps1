[CmdletBinding()]
param(
    [ValidateRange(3, 31)]
    [int] $Samples = 7,
    [ValidateRange(100.0, 60000.0)]
    [double] $TargetMedianMs = 1000.0,
    [ValidateRange(100.0, 60000.0)]
    [double] $TargetP95Ms = 1000.0,
    [ValidateRange(1, 120)]
    [int] $StartupTimeoutSeconds = 30,
    [string] $OutputPath,
    [switch] $SkipBuild,
    [string] $ExecutablePath,
    [switch] $SignedPackage,
    [switch] $InstalledArtifact,
    [switch] $Phase7ReleaseGate
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not $OutputPath) {
    $OutputPath = Join-Path $repoRoot "target/startup-profile.json"
}
$outputFullPath = [System.IO.Path]::GetFullPath($OutputPath)

if ($ExecutablePath -and -not $SkipBuild) {
    throw "Use -SkipBuild with -ExecutablePath so the installed artifact is not replaced by a local build."
}
if (-not $SkipBuild) {
    Write-Host "Building the release GUI once before startup sampling..."
    & cargo build --quiet --release -p zerocad-gui
    if ($LASTEXITCODE -ne 0) {
        throw "The release GUI build failed with exit code $LASTEXITCODE."
    }
}

$executable = if ($ExecutablePath) {
    [System.IO.Path]::GetFullPath($ExecutablePath)
} else {
    Join-Path $repoRoot "target/release/zerocad-gui.exe"
}
if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
    throw "The release GUI executable was not found at $executable."
}

function Measure-OneStartup {
    param([int] $SampleNumber)

    $process = $null
    $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    try {
        $process = Start-Process -FilePath $executable -PassThru -WindowStyle Hidden
        $deadline = [DateTime]::UtcNow.AddSeconds($StartupTimeoutSeconds)
        while ([DateTime]::UtcNow -lt $deadline) {
            Start-Sleep -Milliseconds 10
            if ($process.HasExited) {
                throw "Startup sample $SampleNumber exited before exposing its main window."
            }
            $process.Refresh()
            if ($process.MainWindowHandle.ToInt64() -ne 0) {
                return [ordered]@{
                    sample = $SampleNumber
                    startup_ms = [Math]::Round($stopwatch.Elapsed.TotalMilliseconds, 3)
                    working_set_bytes = [uint64] $process.WorkingSet64
                    peak_working_set_bytes = [uint64] $process.PeakWorkingSet64
                }
            }
        }
        throw "Startup sample $SampleNumber did not expose a main window within $StartupTimeoutSeconds seconds."
    }
    finally {
        $stopwatch.Stop()
        if ($null -ne $process -and -not $process.HasExited) {
            Stop-Process -Id $process.Id -Force
            Wait-Process -Id $process.Id -ErrorAction SilentlyContinue
        }
    }
}

function Get-Percentile {
    param(
        [double[]] $Values,
        [ValidateRange(0.0, 1.0)]
        [double] $Percentile
    )

    $ordered = @($Values | Sort-Object)
    $index = [Math]::Max(0, [Math]::Ceiling($Percentile * $ordered.Count) - 1)
    return [double] $ordered[$index]
}

$measurements = @()
for ($sample = 1; $sample -le $Samples; $sample++) {
    Write-Host "Measuring GUI startup $sample of $Samples..."
    $measurements += Measure-OneStartup -SampleNumber $sample
    Start-Sleep -Milliseconds 250
}

$times = [double[]] @($measurements | ForEach-Object { $_.startup_ms })
$median = Get-Percentile -Values $times -Percentile 0.5
$p95 = Get-Percentile -Values $times -Percentile 0.95
$performancePasses = $median -lt $TargetMedianMs -and $p95 -lt $TargetP95Ms
$releaseIdentityPasses = -not $Phase7ReleaseGate -or ($SignedPackage -and $InstalledArtifact)
$passes = $performancePasses -and $releaseIdentityPasses
$report = [ordered]@{
    measured_utc = [DateTime]::UtcNow.ToString("o")
    executable = $executable
    signed_package = [bool] $SignedPackage
    includes_installed_first_launch = [bool] $InstalledArtifact
    target_median_ms = $TargetMedianMs
    target_p95_ms = $TargetP95Ms
    sample_count = $Samples
    first_startup_ms = $times[0]
    minimum_startup_ms = ($times | Measure-Object -Minimum).Minimum
    median_startup_ms = $median
    p95_startup_ms = $p95
    maximum_startup_ms = ($times | Measure-Object -Maximum).Maximum
    passes = $passes
    samples = $measurements
}

$parent = Split-Path -Parent $outputFullPath
if ($parent) {
    [System.IO.Directory]::CreateDirectory($parent) | Out-Null
}
[System.IO.File]::WriteAllText(
    $outputFullPath,
    (($report | ConvertTo-Json -Depth 5) + [Environment]::NewLine),
    [System.Text.UTF8Encoding]::new($false)
)

Write-Host ("Startup median: {0:N3} ms (target < {1:N3}); p95: {2:N3} ms (target < {3:N3})" -f $median, $TargetMedianMs, $p95, $TargetP95Ms)
Write-Host "Report: $outputFullPath"
if (-not $passes) {
    if (-not $performancePasses) {
        Write-Error "The startup median or p95 exceeds the Phase 7 absolute budget."
    } else {
        Write-Error "The Phase 7 startup gate requires -SignedPackage and -InstalledArtifact."
    }
    exit 1
}
