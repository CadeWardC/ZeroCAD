[CmdletBinding()]
param(
    [string] $BaselinePath,
    [string] $FormatEraReferencePath,
    [string] $CurrentReportPath,
    [string] $OutputPath,
    [ValidateRange(0.0, 1000.0)]
    [double] $AllowedRegressionPercent = 10.0,
    [ValidateRange(0.0, 1000.0)]
    [double] $TimingNoiseFloorMs = 0.5,
    [switch] $SkipGui,
    [ValidateRange(1, 120)]
    [int] $StartupTimeoutSeconds = 30,
    [ValidateRange(0, 60)]
    [int] $IdleSeconds = 3
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not $BaselinePath) {
    $BaselinePath = Join-Path $PSScriptRoot "phase0-baseline.json"
}
$baselineFullPath = [System.IO.Path]::GetFullPath($BaselinePath)
if (-not $FormatEraReferencePath) {
    $FormatEraReferencePath = Join-Path $PSScriptRoot "phase2-v5-format-reference.json"
}
$formatEraReferenceFullPath = [System.IO.Path]::GetFullPath($FormatEraReferencePath)

function Invoke-CoreMeasurements {
    Write-Host "Measuring the frozen Phase 0 core corpora..."
    $output = @(& cargo run --quiet --release -p zerocad-core --example phase0_baseline)
    if ($LASTEXITCODE -ne 0) {
        throw "The Phase 0 core measurement runner failed with exit code $LASTEXITCODE."
    }

    $json = $output -join [Environment]::NewLine
    try {
        return $json | ConvertFrom-Json
    }
    catch {
        throw "The Phase 0 runner did not emit valid JSON: $($_.Exception.Message)"
    }
}

function Get-ReferenceMachine {
    $os = Get-CimInstance Win32_OperatingSystem
    $cpu = Get-CimInstance Win32_Processor | Select-Object -First 1
    $computer = Get-CimInstance Win32_ComputerSystem
    $rust = (& rustc --version) -join " "
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to read the Rust toolchain version."
    }

    return [ordered]@{
        os = "$($os.Caption) $($os.Version)"
        cpu = [string] $cpu.Name
        memory_bytes = [uint64] $computer.TotalPhysicalMemory
        rust = $rust.Trim()
    }
}

function Measure-GuiApplication {
    Write-Host "Building and measuring the release GUI..."
    & cargo build --quiet --release -p zerocad-gui
    if ($LASTEXITCODE -ne 0) {
        throw "The release GUI build failed with exit code $LASTEXITCODE."
    }

    $executable = Join-Path $repoRoot "target/release/zerocad-gui.exe"
    if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
        throw "The release GUI executable was not found at $executable."
    }

    $process = $null
    $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    try {
        $process = Start-Process -FilePath $executable -PassThru -WindowStyle Hidden
        $deadline = [DateTime]::UtcNow.AddSeconds($StartupTimeoutSeconds)
        $ready = $false
        while ([DateTime]::UtcNow -lt $deadline) {
            Start-Sleep -Milliseconds 25
            if ($process.HasExited) {
                throw "The GUI exited before exposing its main window."
            }
            $process.Refresh()
            if ($process.MainWindowHandle.ToInt64() -ne 0) {
                $ready = $true
                break
            }
        }

        if (-not $ready) {
            throw "The GUI did not expose a main window within $StartupTimeoutSeconds seconds."
        }

        $coldStartMs = $stopwatch.Elapsed.TotalMilliseconds
        if ($IdleSeconds -gt 0) {
            Start-Sleep -Seconds $IdleSeconds
        }
        $process.Refresh()

        return [ordered]@{
            binary_bytes = [uint64] (Get-Item -LiteralPath $executable).Length
            cold_start_ms = [Math]::Round($coldStartMs, 3)
            idle_working_set_bytes = [uint64] $process.WorkingSet64
            peak_startup_working_set_bytes = [uint64] $process.PeakWorkingSet64
        }
    }
    finally {
        $stopwatch.Stop()
        if ($null -ne $process -and -not $process.HasExited) {
            Stop-Process -Id $process.Id -Force
            Wait-Process -Id $process.Id -ErrorAction SilentlyContinue
        }
    }
}

function Add-RegressionCheck {
    param(
        [System.Collections.Generic.List[string]] $Failures,
        [string] $Label,
        [double] $Baseline,
        [double] $Current,
        [double] $AllowedPercent,
        [double] $AbsoluteNoiseFloor = 0.0
    )

    $limit = $Baseline * (1.0 + $AllowedPercent / 100.0)
    if ($Current -gt $limit -and ($Current - $Baseline) -gt $AbsoluteNoiseFloor) {
        $change = (($Current / $Baseline) - 1.0) * 100.0
        $Failures.Add((
            "{0}: {1:N3} exceeds baseline {2:N3} by {3:N2}% (allowed {4:N2}%)" -f
                $Label, $Current, $Baseline, $change, $AllowedPercent
        )) | Out-Null
    }
}

function Compare-Reports {
    param(
        [object] $Baseline,
        [object] $Current,
        [double] $AllowedPercent,
        [AllowNull()]
        [object] $FormatEraReference
    )

    if ([int] $Baseline.schema -ne [int] $Current.schema) {
        throw "Baseline schema $($Baseline.schema) does not match current schema $($Current.schema)."
    }

    $failures = [System.Collections.Generic.List[string]]::new()
    Add-RegressionCheck $failures "analytic cylinder tessellation" `
        ([double] $Baseline.tessellation_ms) ([double] $Current.tessellation_ms) `
        $AllowedPercent $TimingNoiseFloorMs

    if ($null -ne $Current.application -and $null -ne $Baseline.application) {
        foreach ($metric in @(
            "binary_bytes",
            "cold_start_ms",
            "idle_working_set_bytes",
            "peak_startup_working_set_bytes"
        )) {
            Add-RegressionCheck $failures "application.$metric" `
                ([double] $Baseline.application.$metric) `
                ([double] $Current.application.$metric) $AllowedPercent
        }
    }
    elseif ($null -eq $Current.application) {
        Write-Warning "GUI measurements were skipped; application footprint budgets were not checked."
    }
    else {
        Write-Warning "The comparison report has no GUI baseline; application footprint budgets were not checked."
    }

    $baselineCorpora = @{}
    foreach ($corpus in @($Baseline.corpora)) {
        $baselineCorpora[[string] $corpus.name] = $corpus
    }
    $formatEraCorpora = @{}
    if ($null -ne $FormatEraReference) {
        foreach ($corpus in @($FormatEraReference.corpora)) {
            $formatEraCorpora[[string] $corpus.name] = $corpus
        }
    }

    $currentNames = [System.Collections.Generic.HashSet[string]]::new()
    foreach ($corpus in @($Current.corpora)) {
        $name = [string] $corpus.name
        $currentNames.Add($name) | Out-Null
        if (-not $baselineCorpora.ContainsKey($name)) {
            $failures.Add("Unexpected corpus '$name' is not present in the frozen baseline.") | Out-Null
            continue
        }

        $frozen = $baselineCorpora[$name]
        foreach ($structuralMetric in @("feature_count", "body_count")) {
            if ([uint64] $corpus.$structuralMetric -ne [uint64] $frozen.$structuralMetric) {
                $failures.Add(
                    "$name.$structuralMetric changed from $($frozen.$structuralMetric) to $($corpus.$structuralMetric)."
                ) | Out-Null
            }
        }

        if ([uint64] $corpus.triangle_count -ne [uint64] $frozen.triangle_count) {
            Write-Warning (
                "$name.triangle_count changed from $($frozen.triangle_count) to $($corpus.triangle_count); " +
                "review this geometry change before accepting it."
            )
        }

        foreach ($metric in @("cold_rebuild_ms", "warm_rebuild_ms")) {
            Add-RegressionCheck $failures "$name.$metric" `
                ([double] $frozen.$metric) ([double] $corpus.$metric) `
                $AllowedPercent $TimingNoiseFloorMs
        }

        $formatFrozen = if ($formatEraCorpora.ContainsKey($name)) {
            $formatEraCorpora[$name]
        }
        else {
            $frozen
        }
        foreach ($metric in @("compact_save_ms", "compact_open_ms")) {
            if ($formatFrozen -ne $frozen) {
                $original = [double] $frozen.$metric
                $currentValue = [double] $corpus.$metric
                $change = (($currentValue / $original) - 1.0) * 100.0
                Write-Host ((
                    "Format-era note: {0}.{1} is {2:N2}% from the original v4 baseline; " +
                    "additional regression is enforced against the bound v5 reference."
                ) -f $name, $metric, $change) -ForegroundColor DarkYellow
            }
            Add-RegressionCheck $failures "$name.$metric" `
                ([double] $formatFrozen.$metric) ([double] $corpus.$metric) `
                $AllowedPercent $TimingNoiseFloorMs
        }
        foreach ($metric in @("compact_bytes", "hydrated_bytes")) {
            if ($formatFrozen -ne $frozen) {
                $original = [double] $frozen.$metric
                $currentValue = [double] $corpus.$metric
                $change = (($currentValue / $original) - 1.0) * 100.0
                Write-Host ((
                    "Format-era note: {0}.{1} is {2:N2}% from the original v4 baseline; " +
                    "additional regression is enforced against the bound v5 reference."
                ) -f $name, $metric, $change) -ForegroundColor DarkYellow
            }
            Add-RegressionCheck $failures "$name.$metric" `
                ([double] $formatFrozen.$metric) ([double] $corpus.$metric) $AllowedPercent
        }
    }

    foreach ($name in $baselineCorpora.Keys) {
        if (-not $currentNames.Contains($name)) {
            $failures.Add("Frozen corpus '$name' is missing from the current report.") | Out-Null
        }
    }

    if ($failures.Count -gt 0) {
        Write-Host "Phase 0 regression gate failed:" -ForegroundColor Red
        foreach ($failure in $failures) {
            Write-Host " - $failure" -ForegroundColor Red
        }
        return $false
    }

    Write-Host "Phase 0 regression gate passed (maximum allowed regression: $AllowedPercent%)."
    return $true
}

$gatePassed = $false
Push-Location $repoRoot
try {
    if (-not (Test-Path -LiteralPath $baselineFullPath -PathType Leaf)) {
        throw "The committed baseline was not found at $baselineFullPath."
    }

    $baseline = Get-Content -Raw -LiteralPath $baselineFullPath | ConvertFrom-Json
    $formatEraReference = $null
    if (Test-Path -LiteralPath $formatEraReferenceFullPath -PathType Leaf) {
        $formatEraReference = Get-Content -Raw -LiteralPath $formatEraReferenceFullPath |
            ConvertFrom-Json
        if ([int] $formatEraReference.schema -ne [int] $baseline.schema) {
            throw "Format-era reference schema $($formatEraReference.schema) does not match baseline schema $($baseline.schema)."
        }
        $baselineDigest = (Get-FileHash -LiteralPath $baselineFullPath -Algorithm SHA256).Hash.ToLowerInvariant()
        if ([string] $formatEraReference.baseline_sha256 -ne $baselineDigest) {
            throw "The format-era reference is not bound to the current frozen Phase 0 baseline."
        }
    }
    if ($CurrentReportPath) {
        $currentFullPath = [System.IO.Path]::GetFullPath($CurrentReportPath)
        if (-not (Test-Path -LiteralPath $currentFullPath -PathType Leaf)) {
            throw "The current report was not found at $currentFullPath."
        }
        $current = Get-Content -Raw -LiteralPath $currentFullPath | ConvertFrom-Json
    }
    else {
        $core = Invoke-CoreMeasurements
        $application = if ($SkipGui) { $null } else { Measure-GuiApplication }
        $current = [pscustomobject] [ordered]@{
            schema = [int] $core.schema
            profile = [string] $core.profile
            captured_on = [DateTime]::UtcNow.ToString("yyyy-MM-ddTHH:mm:ssZ")
            reference_machine = Get-ReferenceMachine
            application = $application
            tessellation_ms = [double] $core.tessellation_ms
            tessellation_triangles = [uint64] $core.tessellation_triangles
            corpora = @($core.corpora)
        }
    }

    if ($OutputPath) {
        $outputFullPath = [System.IO.Path]::GetFullPath($OutputPath)
        if ($outputFullPath -eq $baselineFullPath) {
            throw "Refusing to overwrite the frozen baseline; choose a separate output path."
        }
        if ($outputFullPath -eq $formatEraReferenceFullPath) {
            throw "Refusing to overwrite the format-era reference; choose a separate output path."
        }
        $outputDirectory = Split-Path -Parent $outputFullPath
        if ($outputDirectory -and -not (Test-Path -LiteralPath $outputDirectory)) {
            New-Item -ItemType Directory -Path $outputDirectory | Out-Null
        }
        $current | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $outputFullPath -Encoding utf8
        Write-Host "Wrote current measurements to $outputFullPath"
    }

    $gatePassed = Compare-Reports $baseline $current $AllowedRegressionPercent $formatEraReference
}
finally {
    Pop-Location
}

if (-not $gatePassed) {
    exit 1
}
