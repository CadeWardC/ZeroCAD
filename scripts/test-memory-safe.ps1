[CmdletBinding()]
param(
    [ValidateRange(1, 8)]
    [int] $BuildJobs = 4,

    [ValidateSet(1)]
    [int] $TestThreads = 1,

    [ValidateRange(512, 32768)]
    [int] $MinimumFreeMemoryMB = 4096,

    [switch] $UnitOnly,

    [switch] $SkipUnit,

    [switch] $SkipGui,

    [string] $StartAtUnitGroup,

    [string[]] $UnitGroups,

    [string] $StartAtIntegrationTarget,

    [string[]] $IntegrationTargets
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot

function Assert-FreeMemory {
    $os = Get-CimInstance Win32_OperatingSystem
    [int] $freeMB = [math]::Round($os.FreePhysicalMemory / 1KB)
    if ($freeMB -lt $MinimumFreeMemoryMB) {
        throw "Only $freeMB MB of physical memory is free; refusing to start the next test slice (minimum: $MinimumFreeMemoryMB MB)."
    }
    return $freeMB
}

function Set-AdaptiveBuildJobs {
    [int] $freeMB = Assert-FreeMemory
    [int] $memorySafeJobs = if ($freeMB -ge 8192) {
        4
    }
    elseif ($freeMB -ge 6144) {
        3
    }
    else {
        2
    }
    [int] $effectiveJobs = [math]::Min($BuildJobs, $memorySafeJobs)
    $env:CARGO_BUILD_JOBS = $effectiveJobs.ToString()
    Write-Host (
        "Free memory: {0:N0} MiB; compiler jobs: {1}; test threads: {2}." -f
        $freeMB, $effectiveJobs, $TestThreads
    ) -ForegroundColor DarkGray
}

function Invoke-CargoSlice {
    param(
        [Parameter(Mandatory)]
        [string] $Label,

        [Parameter(Mandatory)]
        [string[]] $CargoArgs
    )

    Set-AdaptiveBuildJobs
    Write-Host "`n==> $Label" -ForegroundColor Cyan
    & cargo @CargoArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Test slice failed: $Label"
    }
}

function Get-UnitTestGroup {
    param(
        [Parameter(Mandatory)]
        [string] $TestName
    )

    [string[]] $parts = $TestName -split "::"

    # Parametric evaluator families are the largest unit-test modules. Give
    # each family its own process so geometry allocations are returned to
    # Windows before the next family starts.
    if ($parts.Count -ge 3 -and $parts[0] -eq "parametric" -and $parts[1] -eq "tests") {
        return ($parts[0..2] -join "::")
    }

    $testsIndex = [Array]::IndexOf($parts, "tests")
    if ($testsIndex -ge 0) {
        return ($parts[0..$testsIndex] -join "::")
    }

    if ($parts.Count -ge 2) {
        return ($parts[0..1] -join "::")
    }
    return $parts[0]
}

Push-Location $repoRoot
try {
    if (-not $SkipUnit) {
        if ($UnitGroups -and $UnitGroups.Count -gt 0) {
            $groups = @($UnitGroups)
        }
        else {
            Set-AdaptiveBuildJobs
            Write-Host "Discovering zerocad-core unit-test slices (parallel compilation, one test process at a time)..." -ForegroundColor Cyan
            $listedTests = & cargo test -p zerocad-core --lib -- --list --format terse
            if ($LASTEXITCODE -ne 0) {
                throw "Could not enumerate zerocad-core unit tests."
            }

            $groups = $listedTests |
                ForEach-Object {
                    if ($_ -match "^(?<name>.+): test$") {
                        Get-UnitTestGroup -TestName $Matches.name
                    }
                } |
                Where-Object { $_ } |
                Sort-Object -Unique
        }

        if ($StartAtUnitGroup) {
            $startIndex = [Array]::IndexOf([string[]] $groups, $StartAtUnitGroup)
            if ($startIndex -lt 0) {
                throw "Unknown unit-test group '$StartAtUnitGroup'."
            }
            $groups = @($groups[$startIndex..($groups.Count - 1)])
        }

        foreach ($group in $groups) {
            Invoke-CargoSlice -Label "zerocad-core unit group: $group" -CargoArgs @(
                "test", "-p", "zerocad-core", "--lib", $group, "--", "--test-threads=$TestThreads"
            )
        }
    }

    if (-not $UnitOnly) {
        if ($IntegrationTargets -and $IntegrationTargets.Count -gt 0) {
            $integrationTargets = @($IntegrationTargets)
        }
        else {
            Set-AdaptiveBuildJobs
            $metadata = (& cargo metadata --no-deps --format-version 1 | ConvertFrom-Json)
            if ($LASTEXITCODE -ne 0) {
                throw "Could not read Cargo test targets."
            }
            $corePackage = $metadata.packages | Where-Object { $_.name -eq "zerocad-core" }
            $integrationTargets = $corePackage.targets |
                Where-Object { $_.kind -contains "test" } |
                Select-Object -ExpandProperty name |
                Sort-Object -Unique
        }

        if ($StartAtIntegrationTarget) {
            $startIndex = [Array]::IndexOf([string[]] $integrationTargets, $StartAtIntegrationTarget)
            if ($startIndex -lt 0) {
                throw "Unknown integration-test target '$StartAtIntegrationTarget'."
            }
            $integrationTargets = @($integrationTargets[$startIndex..($integrationTargets.Count - 1)])
        }

        foreach ($target in $integrationTargets) {
            Invoke-CargoSlice -Label "zerocad-core integration target: $target" -CargoArgs @(
                "test", "-p", "zerocad-core", "--test", $target, "--", "--test-threads=$TestThreads"
            )
        }
    }

    if (-not $SkipGui) {
        Invoke-CargoSlice -Label "zerocad-gui tests" -CargoArgs @(
            "test", "-p", "zerocad-gui", "--", "--test-threads=$TestThreads"
        )
    }

    Write-Host "`nAll memory-safe test slices passed." -ForegroundColor Green
}
finally {
    Pop-Location
}
