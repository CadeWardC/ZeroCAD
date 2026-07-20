[CmdletBinding()]
param(
    [ValidateRange(1, 8)]
    [int] $BuildJobs = 4
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$env:CARGO_BUILD_JOBS = $BuildJobs.ToString()

function Assert-ReleaseUnwind {
    param(
        [Parameter(Mandatory)]
        [string] $Label,

        [Parameter(Mandatory)]
        [string] $WorkingDirectory,

        [Parameter(Mandatory)]
        [string[]] $CargoArgs
    )

    Push-Location $WorkingDirectory
    try {
        # Cargo writes ordinary compile progress to stderr. Do not let
        # PowerShell promote that non-error stream into a terminating error;
        # the native exit code remains authoritative.
        $previousErrorAction = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        $cfg = @(& cargo @CargoArgs)
        $cargoExitCode = $LASTEXITCODE
        $ErrorActionPreference = $previousErrorAction
        if ($cargoExitCode -ne 0) {
            throw "$Label release configuration probe failed."
        }
        if (-not ($cfg | Where-Object { $_.ToString().Trim() -eq 'panic="unwind"' })) {
            throw "$Label release artifacts are not compiled with panic=unwind."
        }
    }
    finally {
        Pop-Location
    }
}

# Probe the effective rustc configuration, not just the manifest text. This
# catches environment/config overrides that would make guarded catch_unwind
# recovery abort in the profiler, replay lane, or shipped application.
Assert-ReleaseUnwind `
    -Label "ZeroCAD" `
    -WorkingDirectory $repoRoot `
    -CargoArgs @("rustc", "--release", "-p", "zerocad-core", "--lib", "--", "--print", "cfg")

Assert-ReleaseUnwind `
    -Label "OpenRCAD" `
    -WorkingDirectory (Join-Path $repoRoot "OpenRCAD") `
    -CargoArgs @("rustc", "--release", "-p", "openrcad-algo", "--lib", "--", "--print", "cfg")

Write-Host "ZeroCAD and OpenRCAD release artifacts use panic=unwind." -ForegroundColor Green
