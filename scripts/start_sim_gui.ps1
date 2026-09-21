# Start VcXsrv on Windows, then launch Gazebo/RViz from WSL onto that display.
[CmdletBinding(PositionalBinding = $false)]
param(
    [string]$Distro = 'Ubuntu-22.04',
    [string]$Workspace = '',
    [string]$Vcxsrv = 'C:\Program Files\VcXsrv\vcxsrv.exe',
    [switch]$SoftwareGl,
    [switch]$XServerOnly,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$LaunchArguments
)
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if (-not $Workspace) { $Workspace = $repoRoot }
if (-not (Test-Path -LiteralPath $Vcxsrv)) {
    throw "VcXsrv not found: $Vcxsrv. Install marha.VcXsrv or pass -Vcxsrv."
}

# Multi-window + native OpenGL. -listen tcp is required for WSL2. -ac skips xauth.
$glArg = if ($SoftwareGl) { '-nowgl' } else { '-wgl' }
if (-not (Get-Process -Name vcxsrv -ErrorAction SilentlyContinue)) {
    Start-Process -FilePath $Vcxsrv -ArgumentList @(
        ':0', '-multiwindow', '-clipboard', $glArg, '-ac', '-dpi', 'auto',
        '-listen', 'tcp', '-silent-dup-error'
    )
    $deadline = (Get-Date).AddSeconds(8)
    $ok = $false
    do {
        Start-Sleep -Milliseconds 250
        if (Get-Process -Name vcxsrv -ErrorAction SilentlyContinue) { $ok = $true; break }
    } while ((Get-Date) -lt $deadline)
    if (-not $ok) { throw 'VcXsrv failed to start.' }
}

if ($XServerOnly) {
    Write-Host 'VcXsrv is running. In WSL you can now: ros2 launch sim_env main.launch.py world:=ocean'
    return
}

# Convert D:\foo to /mnt/d/foo here; wslpath would swallow Windows backslash escapes.
if ($Workspace -match '^[A-Za-z]:[\\/]') {
    $wslRoot = '/mnt/' + $Workspace.Substring(0, 1).ToLowerInvariant() + ($Workspace.Substring(2) -replace '\\', '/')
} else {
    $wslRoot = ($Workspace -replace '\\', '/')
}

if (-not $LaunchArguments) { $LaunchArguments = @() }
$mainSh = "$wslRoot/scripts/main.sh"
Write-Host "Starting simulation GUI via $Distro workspace=$wslRoot"
if ($SoftwareGl) {
    $env:LIBGL_ALWAYS_SOFTWARE = '1'
    if ($env:WSLENV) {
        if ($env:WSLENV -notmatch 'LIBGL_ALWAYS_SOFTWARE') {
            $env:WSLENV = $env:WSLENV + ':LIBGL_ALWAYS_SOFTWARE'
        }
    } else {
        $env:WSLENV = 'LIBGL_ALWAYS_SOFTWARE'
    }
}
$wslCmd = @('-d', $Distro, '--', 'bash', $mainSh) + @($LaunchArguments)
& wsl.exe @wslCmd
