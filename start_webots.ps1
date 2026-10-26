# Start Webots with the ORBIT house world (the NAO humanoid project):  .\start_webots.ps1
# Opens webots\worlds\orbit_house.wbt only. For the persistent NAO controller and MCP bridge, use .\run.ps1 -Mode server.
param([switch]$Humanoid, [string]$World)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
if (Get-Process -Name "webots-bin", "webots" -ErrorAction SilentlyContinue) {
    [Console]::Error.WriteLine("Webots is already running. Close it first; ORBIT uses controller port 1235 and this script will not start a second instance.")
    exit 1
}
$world = if ($World) { if ([System.IO.Path]::IsPathRooted($World)) { $World } else { Join-Path $PSScriptRoot $World } } else { Join-Path $PSScriptRoot "webots\worlds\orbit_house.wbt" }
if (-not (Test-Path $world)) { Write-Host "World file not found: $world" -ForegroundColor Red; exit 1 }
$port = 1235
$portOwner = Get-NetTCPConnection -State Listen -LocalPort $port -ErrorAction SilentlyContinue | Select-Object -First 1
if ($portOwner) {
    $owner = Get-Process -Id $portOwner.OwningProcess -ErrorAction SilentlyContinue
    $name = if ($owner) { $owner.ProcessName } else { "unknown process" }
    Write-Host "Controller port $port is already in use by $name (PID $($portOwner.OwningProcess)). Close it before starting Webots." -ForegroundColor Red
    exit 1
}
. (Join-Path $PSScriptRoot "tools\find_webots.ps1")
$webots = Find-WebotsExe $PSScriptRoot
if (-not $webots) { [Console]::Error.WriteLine("webots.exe not found (looked at WEBOTS_HOME, the default folders, the list of installed programs, the PATH and <drive>:\Webots on every drive). Put WEBOTS_HOME=<your Webots folder> in .env."); exit 1 }
[Console]::Error.WriteLine("Webots: $webots")
[Console]::Error.WriteLine("House world: Webots waits for the NAO's controller. The MCP wrapper will start the external controller.")
[Console]::Error.WriteLine("External-controller port: $port")
Start-Process -FilePath $webots -ArgumentList @("--mode=realtime", "--port=$port", "`"$world`"")