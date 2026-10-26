# VS Code MCP entrypoint: start/verify one Webots house, wait for port 1235,
# then replace this process with the persistent stdio MCP server.
param(
    [int]$Port = 1235,
    [int]$TimeoutSec = 60,
    [int]$RetryIntervalMs = 150
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
if (-not (Test-Path $py)) { [Console]::Error.WriteLine("[orbit-launch] ERROR: .venv Python not found at $py"); exit 1 }

$listener = Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue | Select-Object -First 1
$webots = Get-Process -Name "webots-bin", "webots" -ErrorAction SilentlyContinue
if (-not $listener -and -not $webots) {
    [Console]::Error.WriteLine("[orbit-launch] STARTING: launching Webots house")
    & (Join-Path $root "start_webots.ps1") 2>&1 | ForEach-Object { [Console]::Error.WriteLine($_) }
}

$deadline = (Get-Date).AddSeconds($TimeoutSec)
while ((Get-Date) -lt $deadline) {
    $probe = [System.Net.Sockets.TcpClient]::new()
    try {
        $probe.Connect("127.0.0.1", $Port)
        [Console]::Error.WriteLine("[orbit-launch] READY: Webots port $Port is listening; starting persistent orbit_nao MCP server")
        & $py (Join-Path $root "orbit_mcp_server.py") --robot nao --sim webots --keep-alive
        $serverExit = $LASTEXITCODE
        [Console]::Error.WriteLine("[orbit-launch] DISCONNECTED: MCP server exited with code $serverExit; retrying without launching another Webots")
        Start-Sleep -Milliseconds $RetryIntervalMs
        continue
    } catch {
        Start-Sleep -Milliseconds $RetryIntervalMs
    } finally {
        $probe.Close()
    }
}
[Console]::Error.WriteLine("[orbit-launch] ERROR: Webots did not open controller port $Port within $TimeoutSec s. Check the Webots console and run .\run.ps1 -Mode health.")
exit 2
