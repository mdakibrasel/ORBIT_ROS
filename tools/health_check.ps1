# Fast, safe readiness check for the ORBIT Webots/MCP launch path.
# Default mode only probes the controller port and never takes the single NAO controller slot.
param([switch]$VerifyRobot)
$ErrorActionPreference = "Stop"
Set-Location (Split-Path -Parent $PSScriptRoot)
$port = 1235
$client = [System.Net.Sockets.TcpClient]::new()
try {
    $client.Connect("127.0.0.1", $port)
    Write-Host "[health] READY: Webots controller port $port is listening."
} catch {
    Write-Error "[health] NOT READY: nothing is listening on 127.0.0.1:$port. Start Webots with .\start_webots.ps1 first."
    exit 2
} finally {
    $client.Close()
}
if ($VerifyRobot) {
    $py = Join-Path $PSScriptRoot "..\.venv\Scripts\python.exe"
    if (-not (Test-Path $py)) {
        Write-Error "[health] NOT READY: .venv Python was not found at $py."
        exit 1
    }
    & $py (Join-Path $PSScriptRoot "..\orbit_mcp_server.py") --health-check --verify-robot --robot nao --sim webots
    exit $LASTEXITCODE
}
exit 0
