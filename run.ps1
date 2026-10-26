# One command for the NAO house project.
#   .\run.ps1                         Webots (house) + chat with the LLM (provider from -Provider, key from .env)
#   .\run.ps1 -Provider groq          use Groq instead of Gemini
#   .\run.ps1 -Fake                   no Webots: kinematic stand-in + chat
#   .\run.ps1 -Mode server            only start Webots; your AI app starts the MCP server (.vscode\mcp.json: orbit-nao-webots),
#                                     which connects to Webots directly on 127.0.0.1:1235 and (with --keep-alive) stays connected
#   .\run.ps1 -Mode hold              start Webots AND hold the NAO controller connected in this window until Ctrl+C (persistent,
#                                     independent of any AI client; do not also start orbit-nao-webots, as only one controller may own the robot)
#   .\run.ps1 -Mode selftest          scripted check; Webots stays connected afterwards (-KeepAlive is on)
#   .\run.ps1 -Mode benchmark         NAO benchmark (CSV in results_nao_benchmark\)
#   .\run.ps1 -Mode test              unit tests
param(
    [ValidateSet("chat", "server", "hold", "selftest", "benchmark", "test")] [string]$Mode = "chat",
    [string]$Provider = "gemini",
    [string]$Model = "",
    [switch]$Fake
)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
$py = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $py)) { Write-Host "No .venv: run .\setup.ps1 first." -ForegroundColor Red; exit 1 }
if ($Mode -eq "test") { & $py -m unittest discover -s tests -t .; exit $LASTEXITCODE }

$sim = if ($Fake) { "fake" } else { "webots" }

# Health check: wait until Webots is actually ready to accept the extern controller (a listener on controller port 1235),
# instead of a fixed sleep. Returns as soon as the port is open, or $false after the timeout.
function Wait-WebotsPort([int]$Port = 1235, [int]$TimeoutSec = 30) {
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        if (Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue) { return $true }
        Start-Sleep -Milliseconds 200
    }
    return $false
}

if ($sim -eq "webots") {
    $already = [bool](Get-Process -Name "webots-bin", "webots" -ErrorAction SilentlyContinue)
    if (-not $already) {
        & (Join-Path $PSScriptRoot "start_webots.ps1")
    } else { Write-Host "Webots is already running: using it." }
    # Poll the controller port rather than sleeping a fixed time. On a warm machine Webots is ready in well under 8 s; on a
    # slow first start the 30 s budget still covers it. If it is already running, the port is usually up immediately.
    if (Wait-WebotsPort 1235 30) {
        Write-Host "Webots controller port 1235 is listening (ready)."
    } else {
        Write-Host "Webots controller port 1235 did not open within 30 s; continuing anyway (the MCP server will retry)." -ForegroundColor Yellow
    }
}
if ($Mode -eq "server") {
    Write-Host "Webots is up. Start orbit-nao-webots from your AI app; it connects to Webots directly on 127.0.0.1:1235 (no bridge)."
    Write-Host "The AI-client server uses --keep-alive, so the controller stays connected even when the client disconnects."
    Write-Host "To hold the controller in THIS window instead (no AI client), run:  .\run.ps1 -Mode hold"
    exit 0
}
if ($Mode -eq "hold") {
    # Persistent controller: this process becomes the single 'orbit_nao' extern controller and holds it (stepping the world)
    # until Ctrl+C or Webots closes. It serves MCP on stdin/stdout too, but since no AI client is attached here it simply
    # keeps the connection alive. Do NOT also start orbit-nao-webots while this runs; only one controller may own the robot.
    if ($sim -ne "webots") { Write-Host "-Mode hold needs Webots (drop -Fake)." -ForegroundColor Red; exit 1 }
    Write-Host "Holding the NAO controller connected on 127.0.0.1:1235. Press Ctrl+C to release it."
    & $py orbit_mcp_server.py --robot nao --sim webots --keep-alive
    exit $LASTEXITCODE
}
if ($Mode -eq "benchmark") {
    $b = @("tools\nao_benchmark.py", "--sim", $sim); if ($Model) { $b += @("--model", $Model) }
    & $py @b; exit $LASTEXITCODE
}
$srv = @("--robot", "nao", "--sim", $sim)
if ($Mode -eq "selftest") {
    if ($sim -eq "webots") { $srv += "--keep-alive" }
    & $py orbit_mcp_server.py --selftest @srv; exit $LASTEXITCODE
}
$c = @("orbit_chat.py", "--provider", $Provider); if ($Model) { $c += @("--model", $Model) }
& $py @c -- @srv
