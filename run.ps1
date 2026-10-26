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

# Health check: wait until Webots is actually ready to accept the extern controller (something listening on controller port 1235),
# instead of a fixed sleep. Uses a fast TCP socket probe and returns as soon as the port answers, or $false after the timeout.
function Test-WebotsPort([int]$Port = 1235) {
    $c = [System.Net.Sockets.TcpClient]::new()
    try { $c.Connect("127.0.0.1", $Port); return $true } catch { return $false } finally { $c.Close() }
}
function Wait-WebotsPort([int]$Port = 1235, [int]$TimeoutSec = 60) {
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        if (Test-WebotsPort $Port) { return $true }
        Start-Sleep -Milliseconds 150
    }
    return $false
}

if ($sim -eq "webots") {
    $already = [bool](Get-Process -Name "webots-bin", "webots" -ErrorAction SilentlyContinue)
    if (-not $already) {
        & (Join-Path $PSScriptRoot "start_webots.ps1")
    } else { Write-Host "Webots is already running: using it." }
    # Poll the controller port rather than sleeping a fixed time. On a warm machine the port answers almost immediately; a cold
    # Webots start can take ~10-15 s, so the 60 s budget covers a slow first launch. The old fixed 8 s sleep could under-wait
    # (Webots measured here needed ~13 s), which caused an early connect attempt before the house was ready.
    if (Wait-WebotsPort 1235 60) {
        Write-Host "Webots controller port 1235 is ready."
    } else {
        Write-Host "Webots controller port 1235 did not open within 60 s; continuing anyway (the MCP server will retry)." -ForegroundColor Yellow
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
