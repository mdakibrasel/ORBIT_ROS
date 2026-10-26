# ORBIT-ROS one-time setup for Windows PowerShell.
# Run from this folder:   powershell -ExecutionPolicy Bypass -File .\setup.ps1
# What it does: finds Python 3.10+, creates .venv, runs the tests, creates your private .env from .env.example (never overwrites it),
# checks the API keys, runs the no-Webots probe, tells Webots which Python to use, and shows where Webots is installed.
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
function Step($m) { Write-Host ""; Write-Host "== $m" -ForegroundColor Cyan }
function Ok($m)   { Write-Host "   OK  $m" -ForegroundColor Green }
function Warn($m) { Write-Host "   !!  $m" -ForegroundColor Yellow }

Step "1/7 Python 3.10 or newer"
$py = $null
foreach ($cand in @("py", "python", "python3")) {
    if (Get-Command $cand -ErrorAction SilentlyContinue) {
        $v = & $cand -c "import sys; print(sys.version_info[0]*100+sys.version_info[1])" 2>$null
        if ($v -and ([int]$v -ge 310)) { $py = $cand; break }
    }
}
if (-not $py) { Write-Host "Python 3.10+ not found. Install it from https://www.python.org/downloads/ (tick 'Add to PATH'), then run this script again." -ForegroundColor Red; exit 1 }
Ok "using '$py'"

Step "2/7 Virtual environment (.venv) and packages"
if (-not (Test-Path ".venv\Scripts\python.exe")) { & $py -m venv .venv }
$venvPy = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
Ok $venvPy
# v0.15: install the packages here (until v0.14.2 this script did not, and the tests then failed with "No module named numpy")
& $venvPy -m pip install --upgrade pip
& $venvPy -m pip install -r requirements.txt
if ($LASTEXITCODE -ne 0) { Write-Host "pip could not install requirements.txt (see above). With a very new Python, install Python 3.13 and run: py -3.13 -m venv .venv" -ForegroundColor Red; exit 1 }
Ok "packages installed"

Step "3/7 Unit tests (about 1 minute)"
& $venvPy -m unittest discover -s tests
if ($LASTEXITCODE -ne 0) { Write-Host "Some tests failed: do not continue before reading the output above." -ForegroundColor Red; exit 1 }
Ok "all tests passed"

Step "4/7 Your private .env (API keys)"
if (-not (Test-Path ".env")) { Copy-Item ".env.example" ".env"; Ok "created .env from .env.example. Open it and paste your keys:  code .env" }
else { Ok ".env already exists (left untouched)" }
& $venvPy check_env.py

Step "5/7 NAO MCP configuration (live Webots)"
& $venvPy orbit_mcp_server.py --robot nao --sim webots --bridge-only --print-config | Out-Null
if ($LASTEXITCODE -ne 0) { Warn "the live Webots MCP configuration check failed" } else { Ok "NAO MCP is configured for the live Webots bridge" }

Step "6/7 Python for Webots"
Ok "the NAO runs as an extern controller started by this project's Python ($venvPy); no Webots setting is needed"

Step "7/7 Webots installation"
. (Join-Path $PSScriptRoot "tools\find_webots.ps1")
$found = Find-WebotsExe $PSScriptRoot
if ($found) {
    Ok "found $found"
    $wbHome = Get-WebotsHome $found
    $requiredProto = Join-Path $wbHome "projects\robots\softbank\nao\protos\Nao.proto"
    if (Test-Path $requiredProto) {
        Ok "Webots NAO PROTO assets are installed"
    } else {
        Warn "Webots NAO PROTO assets are missing; the house world uses official R2025a HTTPS PROTO URLs as a fallback"
    }
    $hasHome = (Test-Path ".env") -and (Select-String -Path ".env" -Pattern '^\s*WEBOTS_HOME\s*=\s*[^\s#]' -Quiet)
    if ((Test-Path ".env") -and -not $hasHome) {
        Add-Content -Path ".env" -Value "`r`nWEBOTS_HOME=$wbHome" -Encoding UTF8
        Ok "wrote WEBOTS_HOME=$wbHome into .env (the Python part and start_webots.ps1 use it)"
    }
} else { Warn "webots.exe not found anywhere (default folders, list of installed programs, PATH, <drive>:\Webots). Put WEBOTS_HOME=<your Webots folder> in .env." }

Write-Host ""
Write-Host "Next:" -ForegroundColor Cyan
Write-Host "  1. Paste your API keys into .env (only needed for live-LLM runs; Webots runs need none)."
Write-Host "  2. Open the NAO house:     powershell -ExecutionPolicy Bypass -File .\start_webots.ps1   (default world: webots\worlds\orbit_house.wbt)"
Write-Host "  3. NAO in Webots: run .\run.ps1 -Mode server and keep the bridge window open, then start orbit-nao-webots in VS Code."
