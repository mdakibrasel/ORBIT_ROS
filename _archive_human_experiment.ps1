$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
$ts = Get-Date -Format 'yyyyMMdd_HHmmss'
$arc = Join-Path $PSScriptRoot ("_archive_human_experiment_" + $ts)
New-Item -ItemType Directory -Force -Path $arc | Out-Null
Write-Output ("ARCHIVE=" + $arc)

# Experiment-EXCLUSIVE items (relative paths). Shared/original files (orbit_house.wbt, orbit_ros/*,
# tools/make_house_world.py, tests, docs, results, backups) are intentionally NOT listed.
$items = @(
  # experimental worlds + their project/companion files
  "webots\worlds\orbit_house_human_realistic.wbt",
  "webots\worlds\.orbit_house_human_realistic.wbproj",
  "webots\worlds\orbit_house_human.wbt",
  "webots\worlds\_verify_sit.wbt",
  "webots\worlds\._verify_sit.wbproj",
  "webots\worlds\orbit_human_cmd.txt",
  # experiment controllers
  "webots\controllers\orbit_human_skin",
  "webots\controllers\orbit_sit_verify",
  # tools generators / references created for the human experiment
  "tools\make_sit_bvh.py",
  "tools\make_skin_human_world.py",
  "tools\make_realistic_human_world.py",
  "tools\make_human_world.py",
  "tools\walk_ref.bvh",
  "tools\stepup_ref.bvh",
  "tools\jump_ref.bvh",
  # loose root debug/helper scripts created during the experiment
  "orbit_human_skin.py",
  "_live_verify2.ps1",
  "_run_sit_verify.ps1",
  "_bk.ps1",
  "_bk_now.ps1",
  "_bk_tmp.ps1",
  "_inspect_procs.ps1",
  "_proc_check.ps1",
  "_probe_live2.ps1",
  "_skin_probe.py",
  "_live_human_tool.py",
  "_tmp_probe_human.py",
  "_tmp_layout.py",
  "_tmp_probe",
  "_verify_out",
  "_live_nao_selftest.log",
  "_live_webots_console.log",
  "_verify_webots.log",
  "_verify_webots.err.log"
)

$moved = @()
$skipped = @()
foreach ($rel in $items) {
  $src = Join-Path $PSScriptRoot $rel
  if (Test-Path $src) {
    $dst = Join-Path $arc $rel
    $dstParent = Split-Path $dst -Parent
    New-Item -ItemType Directory -Force -Path $dstParent | Out-Null
    Move-Item -LiteralPath $src -Destination $dst -Force
    $moved += $rel
  } else {
    $skipped += $rel
  }
}

# This archive script itself should go into the archive after it runs (record its own name).
Write-Output "----MOVED----"
$moved | ForEach-Object { Write-Output $_ }
Write-Output "----SKIPPED (not present)----"
$skipped | ForEach-Object { Write-Output $_ }
# write a manifest inside the archive
$manifest = Join-Path $arc "ARCHIVE_MANIFEST.txt"
Set-Content -Path $manifest -Value ("Archived " + (Get-Date).ToString("u") + "`nReason: failed CharacterSkin sitting-human experiment; restored NAO world orbit_house.wbt.`n`nMOVED:`n" + ($moved -join "`n") + "`n`nSKIPPED (absent):`n" + ($skipped -join "`n"))
Write-Output ("MANIFEST=" + $manifest)
Write-Output "DONE"
