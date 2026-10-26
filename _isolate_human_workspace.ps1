$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
$ws = Join-Path $PSScriptRoot "_human_workspace"
New-Item -ItemType Directory -Force -Path $ws | Out-Null
Write-Output ("WORKSPACE=" + $ws)

# Human-EXCLUSIVE source still living in the active tree after the prior archive/restore.
# (Shared NAO modules are deliberately NOT listed: orbit_ros/viewpoint.py, tests/test_camera.py,
#  tools/live/phase16_camera_trial.py all belong to the NAO camera system and stay in place.)
$items = @(
  "orbit_ros\human_controller.py",
  "tests\test_human_controller.py",
  "tests\test_realistic_human_world.py",
  "tools\live\p13_human_live.py"
)

$moved = @(); $skipped = @()
foreach ($rel in $items) {
  $src = Join-Path $PSScriptRoot $rel
  if (Test-Path $src) {
    $dst = Join-Path $ws $rel
    New-Item -ItemType Directory -Force -Path (Split-Path $dst -Parent) | Out-Null
    Move-Item -LiteralPath $src -Destination $dst -Force
    $moved += $rel
  } else { $skipped += $rel }
}

Write-Output "----MOVED----"; $moved | ForEach-Object { Write-Output $_ }
Write-Output "----SKIPPED (absent)----"; $skipped | ForEach-Object { Write-Output $_ }

$readme = Join-Path $ws "README_HUMAN_WORKSPACE.md"
$body = @"
# Human Workspace (isolated)

Isolated $(Get-Date -Format 'u').

This folder holds the **Human-exclusive** source that was separated from the active NAO project so it can
never interfere with or modify the NAO world (``webots/worlds/orbit_house.wbt``). It is NOT on any NAO
run/test path. The NAO world, its controllers, MCP integration, and shared modules remain in place.

## Moved here (mirrors original relative paths)
$($moved | ForEach-Object { "- ``$_``" } | Out-String)
## Why these (and only these)
- ``orbit_ros/human_controller.py`` - drives the animated Human actor (``DEF ORBIT_HUMAN`` Pedestrian); no NAO/production module imports it.
- ``tests/test_human_controller.py`` - imports ``orbit_ros.human_controller`` (Human-only).
- ``tests/test_realistic_human_world.py`` - imports the archived Human world generator (Human-only; was import-broken in the active tree).
- ``tools/live/p13_human_live.py`` - Phase-13 live Human check; imports ``HumanController``.

## Intentionally KEPT in the active NAO project (shared, NOT Human-exclusive)
- ``orbit_ros/viewpoint.py`` - NAO camera/viewpoint system; its optional ORBIT_HUMAN framing branch is inert when no such node exists.
- ``tests/test_camera.py`` - tests the shared NAO camera module.
- ``tools/live/phase16_camera_trial.py`` - NAO camera live trial (records human presence if any; degrades gracefully).

## Related earlier isolation
The Human worlds, the ``orbit_human_skin``/``orbit_sit_verify`` controllers, the Human world generators, and the
sitting-experiment helpers were previously moved to ``_archive_human_experiment_20261025_124510/`` (see its
``ARCHIVE_MANIFEST.txt``). This workspace completes the isolation for the source files that were left behind.

## To restore (if ever needed)
Move each listed file back to its original relative path shown above.
"@
Set-Content -Path $readme -Value $body
Write-Output ("README=" + $readme)
Write-Output "DONE"
