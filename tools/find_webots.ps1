# Finds webots.exe by itself (v0.16.2). Dot-sourced by setup.ps1 and start_webots.ps1:   . (Join-Path $PSScriptRoot "tools\find_webots.ps1")
# Order: WEBOTS_HOME (environment, then .env), the installer's default folders, the Windows list of installed programs, webots on the PATH,
# then <drive>:\Webots and <drive>:\Program Files\Webots on every drive. Returns the full path of webots.exe, or $null.
function Find-WebotsExe([string]$ProjectRoot) {
    $homes = @()
    if ($env:WEBOTS_HOME) { $homes += $env:WEBOTS_HOME }
    $envFile = Join-Path $ProjectRoot ".env"
    if (Test-Path $envFile) {
        Get-Content $envFile | ForEach-Object {
            if ($_ -match '^\s*WEBOTS_HOME\s*=\s*([^#]+?)\s*(#.*)?$') { $homes += $Matches[1].Trim('"') }
        }
    }
    if ($env:ProgramFiles) { $homes += (Join-Path $env:ProgramFiles "Webots") }
    $homes += "C:\Program Files\Webots"
    if ($env:LOCALAPPDATA) { $homes += (Join-Path $env:LOCALAPPDATA "Programs\Webots") }
    $keys = @("HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*",
              "HKLM:\Software\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*",
              "HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*")
    foreach ($k in $keys) {
        Get-ItemProperty -Path $k -ErrorAction SilentlyContinue |
            Where-Object { $_.PSObject.Properties["DisplayName"] -and "$($_.DisplayName)" -like "Webots*" } |
            ForEach-Object {
                if ($_.PSObject.Properties["InstallLocation"] -and $_.InstallLocation) { $homes += "$($_.InstallLocation)".TrimEnd('\') }
                $p = $_.PSObject.Properties["Inno Setup: App Path"]
                if ($p -and $p.Value) { $homes += "$($p.Value)".TrimEnd('\') }
            }
    }
    # plain strings, not Join-Path: Join-Path fails on a drive letter that does not exist (for example an old WEBOTS_HOME=D:\...)
    $cands = @($homes | Where-Object { $_ } | ForEach-Object { $_.TrimEnd('\') + "\msys64\mingw64\bin\webots.exe" })
    $onPath = Get-Command "webots.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($onPath) { $cands += $onPath.Source }
    Get-PSDrive -PSProvider FileSystem -ErrorAction SilentlyContinue | ForEach-Object {
        foreach ($sub in @("Webots", "Program Files\Webots", "Program Files (x86)\Webots")) {
            $cands += $_.Root.TrimEnd('\') + "\$sub\msys64\mingw64\bin\webots.exe"
        }
    }
    return ($cands | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1)
}

# The Webots folder (the one with lib\controller) for a webots.exe path: <home>\msys64\mingw64\bin\webots.exe -> <home>
function Get-WebotsHome([string]$Exe) {
    if (-not $Exe) { return $null }
    return (Split-Path (Split-Path (Split-Path (Split-Path $Exe -Parent) -Parent) -Parent) -Parent)
}
