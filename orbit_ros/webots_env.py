"""Find a Webots installation and make its Python `controller` module importable from a normal Python process (VS Code, extern controller).
Inside Webots itself (a normal controller) none of this is needed."""
from __future__ import annotations
import os, sys

def _registry_install_folders():
    """Webots folders from the Windows list of installed programs (both all-users and current-user installs)."""
    try: import winreg
    except ImportError: return []
    out, key = [], r"Software\Microsoft\Windows\CurrentVersion\Uninstall"
    for hive, sub in ((winreg.HKEY_LOCAL_MACHINE, key), (winreg.HKEY_LOCAL_MACHINE, key.replace("Software", "Software\\WOW6432Node", 1)),
                      (winreg.HKEY_CURRENT_USER, key)):
        try: root = winreg.OpenKey(hive, sub)
        except OSError: continue
        for i in range(winreg.QueryInfoKey(root)[0]):
            try:
                k = winreg.OpenKey(root, winreg.EnumKey(root, i))
                if str(winreg.QueryValueEx(k, "DisplayName")[0]).lower().startswith("webots"):
                    for name in ("InstallLocation", "Inno Setup: App Path"):
                        try: out.append(str(winreg.QueryValueEx(k, name)[0]).rstrip("\\/"))
                        except OSError: pass
            except OSError: continue
    return out

def _home_from_exe(exe):
    """webots.exe is in <home>\\msys64\\mingw64\\bin on Windows, <home>/webots on Linux/macOS (or a launcher link to it)."""
    if not exe: return ""
    d = os.path.dirname(os.path.realpath(exe))
    parts = d.replace("\\", "/").split("/")
    if len(parts) >= 3 and [x.lower() for x in parts[-3:]] in (["msys64", "mingw64", "bin"], ["msys64", "ucrt64", "bin"]): return os.path.dirname(os.path.dirname(os.path.dirname(d)))
    return d

def _drive_folders(env):
    """Webots or Program Files\\Webots on every drive letter that exists (D:\\Webots, E:\\Program Files\\Webots, ...)."""
    out = []
    for letter in "CDEFGHIJ":
        root = letter + ":\\"
        if os.path.isdir(root): out += [root + "Webots", root + r"Program Files\Webots", root + r"Program Files (x86)\Webots"]
    return out

def default_folders(env):
    """Where the Webots installer puts Webots when nothing is changed."""
    if os.name == "nt":
        return [os.path.join(env.get("ProgramFiles", r"C:\Program Files"), "Webots"), os.path.join(env.get("LOCALAPPDATA", ""), "Programs", "Webots")]
    return ["/usr/local/webots", "/opt/webots", "/snap/webots/current/usr/share/webots", "/Applications/Webots.app/Contents"]

def webots_candidates(env=None, defaults=None, registry=None, which=None, drives=None):
    """Places looked at, in order: WEBOTS_HOME, the installer's default folders, the Windows list of installed programs, webots on the
    PATH, then Webots / Program Files\\Webots on every drive. The hooks defaults/registry/which/drives exist for the tests."""
    import shutil
    env = os.environ if env is None else env
    cands = [env.get("WEBOTS_HOME", "")] + (defaults or default_folders)(env)
    cands += (registry or _registry_install_folders)()
    cands.append(_home_from_exe((which or shutil.which)("webots")))
    if os.name == "nt" or drives: cands += (drives or _drive_folders)(env)
    return [c for c in cands if c]

def find_webots_home(env=None, **hooks):
    for c in webots_candidates(env, **hooks):
        if os.path.isdir(os.path.join(c, "lib", "controller", "python")): return c
    return None

def setup_webots(home=None):
    """Adds WEBOTS_HOME/lib/controller/python to sys.path and the DLL folders to the loader path (Windows). Returns the home folder."""
    home = home or find_webots_home()
    if not home:
        raise RuntimeError("Webots not found (looked at WEBOTS_HOME, the default install folders, the Windows list of installed programs, the PATH and "
                           "<drive>:\\Webots on every drive). Install Webots R2025a, or set WEBOTS_HOME in .env / the environment (folder that contains 'lib\\controller').")
    os.environ["WEBOTS_HOME"] = home
    py = os.path.join(home, "lib", "controller", "python")
    if py not in sys.path: sys.path.insert(0, py)
    libdir = os.path.join(home, "lib", "controller")
    os.environ["PATH"] = os.pathsep.join([libdir] + [os.path.join(home, "msys64", sub, "bin") for sub in ("mingw64", "ucrt64")] + [os.environ.get("PATH", "")])
    if hasattr(os, "add_dll_directory"):                       # Python >= 3.8 on Windows ignores PATH for DLLs
        for d in [libdir] + [os.path.join(home, "msys64", sub, "bin") for sub in ("mingw64", "ucrt64")]:
            if os.path.isdir(d): os.add_dll_directory(d)
    return home
