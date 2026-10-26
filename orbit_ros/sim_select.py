"""Simulator selection for the episode runners (v0.14, ORBIT-PEL milestone M5).

    world = in_simulator(world, "fake", "grid")      # World2D -> the same world inside FakeSim (or Webots / ROS 2)

"world2d" returns the world unchanged (the default of every runner, so existing results are untouched). "fake" mirrors it into the
pure-Python FakeSim used by the unit tests. "webots" and "ros2" need the real simulator (Webots extern controller; ROS 2 Jazzy +
Gazebo with configs/ros2_jazzy_tb3.json) and are imported only when chosen. One simulator object is reused for every episode, as in
run_webots_c1.py; in continuous mode its lidar is calibrated once.
"""
from __future__ import annotations

SIMS = ("world2d", "fake", "webots", "ros2")
_cache = {}

def ensure_webots_controller() -> str | None:
    """v0.14.2: make Webots' Python `controller` module importable, so `--sim webots` works from a normal terminal or VS Code (F5)
    as an extern controller, without Webots' `webots-controller` launcher. Inside Webots (or under the launcher) the module is already
    importable and nothing changes. Otherwise the Webots folder is found as run_extern.py finds it (WEBOTS_HOME in .env or the
    environment, then the usual install folders) and its controller library is put on the paths. Returns the Webots folder, or None
    when nothing had to be done. Raises RuntimeError with the fix when Webots cannot be found."""
    try:
        import controller  # noqa: F401
        return None
    except ImportError:
        pass
    from .env_config import load_env
    from .webots_env import setup_webots
    load_env()
    home = setup_webots()
    try:
        import controller  # noqa: F401
    except ImportError as e:
        raise RuntimeError(f"Webots found at {home}, but its Python controller module cannot be loaded ({e}). Check that this is "
                           "Webots R2025a and that your Python version is one Webots supports.") from e
    print(f"[webots] using Webots at {home} (extern controller): start the NAO house with start_webots.ps1 first")
    return home

def get_sim(name: str, motion: str = "grid", ros2_config: str | None = None):
    key = (name, motion, ros2_config)
    if key not in _cache:
        from .webots_adapter import FakeSim, calibrate_lidar
        if name == "fake": sim = FakeSim()
        elif name == "webots":
            ensure_webots_controller()
            from .webots_adapter import WebotsSim; sim = WebotsSim()
        elif name == "ros2":
            from .ros2_adapter import load_ros2_config, Ros2Sim; sim = Ros2Sim(**load_ros2_config(ros2_config))
        else: raise ValueError(f"unknown simulator {name}; choose from {SIMS}")
        if motion == "continuous": calibrate_lidar(sim)
        _cache[key] = sim
    return _cache[key]

def in_simulator(world, sim="world2d", motion: str = "grid", perception: str = "camera", ros2_config: str | None = None):
    """sim: a name from SIMS, or any SimBackend instance (your own adapter, see docs/PEL_ADAPTER_GUIDE.md)."""
    if not isinstance(sim, str):
        from .webots_adapter import WebotsWorld2D, calibrate_lidar
        if motion == "continuous" and not getattr(sim, "_orbit_lidar_calibrated", False):
            calibrate_lidar(sim); sim._orbit_lidar_calibrated = True
        return WebotsWorld2D.adopt(world, sim, perception, motion)
    if sim == "world2d":
        if motion != "grid": raise ValueError("world2d is a grid world: use --motion grid (continuous motion needs fake, webots or ros2)")
        return world
    from .webots_adapter import WebotsWorld2D
    return WebotsWorld2D.adopt(world, get_sim(sim, motion, ros2_config), perception, motion)
