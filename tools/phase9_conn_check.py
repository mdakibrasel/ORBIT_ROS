"""Pre-flight connectivity: one fresh-process live Webots connect + true pose read. Permitted check, not a benchmark."""
import sys, time
sys.path.insert(0, ".")
from orbit_ros.humanoid import HumanoidSession
t0 = time.time()
s = HumanoidSession(sim="webots", log_dir=False, memory=False)
s.load_scene("living_room")
cs = s.connection_status()
d = s.robot_call("get_pose", {})["data"]
print(f"CONN connected={cs['connected']} backend={cs['backend']} port={cs.get('controller_port')} uses_1240={cs['uses_port_1240']} "
      f"latency_s={time.time()-t0:.2f}")
print(f"POSE x={d['x']} y={d['y']} yaw={d['yaw_deg']} room={d['room']}")
print("CONN_CHECK: OK" if cs["connected"] and not cs["uses_port_1240"] else "CONN_CHECK: FAIL")
