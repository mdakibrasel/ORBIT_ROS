# Conformance: `fakesim-grid` (grid motion)

11/11 checks passed.

| check | result | detail |
|---|---|---|
| capabilities | PASS | {'actions': ['move_forward', 'turn', 'clean', 'stop_robot', 'report', 'localize'], 'sensors': ['pose', 'camera'], 'motion': 'grid', 'ground_truth': True, 'action_timeout': True, 'name': 'fakesim-grid'} |
| frame_units | PASS | after +1 m at yaw 0: {'x': 3, 'y': 2, 'yaw_deg': 0, 'room_w': 7, 'room_h': 5}; after left + 1 m: {'x': 3, 'y': 3, 'yaw_deg': 90, 'room_w': 7, 'room_h': 5} |
| turn_convention | PASS | left 90 -> 90, then right 180 -> 270 |
| camera_convention | PASS | view [{'label': 'red box', 'image_x': -0.334, 'apparent_size': 0.316}, {'label': 'blue box', 'image_x': 0.0, 'apparent_size': 0.5}] |
| freshness | PASS | steps 4 -> 6 |
| invalid_args | PASS | status invalid, pose {'x': 2, 'y': 2, 'yaw_deg': 0, 'room_w': 7, 'room_h': 5} |
| collision_stop | PASS | status blocked, true cell (6,2) |
| stop | PASS | status after stop: stopped |
| reset_semantics | PASS | {'x': 2, 'y': 2, 'yaw_deg': 90, 'room_w': 7, 'room_h': 5} vs {'x': 2, 'y': 2, 'yaw_deg': 90, 'room_w': 7, 'room_h': 5} |
| ground_truth_split | PASS | view keys ['apparent_size', 'image_x', 'label'] |
| action_timeout | PASS | status ok, 0.0000 s |
