from dataclasses import dataclass, asdict
from typing import Any

ALLOWED_TOOLS = {"get_goal_odometry", "set_goal", "describe_image", "navigate_to", "stop_robot"}

@dataclass
class Action:
    tool: str
    args: dict[str, Any]
    purpose: str = ""

@dataclass
class Plan:
    task_id: str
    actions: list[Action]
    cursor: int = 0
    revision: int = 0

    def remaining(self) -> list[Action]:
        return self.actions[self.cursor:]

    def to_dict(self) -> dict[str, Any]:
        return {"task_id": self.task_id, "actions": [asdict(a) for a in self.actions], "cursor": self.cursor, "revision": self.revision}
