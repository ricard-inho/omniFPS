
import os
from dataclasses import dataclass, field
from typing import List, Dict


@dataclass
class Pose:
    position: List[float] = field(default_factory=list)
    orientation: List[float] = field(default_factory=list)


@dataclass
class RobotParameters:
    robot_name: str = field(default_factory=str)
    usd_path: str = field(default_factory=str)
    pose: Pose = field(default_factory=dict)
    domain_id: int = field(default_factory=int)
    target_links: List[str] = field(default_factory=list)

    def __post_init__(self):
        self.usd_path = os.path.join(os.getcwd(), self.usd_path)
        self.pose = Pose(**self.pose)


@dataclass
class RobotManagerConf:
    uses_nucleus: str = False
    is_ROS2: str = False
    max_robots: int = 5
    robots_root: str = "/Robots"
    name: str = "MyRobot"
    robot_type: str = "UniLuFP"
    parameters: List[RobotParameters] = field(default_factory=list)

    def __post_init__(self):
        if self.parameters:
            assert len(self.parameters) <= self.max_robots, "number of robots to register should not exeed max_robots"
            self.parameters = [RobotParameters(**param) for param in self.parameters]
        else:
            self.parameters = []