__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple

from pxr import Gf

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from src.robots.robot import RobotManager

import torch


class ROS_RobotManager(Node):
    """
    ROS2 node that manages the robots.
    """

    def __init__(self, RM_conf: dict) -> None:
        super().__init__("Robot_spawn_manager_node")
        self.RM = RobotManager(RM_conf)

        self.create_subscription(PoseStamped, "/ZeroGLab/Robots/Spawn", self.spawn_robot, 1)
        self.create_subscription(PoseStamped, "/ZeroGLab/Robots/SpawnFP", self.spawn_floating_platform, 1)
        self.create_subscription(PoseStamped, "/ZeroGLab/Robots/Teleport", self.teleport_robot, 1)
        self.create_subscription(String, "/ZeroGLab/Robots/Reset", self.reset_robot, 1)
        self.create_subscription(Empty, "/ZeroGLab/Robots/ResetAll", self.reset_robots, 1)
        self.create_subscription(PoseStamped, "/ZeroGLab/Robots/Forward", self.apply_forces, 1)

        self.domain_id = 0
        self.modifications: List[Tuple[callable, dict]] = []

    def reset(self) -> None:
        """
        Resets the robots to their initial state.
        """

        self.clear_modifications()
        self.reset_robots(Empty())

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab.
        """

        self.modifications: List[Tuple[callable, dict]] = []

    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab.
        """

        for mod in self.modifications:
            mod[0](**mod[1])
        self.clear_modifications()

    def spawn_floating_platform(self, data: PoseStamped) -> None:
        """
        Spawns floating platform w/o usd

        Args:
            data (String): Name and path of the robot to spawn.
                           Must be in the format: robot_name
        """
        robot_name, usd_path = data.header.frame_id.split(":")
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
        self.modifications.append(
            [
                self.RM.add_floating_platform,
                {"robot_name": robot_name, "p": p, "q": q, "domain_id": self.domain_id}
            ]
        )


    def apply_forces(self, data: PoseStamped) -> None:

        forces = torch.tensor([
                        [1000, 1000, 0.],
                        [0., 0., 0.],
                        [0., 0., 0.],
                        [0., 0., 0.]
                    ])
        positions = torch.tensor([
                        [0.5, 0.5, 0.],
                        [0., 0., 0.],
                        [0., 0., 0.],
                        [0., 0., 0.]
                    ])
        is_global=True
        robot_name="FloatingPlatform"

        self.modifications.append(
            [
                self.RM.apply_forces,
                {"forces": forces, "positions": positions, "is_global": is_global, "robot_name": robot_name}
            ]
        )
        


    def spawn_robot(self, data: PoseStamped) -> None:
        """
        Spawns a robot.

        Args:
            data (String): Name and path of the robot to spawn.
                           Must be in the format: robot_name:usd_path
        """

        assert len(data.header.frame_id.split(":")) == 2, "The data should be in the format: robot_name:usd_path"
        robot_name, usd_path = data.header.frame_id.split(":")
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
        self.modifications.append(
            [
                self.RM.add_robot,
                {"usd_path": usd_path, "robot_name": robot_name, "p": p, "q": q, "domain_id": self.domain_id},
            ]
        )

    def teleport_robot(self, data: PoseStamped) -> None:
        """
        Teleports a robot.

        Args:
            data (Pose): Pose in ROS2 Pose format.
        """

        robot_name = data.header.frame_id
        p = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.modifications.append([self.RM.teleport_robot, {"robot_name": robot_name, "position": p, "orientation": q}])

    def reset_robot(self, data: String) -> None:
        """
        Resets a robot.

        Args:
            data (String): Name of the robot to reset.
        """

        robot_name = data.data
        self.modifications.append([self.RM.reset_robot, {"robot_name": robot_name}])

    def reset_robots(self, data: Empty) -> None:
        """
        Resets all the robots.

        Args:
            data (Int32): Dummy argument.
        """

        self.modifications.append([self.RM.reset_robots, {}])

    def cleanRobots(self) -> None:
        """
        Cleans the robots."""

        self.destroy_node()