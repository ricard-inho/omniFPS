__author__ = "Antoine Richard, Ricard Marsal"
__copyright__ = "Copyright 2024-25, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Ricard Marsal"
__email__ = "ricard.marsal@uni.lu"
__status__ = "development"

from typing import List, Tuple, Dict

from pxr import Gf

from std_msgs.msg import String, Empty, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from src.robots.robot_manager import RobotManager

import torch


class ROS_RobotManager(Node):
    """
    ROS2 node that manages the robots.
    """

    def __init__(self, RM_conf: dict) -> None:
        super().__init__("Robot_spawn_manager_node")
        self.RM = RobotManager(RM_conf)

        self.create_subscription(String, "/omniFPS/Robots/Reset", self.reset_robot, 1)
        self.create_subscription(Empty, "/omniFPS/Robots/ResetAll", self.reset_robots, 1)
        self.create_subscription(Float32MultiArray, "/omniFPS/Robots/FloatingPlatform/thrusters/input", self.set_robot_forces, 1)

        self.domain_id = 0
        self.modifications: List[Tuple[callable, dict]] = []
        self.persistent_modifications: Dict[str, dict] = {}

    def reset(self) -> None:
        """
        Resets the robots to their initial state.
        """

        self.clear_modifications()
        self.reset_robots(Empty())
        self.RM.reset_robots()

    def clear_modifications(self) -> None:
        """
        Clears the list of modifications to be applied to the lab.
        """

        self.modifications: List[Tuple[callable, dict]] = []
        # self.persistent_modifications.clear()

    def apply_modifications(self) -> None:
        """
        Applies the list of modifications to the lab.
        """

        for mod in self.modifications:
            mod[0](**mod[1])

        for mod in self.persistent_modifications.values():
            self.RM.custom_funct(**mod)

        self.clear_modifications()        

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

    def set_robot_forces(self, data: Float32MultiArray) -> None:
        """
        Sets forces for a specific robot and stores it in `last_forces_command`.
        """
        robot_name = "/FloatingPlatform"
        forces = data 
        
        # Use custom_funct to set last forces on the specified robot
        self.RM.custom_funct(robot_name, "set_forces_command", forces=forces)
        
        # Schedule force application for the robot in the modifications list
        self.modifications.append([self.RM.custom_funct, {"robot_name": robot_name, "function_name": "apply_forces_command"}])
        self.persistent_modifications[robot_name] = {"robot_name": robot_name, "function_name": "apply_forces_command"}