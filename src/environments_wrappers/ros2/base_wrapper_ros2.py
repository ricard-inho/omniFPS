__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# System imports
from typing import List, Tuple

# Custom libs
import src.environments.rendering as rndr

# Loads ROS2 dependent libraries
from std_msgs.msg import Bool, Float32, ColorRGBA, Int8, Int32, String, Empty
from rclpy.executors import SingleThreadedExecutor as Executor
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
import rclpy


class ROS_BaseManager(Node):
    """
    ROS2 node that manages the lab environment"""

    def __init__(
        self,
        environment_cfg: dict = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab manager.

        Args:
            environment_cfg (dict): Environment configuration.
            flares_cfg (dict): Flares configuration.
            **kwargs: Additional arguments."""

        super().__init__("Lab_controller_node")
        self.trigger_reset = False

        self.modifications: List[Tuple[callable, dict]] = []

    def periodic_update(self, dt: float) -> None:
        """
        Updates the lab.

        Args:
            dt (float): Time step.
        """

        raise NotImplementedError

    def reset(self) -> None:
        """
        Resets the lab to its initial state.
        """

        raise NotImplementedError

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

    def clean_scene(self):
        """
        Cleans the scene.
        """

        self.destroy_node()

    def monitor_thread_is_alive(self):
        """
        Checks if the monitor thread is alive.
        """

        return True

    def get_wait_for_threads(self):
        """
        Returns the list of waiting threads.
        """

        return []