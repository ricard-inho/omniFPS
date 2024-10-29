__author__ = "Antoine Richard, Junnosuke Kamohara, Ricard Marsal"
__copyright__ = "Copyright 2024-25, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Ricard Marsal"
__email__ = "ricard.marsal@uni.lu"
__status__ = "development"

from typing import Dict, List, Tuple
import numpy as np
import warnings

import omni
from omni.isaac.core.world import World


from src.utils.pxr_utils import createXform
from src.configurations.robot_confs import RobotManagerConf
from src.robots.robot import (
    Robot,
    RobotRigidGroup
)
from src.robots.uni_lu_fp.uni_lu_fp import UniLuFP

class RobotFactory:
    @staticmethod
    def create_robot(
        usd_path: str,
        robot_name: str,
        is_on_nucleus: bool,
        is_ROS2: bool,
        domain_id: int,
        robots_root: str,
        robot_type: str
    ) -> Robot:
        # Decide which type of robot to create based on `robot_type`
        if robot_type == "UniLuFP":
            return UniLuFP(
                usd_path= usd_path,
                robot_name= robot_name,
                robots_root= robots_root,
                is_on_nucleus=is_on_nucleus,
                is_ROS2= is_ROS2,
                domain_id= domain_id,
            )
        else:
            return Robot(
                usd_path= usd_path,
                robot_name= robot_name,
                robots_root= robots_root,
                is_on_nucleus=is_on_nucleus,
                is_ROS2= is_ROS2,
                domain_id= domain_id,
            )


class RobotManager:
    """
    RobotManager class.
    It allows to spawn, reset, teleport robots. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation."""

    def __init__(
        self,
        RM_conf: RobotManagerConf,
    ) -> None:
        """
        Args:
            RM_conf (RobotManagerConf): The configuration of the robot manager.
        """

        self.stage = omni.usd.get_context().get_stage()
        self.RM_conf = RobotManagerConf(**RM_conf)
        self.robot_parameters = self.RM_conf.parameters
        self.uses_nucleus = self.RM_conf.uses_nucleus
        self.is_ROS2 = self.RM_conf.is_ROS2
        self.max_robots = self.RM_conf.max_robots
        self.robots_root = self.RM_conf.robots_root
        createXform(self.stage, self.robots_root)
        self.robots: Dict[str, Robot] = {}
        self.robots_RG: Dict[str, RobotRigidGroup] = {}
        self.num_robots = 0

    def preload_robot(
        self,
        world: World,
    ) -> None:
        """
        Preload the robots in the scene.
        Args:
            world (Usd.Stage): The usd stage scene.
        """
        if len(self.robot_parameters) > 0:
            for robot_parameter in self.robot_parameters:
                self.add_robot(
                    robot_parameter.usd_path,
                    robot_parameter.robot_name,
                    robot_parameter.pose.position,
                    robot_parameter.pose.orientation,
                    robot_parameter.domain_id,
                )
                self.add_RRG(
                    robot_parameter.robot_name,
                    robot_parameter.target_links,
                    world,
                )

    def preload_robot_at_pose(
        self,
        world: World,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
    ) -> None:
        """
        Preload the robots in the scene.
        Args:
            world (Usd.Stage): The usd stage scene.
            position (Tuple[float, float, float]): The position of the robot. (x, y, z)
            orientation (Tuple[float, float, float, float]): The orientation of the robot. (w, x, y, z)
        """
        if len(self.robot_parameters) > 0:
            for robot_parameter in self.robot_parameters:
                self.add_robot(
                    robot_parameter.usd_path,
                    robot_parameter.robot_name,
                    position,
                    orientation,
                    robot_parameter.domain_id,
                )
                self.add_RRG(
                    robot_parameter.robot_name,
                    robot_parameter.target_links,
                    world,
                )

    def add_floating_platform(
        self,
        robot_name: str = None,
        p: Tuple[float, float, float] = [0, 0, 0],
        q: Tuple[float, float, float, float] = [0, 0, 0, 1],
        domain_id: int = None
    ) -> None:
        
        """
        Add floating platform to the scene

        Args:
            robot_name (str): The name of the robot.
            p (Tuple[float, float, float]): The position of the robot. (x, y, z)
            q (Tuple[float, float, float, float]): The orientation of the robot. (w, x, y, z)
            domain_id (int): The domain id of the robot. Not required if the robot is not ROS2 enabled.
        """
        if robot_name[0] != "/":
            robot_name = "/" + robot_name
        if self.num_robots >= self.max_robots:
            pass
        if robot_name in self.robots.keys():
            warnings.warn("Robot already exists. Ignoring request.")
        else:
            self.robots[robot_name] = Robot(
                "/None",
                robot_name,
                is_on_nucleus=self.uses_nucleus,
                is_ROS2=self.is_ROS2,
                domain_id=domain_id,
                robots_root=self.robots_root,
            )

            self.robots[robot_name].load_floating_platform(p,q)
            self.num_robots += 1

    def apply_forces(self, forces, positions, is_global: bool, robot_name: str = None,) -> None:
        if robot_name[0] != "/":
            robot_name = "/" + robot_name

        if robot_name in self.robots.keys():
            self.robots[robot_name].set_forces(forces, positions, is_global)
        else:
            warnings.warn("Robot doesn't exists. Ignoring request.")


    def add_robot(
        self,
        usd_path: str = None,
        robot_name: str = None,
        p: Tuple[float, float, float] = [0, 0, 0],
        q: Tuple[float, float, float, float] = [0, 0, 0, 1],
        domain_id: int = None,
    ) -> None:
        """
        Add a robot to the scene.

        Args:
            usd_path (str): The path of the robot's usd file.
            robot_name (str): The name of the robot.
            p (Tuple[float, float, float]): The position of the robot. (x, y, z)
            q (Tuple[float, float, float, float]): The orientation of the robot. (w, x, y, z)
            domain_id (int): The domain id of the robot. Not required if the robot is not ROS2 enabled.
        """

        if robot_name[0] != "/":
            robot_name = "/" + robot_name
        if self.num_robots >= self.max_robots:
            pass
        else:
            if robot_name in self.robots.keys():
                warnings.warn("Robot already exists. Ignoring request.")
            else:
                robot = RobotFactory.create_robot(
                    usd_path=usd_path,
                    robot_name=robot_name,
                    is_on_nucleus=self.uses_nucleus,
                    is_ROS2=self.is_ROS2,
                    domain_id=domain_id,
                    robots_root=self.robots_root,
                    robot_type=self.RM_conf.robot_type
                )
                self.robots[robot_name] = robot
                self.robots[robot_name].load(p, q)
                self.num_robots += 1

    def add_RRG(
        self,
        robot_name: str = None,
        target_links: List[str] = None,
        world=None,
    ) -> None:
        """
        Add a robot rigid group to the scene.

        Args:
            robot_name (str): The name of the robot.
            target_links (List[str]): List of link names.
            world (Usd.Stage): usd stage scene.
        """
        rrg = RobotRigidGroup(
            self.robots_root,
            robot_name,
            target_links,
        )
        rrg.initialize(world)
        self.robots_RG[robot_name] = rrg

    def reset_robots(self) -> None:
        """
        Reset all the robots to their original position.
        """
        for robot in self.robots.keys():
            self.robots[robot].reset()

    def reset_robot(self, robot_name: str = None) -> None:
        """
        Reset a specific robot to its original position.

        Args:
            robot_name (str): The name of the robot.
        """

        if robot_name in self.robots.keys():
            self.robots[robot_name].reset()
        else:
            warnings.warn("Robot does not exist. Ignoring request.")

    def teleport_robot(
        self, robot_name: str = None, position: np.ndarray = None, orientation: np.ndarray = None
    ) -> None:
        """
        Teleport a specific robot to a specific position and orientation.

        Args:
            robot_name (str): The name of the robot.
        """
        if robot_name in self.robots.keys():
            self.robots[robot_name].teleport(position, orientation)
        else:
            warnings.warn(f"Robot {robot_name} does not exist. Ignoring request.")
            print("available robots: ", self.robots.keys())

    def apply_thrusters_forces(self, robot_name: str=None, function_name: str=None)-> None:
        if robot_name in self.robots.keys():
            robot = self.robots.get(robot_name)
            if robot and hasattr(robot, function_name):
                getattr(robot, function_name)()
            else:
                print(f"{robot_name} does not have the function {function_name}")
        else:
            warnings.warn(f"Robot {robot_name} does not exist. Ignoring request.")
            print("available robots: ", self.robots.keys())

    def set_dof_pos(self, robot_name:str=None, function_name:str=None, position:np.ndarray=None, orientation:np.ndarray=None)-> None:
        if robot_name in self.robots.keys():
            robot = self.robots.get(robot_name)
            if robot and hasattr(robot, function_name):
                getattr(robot, function_name)(position, orientation)
            else:
                print(f"{robot_name} does not have the function {function_name}")
        else:
            warnings.warn(f"Robot {robot_name} does not exist. Ignoring request.")
            print("available robots: ", self.robots.keys())