
from typing import Dict, List, Tuple
import numpy as np
import warnings
import os
import yaml

import omni
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.world import World
import omni.graph.core as og
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.transformations import (
    get_relative_transform,
    pose_from_tf_matrix,
)
from omni.isaac.core.utils.rotations import quat_to_rot_matrix
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.prims import RigidPrim, RigidPrimView
from pxr import Gf, UsdGeom, Usd, UsdPhysics, PhysxSchema

from zeroGLab_src.utils.pxr_utils import (
    createObject,
    createXform,
    addDefaultOps,
    setDefaultOpsTyped,
)

# from zeroGLab_src.configurations.robot_confs import RobotManagerConf
# from zeroGLab_src.robots.articulations.modular_floating_platform import ModularFloatingPlatform
# from zeroGLab_src.robots.articulations.views.modular_floating_platform_view import ModularFloatingPlatformView

class Robot:
    """
    Robot class.
    It allows to spawn, reset, teleport a robot. It also allows to automatically add namespaces to topics,
    and tfs to enable multi-robot operation.
    """

    def __init__(
        self,
        usd_path: str,
        robot_name: str,
        robots_root: str = "/Robots",
        is_on_nucleus: bool = False,
        is_ROS2: bool = False,
        domain_id: int = 0,
    ) -> None:
        """
        Args:
            usd_path (str): The path of the robot's usd file.
            robot_name (str): The name of the robot.
            robots_root (str, optional): The root path of the robots. Defaults to "/Robots".
            is_on_nucleus (bool, optional): Whether the robots are loaded from the nucleus or not. Defaults to False.
            is_ROS2 (bool, optional): Whether the robots are ROS2 enabled or not. Defaults to False.
            domain_id (int, optional): The domain id of the robot. Defaults to 0."""

        self.stage: Usd.Stage = omni.usd.get_context().get_stage()
        self.usd_path = str(usd_path)
        self.robots_root = robots_root
        self.robot_name = robot_name
        self.robot_path = os.path.join(self.robots_root, self.robot_name.strip("/"))
        self.is_on_nucleus = is_on_nucleus
        self.is_ROS2 = is_ROS2
        self.domain_id = int(domain_id)
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.root_body_id = None

    def get_root_rigid_body_path(self) -> None:
        """
        Get the root rigid body path of the robot.
        """

        art = self.dc.get_articulation(self.robot_path)
        self.root_body_id = self.dc.get_articulation_root_body(art)

    def edit_graphs(self) -> None:
        """
        Edit the graphs of the robot to add namespaces to topics and tfs.
        """

        selected_paths = []
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(self.robot_path)):
            l = [attr for attr in prim.GetAttributes() if attr.GetName().split(":")[0] == "graph"]
            if l:
                selected_paths.append(prim.GetPath())

        for path in selected_paths:
            prim = self.stage.GetPrimAtPath(path)
            prim.GetAttribute("graph:variable:Namespace").Set(self.robot_name)
            if self.is_ROS2:
                prim.GetAttribute("graph:variable:Context").Set(self.domain_id)


    def load(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Load the robot in the scene, and automatically edit its graphs.

        Args:
            position (np.ndarray): The position of the robot.
            orientation (np.ndarray): The orientation of the robot.
        """

        self.stage = omni.usd.get_context().get_stage()
        self.set_reset_pose(position, orientation)
        if self.is_on_nucleus:
            nucleus = get_assets_root_path()
            self.usd_path = os.path.join(nucleus, self.usd_path)
        createObject(
            self.robot_path,
            self.stage,
            self.usd_path,
            is_instance=False,
            position=Gf.Vec3d(*position),
            rotation=Gf.Quatd(*orientation),
        )
        self.edit_graphs()

    def get_pose(self) -> List[float]:
        """
        Get the pose of the robot.
        Returns:
            List[float]: The pose of the robot. (x, y, z), (qx, qy, qz, qw)
        """
        if self.root_body_id is None:
            self.get_root_rigid_body_path()
        pose = self.dc.get_rigid_body_pose(self.root_body_id)
        return pose.p, pose.r

    def set_reset_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Set the reset pose of the robot.

        Args:
            position (np.ndarray): The position of the robot.
            orientation (np.ndarray): The orientation of the robot.
        """

        self.reset_position = position
        self.reset_orientation = orientation

    
class RobotRigidGroup:
    """
    Class which deals with rigidprims and rigidprimview of a single robot.
    It is used to retrieve world pose, and contact forces, or apply force/torque.
    """

    def __init__(self, root_path: str = "/Robots", robot_name: str = None, target_links: List[str] = None):
        """
        Args:
            root_path (str): The root path of the robots.
            robot_name (str): The name of the robot.
            target_links (List[str]): List of link names.
        """

        self.root_path = root_path
        self.robot_name = robot_name
        self.target_links = target_links
        self.prims = []
        self.prim_views = []

    def initialize(self, world: World) -> None:
        """
        Initialize the rigidprims and rigidprimviews of the robot.

        Args:
            world (World): A Omni.isaac.core.world.World object.
        """

        world.reset()
        if len(self.target_links) > 0:
            for target_link in self.target_links:
                rigid_prim = RigidPrim(
                    prim_path=os.path.join(self.root_path, self.robot_name, target_link),
                    name=f"{self.robot_name}/{target_link}",
                )
                rigid_prim_view = RigidPrimView(
                    prim_paths_expr=os.path.join(self.root_path, self.robot_name, target_link),
                    name=f"{self.robot_name}/{target_link}_view",
                    track_contact_forces=True,
                )
                rigid_prim_view.initialize()
                self.prims.append(rigid_prim)
                self.prim_views.append(rigid_prim_view)
        world.reset()

    def get_world_poses(self) -> np.ndarray:
        """
        Returns the world pose matrix of target links.

        Returns:
            pose (np.ndarray): The world pose matrix of target links.
        """

        n_links = len(self.target_links)
        pose = np.zeros((n_links, 4, 4))
        for i, prim in enumerate(self.prims):
            position, orientation = prim.get_world_pose()
            orientation = quat_to_rot_matrix(orientation)
            pose[i, :3, 3] = 1
            pose[i, :3, :3] = orientation
            pose[i, :3, 3] = position
        return pose

    def get_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns the pose of target links.

        Returns:
            positions (np.ndarray): The position of target links. (x, y, z)
            orientations (np.ndarray): The orientation of target links. (w, x, y, z)
        """

        n_links = len(self.target_links)
        positions = np.zeros((n_links, 3))
        orientations = np.zeros((n_links, 4))
        for i, prim in enumerate(self.prims):
            position, orientation = prim.get_world_pose()
            positions[i, :] = position
            orientations[i, :] = orientation
        return positions, orientations

    def get_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns the linear/angular velocity of target links.

        Returns:
            linear_velocities (np.ndarray): The linear velocity of target links.
            angular_velocities (np.ndarray): The angular velocity of target links.
        """

        n_links = len(self.target_links)
        linear_velocities = np.zeros((n_links, 3))
        angular_velocities = np.zeros((n_links, 3))
        for i, prim in enumerate(self.prims):
            linear_velocity, angular_velocity = prim.get_velocities()
            linear_velocities[i, :] = linear_velocity
            angular_velocities[i, :] = angular_velocity
        return linear_velocities, angular_velocities

    def get_net_contact_forces(self) -> np.ndarray:
        """
        Returns net contact forces on each target link.

        Returns:
            contact_forces (np.ndarray): The net contact forces on each target link.
        """

        n_links = len(self.target_links)
        contact_forces = np.zeros((n_links, 3))
        for i, prim_view in enumerate(self.prim_views):
            contact_force = prim_view.get_net_contact_forces().squeeze()
            contact_forces[i, :] = contact_force
        return contact_forces

    def apply_force_torque(self, forces: np.ndarray, torques: np.ndarray) -> None:
        """
        Apply force and torque (defined in local body frame) to body frame of the four wheels.

        Args:
            forces (np.ndarray): The forces to apply to the body origin of the four wheels.
                                 (Fx, Fy, Fz) = (F_DP, F_S, F_N)
            torques (np.ndarray): The torques to apply to the body origin of the four wheels.
                                 (Mx, My, Mz0 = (M_O,-M_R, M_S)
        """

        n_links = len(self.target_links)
        assert forces.shape[0] == n_links, "given force does not have matching shape."
        assert torques.shape[0] == n_links, "given torque does not have matching shape."
        for i, prim_view in enumerate(self.prim_views):
            prim_view.apply_forces_and_torques_at_pos(forces=forces[i], torques=torques[i], is_global=False)