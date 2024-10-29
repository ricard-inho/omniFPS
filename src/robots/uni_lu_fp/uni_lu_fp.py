__author__ = "Ricard Marsal"
__copyright__ = "Copyright 2024-25, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Ricard Marsal"
__email__ = "ricard.marsal@uni.lu"
__status__ = "development"


from src.robots.robot import Robot
import numpy as np
from pxr import Gf

class UniLuFP(Robot):
    def __init__(self, usd_path, robot_name, robots_root, is_on_nucleus, is_ROS2, domain_id):
        print("Creattinngngg hehereeee")
        super().__init__(
            usd_path= usd_path,
            robot_name= robot_name,
            robots_root= robots_root,
            is_on_nucleus=is_on_nucleus,
            is_ROS2= is_ROS2,
            domain_id= domain_id,
        )

    def apply_thrusters_forces(self):
        pass

    def set_dof_pos(self, position:np.ndarray=None, orientation:np.ndarray=None)->None:
        art = self.dc.get_articulation("/Robots/FloatingPlatform")
        dof_ptr_x = self.dc.find_articulation_dof(art, "fp_world_joint_x")
        dof_ptr_y = self.dc.find_articulation_dof(art, "fp_world_joint_y")
        self.dc.wake_up_articulation(art)
        self.dc.set_dof_position_target(dof_ptr_x, 5.0)
        self.dc.set_dof_position_target(dof_ptr_y, -5.0)
        
        num_joints = self.dc.get_articulation_joint_count(art)
        # breakpoint()
        # self.dc.find_articulation_body(art, "v_thruster_0")
        # dof_ptr = self.dc.find_articulation_dof(art, "core/body")
        # self.dc.set_dof_position()
        print(f"doooof{position},{dof_ptr_x}, {dof_ptr_y}")