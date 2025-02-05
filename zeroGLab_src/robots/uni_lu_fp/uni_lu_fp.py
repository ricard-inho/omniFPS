


from zeroGLab_src.robots.robot import Robot
import numpy as np
from pxr import Gf
import omni.isaac.dynamic_control as dc_module
from std_msgs.msg import Float32MultiArray
from typing import List

class UniLuFP(Robot):
    def __init__(self, usd_path, robot_name, robots_root, is_on_nucleus, is_ROS2, domain_id):
        super().__init__(
            usd_path= usd_path,
            robot_name= robot_name,
            robots_root= robots_root,
            is_on_nucleus=is_on_nucleus,
            is_ROS2= is_ROS2,
            domain_id= domain_id,
        )

        #Forces to apply to the thrusters 
        self.forces_command = None
        
        self.thrusters_directions = [
            (-1, 1),   # Thruster 0
            (1, -1),   # Thruster 1
            (-1, -1),  # Thruster 2
            (1, 1),    # Thruster 3
            (1, -1),   # Thruster 4
            (-1, 1),   # Thruster 5
            (1, 1),    # Thruster 6
            (-1, -1)   # Thruster 7
        ]
        #Thruster position
        self.thrusters_pos = [
            [0.155, 0.155, 0.0],    # Thruster 0,1
            [-0.155, 0.155, 0.0],   # Thruster 2,3
            [-0.155, -0.155, 0.0],  # Thruster 4,5
            [0.155, -0.155, 0.0]    # Thruster 6,7
        ]

    def define_articulations(self)->None:
        self.articulation = self.dc.get_articulation("/Robots/FloatingPlatform")
        #Joints
        self.dof_ptr_x = self.dc.find_articulation_dof(self.articulation, "fp_world_joint_x")
        self.dof_ptr_y = self.dc.find_articulation_dof(self.articulation, "fp_world_joint_y")
        self.dof_ptr_z = self.dc.find_articulation_dof(self.articulation, "fp_world_joint_z")
        #Thrusters
        self.v_thruster_0 = self.dc.find_articulation_body(self.articulation, "v_thruster_0")
        self.v_thruster_1 = self.dc.find_articulation_body(self.articulation, "v_thruster_1")
        self.v_thruster_2 = self.dc.find_articulation_body(self.articulation, "v_thruster_2")
        self.v_thruster_3 = self.dc.find_articulation_body(self.articulation, "v_thruster_3")
        self.v_thruster_4 = self.dc.find_articulation_body(self.articulation, "v_thruster_4")
        self.v_thruster_5 = self.dc.find_articulation_body(self.articulation, "v_thruster_5")
        self.v_thruster_6 = self.dc.find_articulation_body(self.articulation, "v_thruster_6")
        self.v_thruster_7 = self.dc.find_articulation_body(self.articulation, "v_thruster_7")
        

    def set_dof_pos(self)->None:
        """
        Sets the position and velocity of degrees of freedom (DOFs) for the FloatingPlatform articulation.
        """
        self.dc.set_dof_position(self.dof_ptr_x, 2.5)
        self.dc.set_dof_position(self.dof_ptr_y, -1)
        self.dc.set_dof_position(self.dof_ptr_z, 0.0)

        self.dc.set_dof_velocity(self.dof_ptr_x, 0.0)
        self.dc.set_dof_velocity(self.dof_ptr_y, 0.0)
        self.dc.set_dof_velocity(self.dof_ptr_z, 0.0)

    def reset(self) -> None:
        """
        Reset the robot to its original position and orientation.
        """
        self.set_dof_pos()
        

    def set_forces_command(self, comanded_forces: List) -> None:
        """
        Sets the last forces to apply to the robot.
        """
        self.forces_command = comanded_forces

    def apply_forces_command(self) -> None:
        """
        Applies the last set forces to the robot's thrusters.
        """
        if self.forces_command:
            if self.forces_command[0] == 1: #Air bearing
                forces = self._map_forces(commands=self.forces_command[1:]) 
                self.dc.apply_body_force(self.v_thruster_0, forces[0], self.thrusters_pos[0], False)
                self.dc.apply_body_force(self.v_thruster_1, forces[1], self.thrusters_pos[0], False)
                self.dc.apply_body_force(self.v_thruster_2, forces[2], self.thrusters_pos[1], False)
                self.dc.apply_body_force(self.v_thruster_3, forces[3], self.thrusters_pos[1], False)
                self.dc.apply_body_force(self.v_thruster_4, forces[4], self.thrusters_pos[2], False)
                self.dc.apply_body_force(self.v_thruster_5, forces[5], self.thrusters_pos[2], False)
                self.dc.apply_body_force(self.v_thruster_6, forces[6], self.thrusters_pos[3], False)
                self.dc.apply_body_force(self.v_thruster_7, forces[7], self.thrusters_pos[3], False)
        else:
            print("Forces Not defined")

    def _map_forces(self, commands)->List:
        forces = []
        num_open_thrusters = max(commands.count(1), 1) #Aboids division by 0
        for i, force in enumerate(commands):
            forces.append(
                [
                    force * (1/num_open_thrusters) * self.thrusters_directions[i][0], 
                    force * (1/num_open_thrusters) * self.thrusters_directions[i][1], 
                    0.0
                ]
            )
        return forces