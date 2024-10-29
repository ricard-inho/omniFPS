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
import omni.isaac.dynamic_control as dc_module

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

    def apply_thrusters_forces(self):
        pass

    def set_dof_pos(self, position:np.ndarray=None, orientation:np.ndarray=None)->None:
        art = self.dc.get_articulation("/Robots/FloatingPlatform")

        dof_ptr_x = self.dc.find_articulation_dof(art, "fp_world_joint_x")
        dof_ptr_y = self.dc.find_articulation_dof(art, "fp_world_joint_y")
        dof_ptr_z = self.dc.find_articulation_dof(art, "fp_world_joint_z")

        self.dc.set_dof_position(dof_ptr_x, 2.0)
        self.dc.set_dof_position(dof_ptr_y, -1.5)
        self.dc.set_dof_position(dof_ptr_z, 0.3)

        self.dc.set_dof_velocity(dof_ptr_x, 0.0)
        self.dc.set_dof_velocity(dof_ptr_y, 0.0)
        self.dc.set_dof_velocity(dof_ptr_z, 0.0)

    def reset(self) -> None:
        print(f"reset pos {self.reset_position}")
        self.set_dof_pos(position=self.reset_position)

    # def load_floating_platform(self, position: np.ndarray, orientation: np.ndarray) -> None:
    #     """
    #     self.stage = omni.usd.get_context().get_stage()
    #     self.set_reset_pose(position, orientation)

    #     position = Gf.Vec3d(position)
    #     rotation = Gf.Quatd(orientation[0],orientation[1],orientation[2],orientation[3])
    #     scale = Gf.Vec3d(1,1,1)

    #     prefix = "/Robot/FloatingPlatform"
    #     obj_prim, prim_path = createXform(self.stage, prefix)
    #     xform = UsdGeom.Xformable(obj_prim)
    #     addDefaultOps(xform)
    #     setDefaultOpsTyped(xform, position, rotation, scale)

    #     cylinder_path = prim_path + "/Cylinder"
    #     cylinder_prim = UsdGeom.Cylinder.Define(self.stage, cylinder_path)
    #     cylinder_prim.CreateRadiusAttr(0.25)
    #     cylinder_prim.CreateHeightAttr(0.5)
    #     cylinder_xform = UsdGeom.Xformable(cylinder_prim)
    #     cylinder_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    #     cylinder_xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))
    #     cylinder_xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
    #     UsdPhysics.RigidBodyAPI.Apply(cylinder_prim.GetPrim())
    #     UsdPhysics.CollisionAPI.Apply(cylinder_prim.GetPrim())
    #     mass_api = UsdPhysics.MassAPI.Apply(cylinder_prim.GetPrim())
    #     mass_api.CreateMassAttr().Set(5.0)
    #     mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3d([0, 0, 0]))
    #     # obj_prim.GetReferences().AddReference(path)
    #     # obj_prim.SetInstanceable(True)
    #     """

    #     with open("cfg/robot/modularfloatingplatform.yaml", "r") as file:
    #         cfg = yaml.safe_load(file)

    #     name = cfg["robots_settings"]["name"]
    #     fp = ModularFloatingPlatform(
    #         f"/Robots/{name}", 
    #         cfg=cfg, 
    #         translation=[position],
    #         orientation=[orientation[1], orientation[2], orientation[3], orientation[0]]
    #     )
        
    #     root_path = f"/Robots/{name}"
    #     self.platform = ModularFloatingPlatformView(
    #         prim_paths_expr=root_path,
    #         name="modular_floating_platform_view",
    #         track_contact_force=True,
    #     )

    #     stage = omni.usd.get_context().get_stage()
    #     world = World(stage)
    #     world.reset()
    #     self.platform.initialize()
    #     world.scene.add(self.platform)
    #     world.reset()


    # def set_forces(self, forces, positions, is_global: bool) -> None:

    #     articulation = self.dc.get_articulation("/Robots/FloatingPlatform")

    #     v_thruster_0 = self.dc.find_articulation_body(articulation, "v_thruster_0")
    #     force_vector = forces[0]
    #     position = positions[0] 
    #     success = self.dc.apply_body_force(v_thruster_0, force_vector, position, False)
    #     print(f"Thruster 0-> Force: {force_vector}, position: {position}")

    #     v_thruster_1 = self.dc.find_articulation_body(articulation, "v_thruster_1")
    #     force_vector = forces[1] 
    #     position = positions[1] 
    #     success = self.dc.apply_body_force(v_thruster_1, force_vector, position, False)
    #     print(f"Thruster 1-> Force: {force_vector}, position: {position}")

    #     v_thruster_2 = self.dc.find_articulation_body(articulation, "v_thruster_2")
    #     force_vector = forces[2]
    #     position = positions[2] 
    #     success = self.dc.apply_body_force(v_thruster_2, force_vector, position, False)
    #     print(f"Thruster 2-> Force: {force_vector}, position: {position}")

    #     v_thruster_3 = self.dc.find_articulation_body(articulation, "v_thruster_3")
    #     force_vector = forces[3] 
    #     position = positions[3]
    #     success = self.dc.apply_body_force(v_thruster_3, force_vector, position, False)
    #     print(f"Thruster 3-> Force: {force_vector}, position: {position}")

    #     print("##########################################################################check local frame")

    #     # self.platform.thrusters.apply_forces_and_torques_at_pos(
    #     #     forces=forces, positions=positions, is_global=is_global
        # )

        