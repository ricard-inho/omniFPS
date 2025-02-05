
from typing import List, Tuple, Dict
import numpy as np

from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
import omni

from pxr import UsdGeom, UsdLux, Gf, Usd, UsdPhysics

from zeroGLab_src.configurations.environments import ZeroGLabConf
from zeroGLab_src.environments.base_env import BaseEnv
from zeroGLab_src.robots.robot_manager import RobotManager
from assets import get_assets_path


class ZeroGLabController(BaseEnv):
    """
    This class is used to control the environment's interactive elements."""

    def __init__(
        self,
        zeroglab_settings: ZeroGLabConf = None,
        **kwargs,
    ) -> None:
        """
        Initializes the lab controller. This class is used to control the lab interactive elements.
        Including:
            - Projector position, intensity, radius, color.
            - Room lights intensity, radius, color.
            - Curtains open or closed.
            - Terrains randomization or using premade DEMs.
            - Rocks random placements.

        Args:
            lunalab_settings (LunalabLabConf): The settings of the lab.
            rocks_settings (Dict): The settings of the rocks.
            terrain_manager (TerrainManagerConf): The settings of the terrain manager.
            **kwargs: Arbitrary keyword arguments."""

        super().__init__(**kwargs)
        self.stage_settings = zeroglab_settings
        self.dem = None
        self.mask = None
        self.scene_name = "/ZeroGLab"

    def build_scene(self) -> None:
        """
        Builds the scene. It either loads the scene from a file or creates it from scratch.
        """
        stage = omni.usd.get_context().get_stage()
        
        scene_path = get_assets_path() + "/USD_Assets/environments/zero_g_lab.usd"
        # Loads the Lunalab
        stage_prim = add_reference_to_stage(scene_path, self.scene_name)
        UsdPhysics.CollisionAPI.Apply(stage_prim)

        #Move to correct location
        zero_g_lab_prim = stage.GetPrimAtPath(self.scene_name)
        zero_g_lab_xform = UsdGeom.Xformable(zero_g_lab_prim)
        
        zero_g_lab_xform.AddTranslateOp().Set(Gf.Vec3d(2.5, -1.5, 0))
        zero_g_lab_xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, 90))
        
        # Creates lights
        distant_light = UsdLux.DistantLight.Define(stage, "/Lights/sun")
        distant_light.GetIntensityAttr().Set(7000)



    def instantiate_scene(self) -> None:
        """
        Instantiates the scene. Applies any operations that need to be done after the scene is built and
        the renderer has been stepped.
        """

        pass

    def reset(self) -> None:
        """
        Resets the environment. Implement the logic to reset the environment.
        """

        pass

    def update(self) -> None:
        """
        Updates the environment.
        """

        pass

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain.
        """

        self.build_scene()
        # Fetches the interactive elements
        self.collect_interactive_assets()


    def add_robot_manager(self, robotManager: RobotManager) -> None:
        """
        Adds the robot manager to the environment.

        Args:
            robotManager (RobotManager): The robot manager to be added.
        """

        self.robotManager = robotManager

    def get_lux_assets(self, prim: "Usd.Prim") -> List[Usd.Prim]:
        """
        Returns the UsdLux prims under a given prim.

        Args:
            prim (Usd.Prim): The prim to be searched.

        Returns:
            list: A list of UsdLux prims.
        """

        lights = []
        for prim in Usd.PrimRange(prim):
            if prim.IsA(UsdLux.SphereLight):
                lights.append(prim)
            if prim.IsA(UsdLux.CylinderLight):
                lights.append(prim)
            if prim.IsA(UsdLux.DiskLight):
                lights.append(prim)
        return lights


    def collect_interactive_assets(self) -> None:
        """
        Collects the interactive assets from the stage and assigns them to class variables.
        """

        # Room Lights
        self._room_lights_prim = self.stage.GetPrimAtPath(self.stage_settings['room_lights_path'])
        self._room_lights_xform = UsdGeom.Xformable(self._room_lights_prim)
        self._room_lights_lux = self.get_lux_assets(self._room_lights_prim)
