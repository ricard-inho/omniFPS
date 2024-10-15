__author__ = "Antoine Richard, Junnosuke Kamohara, Ricard Marsal"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Ricard Marsal"
__email__ = "ricard.marsal@uni.lu"
__status__ = "development"

from typing import List, Tuple, Dict
import numpy as np

from omni.isaac.core.utils.stage import open_stage, add_reference_to_stage
import omni

from pxr import UsdGeom, UsdLux, Gf, Usd

from src.configurations.environments import ZeroGLabConf
from src.environments.base_env import BaseEnv
from src.robots.robot import RobotManager
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

        scene_path = get_assets_path() + "/USD_Assets/environments/lab_test.usd"
        # Loads the Lunalab
        add_reference_to_stage(scene_path, self.scene_name)

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

    def load_DEM(self) -> None:
        """
        Loads the DEM and the mask from the TerrainManager.
        """

        self.dem = self.T.getDEM()
        self.mask = self.T.getMask()

    def collect_interactive_assets(self) -> None:
        """
        Collects the interactive assets from the stage and assigns them to class variables.
        """

        # Room Lights
        self._room_lights_prim = self.stage.GetPrimAtPath(self.stage_settings['room_lights_path'])
        self._room_lights_xform = UsdGeom.Xformable(self._room_lights_prim)
        self._room_lights_lux = self.get_lux_assets(self._room_lights_prim)


    # ==============================================================================
    # Room lights control
    # ==============================================================================
    def set_room_lights_intensity(self, intensity: float = 0.0) -> None:
        """
        Sets the intensity of the room lights.

        Args:
            intensity (float): The intensity of the room lights (arbitrary unit).
        """

        for light in self._room_lights_lux:
            light.GetAttribute("intensity").Set(intensity)

    def set_room_lights_radius(self, radius: float = 0.1) -> None:
        """
        Sets the radius of the room lights.

        Args:
            radius (float): The radius of the room lights (in meters).
        """

        for light in self._room_lights_lux:
            light.GetAttribute("radius").Set(radius)

    def set_room_lights_FOV(self, FOV: float = 42.0) -> None:
        """
        Sets the FOV of the room lights.

        Args:
            FOV (float): The FOV of the room lights (in degrees).
        """

        for light in self._room_lights_lux:
            light.GetAttribute("shaping:cone:angle").Set(FOV)

    def set_room_lights_color(self, color: Tuple[float, float, float] = (1.0, 1.0, 1.0)) -> None:
        """
        Sets the color of the room lights.

        Args:
            color (List[float]): The color of the room lights (RGB).
        """

        color = Gf.Vec3d(color[0], color[1], color[2])
        for light in self._room_lights_lux:
            light.GetAttribute("color").Set(color)

    def turn_room_lights_on_off(self, flag: bool = True) -> None:
        """
        Turns the room lights on or off.

        Args:
            flag (bool): True to turn the room lights on, False to turn them off.
        """

        if flag:
            self._room_lights_prim.GetAttribute("visibility").Set("visible")
        else:
            self._room_lights_prim.GetAttribute("visibility").Set("invisible")

    