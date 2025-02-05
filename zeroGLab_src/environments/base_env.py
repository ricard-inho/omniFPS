
import omni

from pxr import Usd

from zeroGLab_src.robots.robot_manager import RobotManager


class BaseEnv:
    """
    This class is used to control the environment's interactive elements.
    """

    def __init__(
        self,
        **kwargs,
    ) -> None:
        """
        Initializes the lab controller. This class is used to control the lab interactive elements.

        Args:
            **kwargs: Arbitrary keyword arguments.
        """

        self.stage: Usd.Stage = omni.usd.get_context().get_stage()

    def build_scene(self) -> None:
        """
        Builds the scene. It either loads the scene from a file or creates it from scratch.
        """

        raise NotImplementedError()

    def instantiate_scene(self) -> None:
        """
        Instantiates the scene. Applies any operations that need to be done after the scene is built and
        the renderer has been stepped.
        """

        raise NotImplementedError()

    def reset(self) -> None:
        """
        Resets the environment. Implement the logic to reset the environment.
        """

        raise NotImplementedError()

    def update(self) -> None:
        """
        Updates the environment.
        """

        raise NotImplementedError()

    def load(self) -> None:
        """
        Loads the lab interactive elements in the stage.
        Creates the instancer for the rocks, and generates the terrain.
        """

        raise NotImplementedError()

    def add_robot_manager(self, robotManager: RobotManager) -> None:
        """
        Adds the robot manager to the environment.

        Args:
            robotManager (RobotManager): The robot manager.
        """
        self.robotManager = robotManager

    def deform_terrain(self) -> None:
        """
        Deforms the terrain.
        Args:
            world_poses (np.ndarray): The world poses of the contact points.
            contact_forces (np.ndarray): The contact forces in local frame reported by rigidprimview.
        """

        raise NotImplementedError()

    def apply_terramechanics(self) -> None:
        """
        Applies the terramechanics.
        """

        raise NotImplementedError()