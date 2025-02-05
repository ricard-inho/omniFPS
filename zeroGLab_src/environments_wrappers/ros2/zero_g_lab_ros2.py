


from zeroGLab_src.environments_wrappers.ros2.base_wrapper_ros2 import ROS_BaseManager
from zeroGLab_src.environments.zeroglab import ZeroGLabController

from std_msgs.msg import Bool, Float32, ColorRGBA, Int32
from geometry_msgs.msg import Pose
import rclpy

class ROS_ZeroGLabManager(ROS_BaseManager):
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
            **kwargs: Additional arguments.
        """

        super().__init__(environment_cfg=environment_cfg, **kwargs)
        self.FPLC = ZeroGLabController(**environment_cfg)
        self.FPLC.load()
        self.trigger_reset = False

    def periodic_update(self, dt: float) -> None:
        pass

    def reset(self) -> None:
        """
        Resets the lab to its initial state."""

        pass