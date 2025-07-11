from threading import Thread

from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from pxr import UsdGeom, Gf
from typing import Union
import logging
import omni
import time


from zeroGLab_src.physics.physics_scene import PhysicsSceneManager
from zeroGLab_src.environments_wrappers.ros2.zero_g_lab_ros2 import ROS_ZeroGLabManager
from zeroGLab_src.environments_wrappers.ros2.robot_manager_ros2 import ROS_RobotManager
from rclpy.executors import SingleThreadedExecutor as Executor
import rclpy


logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")

class Rate:
    """
    Creates a rate object that enables to sleep for a minimum amount
    of time between two iterations of a loop. If freq and dt are
    passed, the object will only use the information provided by dt.
    """

    def __init__(self, freq: float = None, dt: float = None, is_disabled: bool = False) -> None:
        """
        Args:
          freq (float): The frequency at which the loop should be executed.
          dt (float): The delta of time to be kept between two loop iterations.
        """

        self.is_disabled = is_disabled

        if not self.is_disabled:
            if dt is None:
                if freq is None:
                    raise ValueError("You must provide either a frequency or a delta time.")
                else:
                    self.dt = 1.0 / freq
            else:
                self.dt = dt

            self.last_check = time.time()

    def reset(self) -> None:
        """
        Resets the timer.
        """
        if not self.is_disabled:
            self.last_check = time.time()

    def sleep(self) -> None:
        """
        Wait for a minimum amount of time between two iterations of a loop.
        """

        if not self.is_disabled:
            now = time.time()
            delta = now - self.last_check
            # If time delta is too low sleep, else carry on.
            if delta < self.dt:
                to_sleep = self.dt - delta
                time.sleep(to_sleep)

class ROS2_LabManagerFactory:
    def __init__(self):
        self._lab_managers = {}

    def register(
        self,
        name: str,
        lab_manager: Union[ROS_ZeroGLabManager],
    ) -> None:
        """
        Registers a lab manager.

        Args:
            name (str): Name of the lab manager.
            lab_manager (Union[ROS_ZeroGLabManager]): Instance of the lab manager.
        """

        self._lab_managers[name] = lab_manager

    def __call__(
        self,
        cfg: dict,
        **kwargs,
    ) -> Union[ROS_ZeroGLabManager]:
        """
        Returns an instance of the lab manager corresponding to the environment name.

        Args:
            cfg (dict): Configuration dictionary.

        Returns:
            Union[ROS_ZeroGLabManager]: Instance of the lab manager.
        """

        return self._lab_managers[cfg["environment"]["name"]](
            environment_cfg=cfg["environment"],
            **kwargs,
        )


ROS2_LMF = ROS2_LabManagerFactory()
ROS2_LMF.register("ZeroGLab", ROS_ZeroGLabManager)


class ROS2_SimulationManager:
    """
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the lab manager thread
    - Running the robot manager thread
    - Running the simulation
    - Cleaning the simulation
    """

    def __init__(
        self,
        cfg: dict,
        simulation_app: SimulationApp,
    ) -> None:
        """
        Initializes the simulation.

        Args:
            cfg (dict): Configuration dictionary.
            simulation_app (SimulationApp): SimulationApp instance.
        """

        self.cfg = cfg
        self.simulation_app = simulation_app
        # Setups the physics and acquires the different interfaces to talk with Isaac
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=cfg["environment"]["physics_dt"],
            rendering_dt=cfg["environment"]["rendering_dt"],
        )
        PSM = PhysicsSceneManager(cfg["physics"]["physics_scene"])
        for i in range(100):
            self.world.step(render=True)
        self.world.reset()

        if cfg["environment"]["enforce_realtime"]:
            self.rate = Rate(dt=cfg["environment"]["physics_dt"])
        else:
            self.rate = Rate(is_disabled=True)

        # Lab manager thread
        self.ROSLabManager = ROS2_LMF(
            cfg, is_simulation_alive=self.simulation_app.is_running, close_simulation=self.simulation_app.close
        )
        self.exec1 = Executor()
        self.exec1.add_node(self.ROSLabManager)
        self.exec1_thread = Thread(target=self.exec1.spin, daemon=True, args=())
        self.exec1_thread.start()
        # Robot manager thread
        self.ROSRobotManager = ROS_RobotManager(cfg["robot"]["robots_settings"])
        self.exec2 = Executor()
        self.exec2.add_node(self.ROSRobotManager)
        self.exec2_thread = Thread(target=self.exec2.spin, daemon=True, args=())
        self.exec2_thread.start()

        if self.ROSLabManager.get_wait_for_threads():
            self.simulation_app.add_wait(self.ROSLabManager.get_wait_for_threads())

        # Have you ever asked your self: "Is there a limit of topics one can subscribe to in ROS2?"
        # Yes "Josh" there is.
        # 24 topics. More than that and you won't reveive any messages.
        # Keep it in mind if you want to go crazy with the ROS2 calls to modify the sim...

        # Preload the assets
        if cfg["robot"]["robots_settings"]["name"] == "FloatingPlatform":
            stage = omni.usd.get_context().get_stage()
            robots_prim = stage.GetPrimAtPath("/Robots")
            robots_xform = UsdGeom.Xformable(robots_prim)
            robots_xform.ClearXformOpOrder()
            robots_xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, 180))

            self.ROSRobotManager.RM.preload_robot(self.world)
            self.ROSRobotManager.RM.custom_funct(
                robot_name="/FloatingPlatform",
                function_name="define_articulations"
            )
            self.ROSRobotManager.RM.custom_funct(
                robot_name="/FloatingPlatform", 
                function_name="set_dof_pos"
            )

        self.ROSLabManager.FPLC.add_robot_manager(self.ROSRobotManager.RM)

        # for i in range(100):
        #     self.world.step(render=True)
        # self.world.reset()
        # self.ROSRobotManager.reset()

    def run_simulation(self) -> None:
        """
        Runs the simulation.
        """

        self.timeline.play()
        self.world.reset() 
        self.ROSLabManager.reset()
        self.ROSRobotManager.reset()

        prev_state = None
        while self.simulation_app.is_running():
            self.rate.reset()
            self.world.step(render=True)
            current_state = self.world.is_playing()

            if current_state and not prev_state:
                # If transitioning from stopped to playing, reset
                self.ROSRobotManager.reset()
                self.ROSLabManager.reset()

            prev_state = current_state

            if self.world.is_playing():
                # Apply modifications to the lab only once the simulation step is finished
                # This is extremely important as modifying the stage during a simulation step
                # will lead to a crash.
                self.ROSLabManager.periodic_update(dt=self.world.get_physics_dt())
                if self.world.current_time_step_index == 0:
                    self.world.reset()
                    self.ROSLabManager.reset()
                    self.ROSRobotManager.reset()
                self.ROSLabManager.apply_modifications()
                if self.ROSLabManager.trigger_reset:
                    self.ROSRobotManager.reset()
                    self.ROSLabManager.trigger_reset = False
                self.ROSRobotManager.apply_modifications()

            if not self.ROSLabManager.monitor_thread_is_alive():
                logger.debug("Destroying the ROS nodes")
                self.ROSLabManager.destroy_node()
                self.ROSRobotManager.destroy_node()
                logger.debug("Shutting down the ROS executors")
                self.exec1.shutdown()
                self.exec2.shutdown()
                logger.debug("Joining the ROS threads")
                self.exec1_thread.join()
                self.exec2_thread.join()
                logger.debug("Shutting down ROS2")
                rclpy.shutdown()
                break

            self.rate.sleep()
        self.world.stop()
        self.timeline.stop()