

def enable_ros2(simulation_app, bridge_name="foxy", **kwargs) -> None:
    """
    Enables ROS2 in the simulation.

    Args:
        simulation_app (SimulationApp): SimulationApp instance.
        **kwargs: Additional keyword arguments."""

    from omni.isaac.core.utils.extensions import enable_extension

    if bridge_name == "foxy":
        enable_extension("omni.isaac.ros2_bridge")
    elif bridge_name == "humble":
        enable_extension("omni.isaac.ros2_bridge") # replaced omni.isaac.ros2_bridge_humble
    else:
        raise ValueError("Bridge not supported, please choose between foxy and humble.")
    enable_extension("omni.kit.viewport.actions")

    simulation_app.update()