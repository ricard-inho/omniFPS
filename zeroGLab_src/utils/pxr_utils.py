
import omni
from typing import List, Tuple, Union
from pxr import Gf, UsdPhysics, UsdGeom, UsdShade, Sdf, Usd, PhysxSchema
import numpy as np


def addDefaultOps(prim: Usd.Prim,
                  ) -> UsdGeom.Xform:
    """
    Adds the default ops to a prim.
    
    Args:
        prim (Usd.Prim): The prim.
    
    Returns:
        UsdGeom.Xform: The Xform of the prim."""
    xform = UsdGeom.Xform(prim)
    xform.ClearXformOpOrder()

    try:
        xform.AddTranslateOp(precision = UsdGeom.XformOp.PrecisionDouble)
    except:
        pass
    try:
        xform.AddOrientOp(precision = UsdGeom.XformOp.PrecisionDouble)
    except:
        pass
    try:
        xform.AddScaleOp(precision = UsdGeom.XformOp.PrecisionDouble)
    except:
        pass
    return xform


def setDefaultOpsTyped(xform: UsdGeom.Xform,
                       pos: Union[Gf.Vec3d, Gf.Vec3f],
                       rot: Union[Gf.Quatd, Gf.Quatf],
                       scale: Union[Gf.Vec3d, Gf.Vec3f],
                       ) -> None:
    """
    Sets the default ops of a Xform prim using the native values (Gf.Vec3d, Gf.Vec3f, Gf.Quatd, Gf.Quatf).
    
    Args:
        xform (UsdGeom.Xform): The Xform prim.
        pos (Union[Gf.Vec3d, Gf.Vec3f]): The position.
        rot (Union[Gf.Quatd, Gf.Quatf]): The rotation as a quaternion.
        scale (Union[Gf.Vec3d, Gf.Vec3f]): The scale."""

    xform_ops = xform.GetOrderedXformOps()
    xform_ops[0].Set(pos)
    xform_ops[1].Set(rot)
    xform_ops[2].Set(scale)

# ==================================================================================================
# Utils for Xform manipulation
# ==================================================================================================


def setXformOp(prim: Usd.Prim, value, property: UsdGeom.XformOp.Type) -> None:
    """
    Sets a transform operatios on a prim.

    Args:
        prim (Usd.Prim): The prim to set the transform operation.
        value: The value of the transform operation.
        property (UsdGeom.XformOp.Type): The type of the transform operation.
    """

    xform = UsdGeom.Xformable(prim)
    op = None
    for xformOp in xform.GetOrderedXformOps():
        if xformOp.GetOpType() == property:
            op = xformOp
    if op:
        xform_op = op
    else:
        xform_op = xform.AddXformOp(property, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(value)


def setScale(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the scale of a prim.

    Args:
        prim (Usd.Prim): The prim to set the scale.
        value (Gf.Vec3d): The value of the scale.
    """

    setXformOp(prim, value, UsdGeom.XformOp.TypeScale)


def setTranslate(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the translation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the translation.
        value (Gf.Vec3d): The value of the translation.
    """

    setXformOp(prim, value, UsdGeom.XformOp.TypeTranslate)


def setRotateXYZ(prim: Usd.Prim, value: Gf.Vec3d) -> None:
    """
    Sets the rotation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the rotation.
        value (Gf.Vec3d): The value of the rotation.
    """

    setXformOp(prim, value, UsdGeom.XformOp.TypeRotateXYZ)


def setOrient(prim: Usd.Prim, value: Gf.Quatd) -> None:
    """
    Sets the rotation of a prim.

    Args:
        prim (Usd.Prim): The prim to set the rotation.
        value (Gf.Quatd): The value of the rotation.
    """

    setXformOp(prim, value, UsdGeom.XformOp.TypeOrient)


def setTransform(prim, value: Gf.Matrix4d) -> None:
    """
    Sets the transform of a prim.

    Args:
        prim (Usd.Prim): The prim to set the transform.
        value (Gf.Matrix4d): The value of the transform.
    """

    setXformOp(prim, value, UsdGeom.XformOp.TypeTransform)


def setXformOps(
    prim,
    translate: Gf.Vec3d = Gf.Vec3d([0, 0, 0]),
    orient: Gf.Quatd = Gf.Quatd(1, Gf.Vec3d([0, 0, 0])),
    scale: Gf.Vec3d = Gf.Vec3d([1, 1, 1]),
) -> None:
    """
    Sets the transform of a prim.

    Args:
        prim (Usd.Prim): The prim to set the transform.
        translate (Gf.Vec3d): The value of the translation.
        orient (Gf.Quatd): The value of the rotation.
        scale (Gf.Vec3d): The value of the scale.
    """

    setTranslate(prim, translate)
    setOrient(prim, orient)
    setScale(prim, scale)


def getTransform(prim: Usd.Prim, parent: Usd.Prim) -> Gf.Matrix4d:
    """
    Gets the transform of a prim relative to its parent.

    Args:
        prim (Usd.Prim): The prim to get the transform.
        parent (Usd.Prim): The parent of the prim.
    """

    return UsdGeom.XformCache(0).ComputeRelativeTransform(prim, parent)[0]


# ==================================================================================================
# Utils for API manipulation
# ==================================================================================================


def applyMaterial(
    prim: Usd.Prim,
    material: UsdShade.Material,
    purpose: str = None,
    weaker_than_descendants=False,
) -> UsdShade.MaterialBindingAPI:
    """
    Applies a material to a prim.

    Args:
        prim (Usd.Prim): The prim to apply the material.
        material (UsdShade.Material): The material to apply.
        purpose (None): The purpose of the material.
        weaker_than_descendants (bool): The material is weaker than its descendants.

    Returns:
        UsdShade.MaterialBindingAPI: The MaterialBindingAPI.
    """

    binder = UsdShade.MaterialBindingAPI.Apply(prim)
    if purpose is None:
        if weaker_than_descendants:
            binder.Bind(
                material,
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            )
        else:
            binder.Bind(
                material,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            )
    else:
        assert purpose in [
            "allPurpose",
            "all",
            "preview",
            "physics",
        ], "Purpose must be 'allPurpose', 'all', 'preview' or 'physics'."
        if weaker_than_descendants:
            binder.Bind(
                material,
                materialPurpose=purpose,
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            )
        else:
            binder.Bind(
                material,
                materialPurpose=purpose,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            )

    return binder


def applyRigidBody(prim: Usd.Prim) -> UsdPhysics.RigidBodyAPI:
    """
    Applies a RigidBodyAPI to a prim.

    Args:
        prim (Usd.Prim): The prim to apply the RigidBodyAPI.

    Returns:
        UsdPhysics.RigidBodyAPI: The RigidBodyAPI.
    """

    rigid = UsdPhysics.RigidBodyAPI.Apply(prim)
    return rigid


def applyCollider(prim: Usd.Prim, enable: bool = False) -> UsdPhysics.CollisionAPI:
    """
    Applies a ColliderAPI to a prim.

    Args:
        prim (Usd.Prim): The prim to apply the ColliderAPI.
        enable (bool): Enable or disable the collider.

    Returns:
        UsdPhysics.CollisionAPI: The ColliderAPI.
    """

    collider = UsdPhysics.CollisionAPI.Apply(prim)
    collider.CreateCollisionEnabledAttr(enable)
    return collider


def applyMass(
    prim: Usd.Prim, mass: float, CoM: Gf.Vec3d = Gf.Vec3d([0, 0, 0])
) -> UsdPhysics.MassAPI:
    """
    Applies a MassAPI to a prim.
    Sets the mass and the center of mass of the prim.

    Args:
        prim (Usd.Prim): The prim to apply the MassAPI.
        mass (float): The mass of the prim.
        CoM (Gf.Vec3d): The center of mass of the prim.

    Returns:
        UsdPhysics.MassAPI: The MassAPI.
    """

    massAPI = UsdPhysics.MassAPI.Apply(prim)
    massAPI.CreateMassAttr().Set(mass)
    massAPI.CreateCenterOfMassAttr().Set(CoM)
    return massAPI


def createDrive(
    joint: Usd.Prim,
    token: str = "transX",
    damping: float = 1e3,
    stiffness: float = 1e6,
    max_force: float = None,
) -> UsdPhysics.DriveAPI:
    """
    Creates a DriveAPI on a joint.

    List of allowed tokens:
     "transX", "transY", "transZ", "linear"
     "rotX", "rotY", "rotZ", "angular"

    Args:
        joint (Usd.Prim): The joint to apply the DriveAPI.
        token (str, optional): The type of the drive.
        damping (float, optional): The damping of the drive.
        stiffness (float, optional): The stiffness of the drive.
        max_force (float, optional): The maximum force of the drive.

    Returns:
        UsdPhysics.DriveAPI: The DriveAPI.
    """

    driveAPI = UsdPhysics.DriveAPI.Apply(joint, token)
    driveAPI.CreateTypeAttr("force")
    driveAPI.CreateDampingAttr(damping)
    driveAPI.CreateStiffnessAttr(stiffness)
    if max_force is not None:
        driveAPI.CreateMaxForceAttr(max_force)
    return driveAPI


def createLimit(
    joint: Usd.Prim,
    token: str = "transX",
    low: float = None,
    high: float = None,
) -> UsdPhysics.LimitAPI:
    """
    Creates a LimitAPI on a joint.

    List of allowed tokens:
     "transX", "transY", "transZ", "linear"
     "rotX", "rotY", "rotZ", "angular"

    Args:
        joint (Usd.Prim): The joint to apply the LimitAPI.
        token (str, optional): The type of the limit.
        low (float, optional): The lower limit of the joint.
        high (float, optional): The upper limit of the joint.

    Returns:
        UsdPhysics.LimitAPI: The LimitAPI.
    """

    limitAPI = UsdPhysics.LimitAPI.Apply(joint, token)
    if low:
        limitAPI.CreateLowAttr(low)
    if high:
        limitAPI.CreateHighAttr(high)
    return limitAPI


# ==================================================================================================
# Utils for Geom manipulation
# ==================================================================================================


def createObject(prefix: str,
    stage: Usd.Stage,
    path: str,
    position: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
    rotation: Gf.Quatd = Gf.Quatd(0,0,0,1),
    scale: Gf.Vec3d = Gf.Vec3d(1,1,1),
    is_instance: bool = True,
    ) -> Tuple[Usd.Prim, str]:
    """
    Creates a 3D object from a USD file and adds it to the stage.
    
    Args:
        prefix (str): The prefix of the object.
        stage (Usd.Stage): The stage.
        path (str): The path to the USD file.
        position (Gf.Vec3d, optional): The position of the object. Defaults to Gf.Vec3d(0, 0, 0).
        rotation (Gf.Quatd, optional): The rotation of the object. Defaults to Gf.Quatd(0,0,0,1).
        scale (Gf.Vec3d, optional): The scale of the object. Defaults to Gf.Vec3d(1,1,1).
        is_instance (bool, optional): Whether the object is an instance or not. Defaults to True."""
    
    obj_prim, prim_path = createXform(stage, prefix)
    xform = UsdGeom.Xformable(prim_path)
    addDefaultOps(xform)
    setDefaultOpsTyped(xform, position, rotation, scale)
    prim_path.GetReferences().AddReference(path)
    if is_instance:
        obj_prim.SetInstanceable(True)
    return obj_prim, prim_path

def createXform(
    stage: Usd.Stage,
    path: str,
) -> Tuple[str, Usd.Prim]:
    """
    Creates an Xform prim.
    And sets the default transform operations.

    Args:
        stage (Usd.Stage): The stage to create the Xform prim.
        path (str): The path of the Xform prim.

    Returns:
        Tuple[str, Usd.Prim]: The path and the prim of the Xform prim.
    """

    path = omni.usd.get_stage_next_free_path(stage, path, False)
    prim = stage.DefinePrim(path, "Xform")
    setXformOps(prim)
    return path, prim


def refineShape(stage: Usd.Stage, path: str, refinement: int) -> None:
    """
    Refines the geometry of a shape.
    This operation is purely visual, it does not affect the physics simulation.

    Args:
        stage (Usd.Stage): The stage to refine the shape.
        path (str): The path of the shape.
        refinement (int): The number of times to refine the shape.
    """

    prim = stage.GetPrimAtPath(path)
    prim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int)
    prim.GetAttribute("refinementLevel").Set(refinement)
    prim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool)
    prim.GetAttribute("refinementEnableOverride").Set(True)


def createSphere(
    stage: Usd.Stage,
    path: str,
    radius: float,
    refinement: int,
) -> Tuple[str, UsdGeom.Sphere]:
    """
    Creates a sphere.

    Args:
        stage (Usd.Stage): The stage to create the sphere.
        path (str): The path of the sphere.
        radius (float): The radius of the sphere.
        refinement (int): The number of times to refine the sphere.

    Returns:
        Tuple[str, UsdGeom.Sphere]: The path and the prim of the sphere.
    """

    path = omni.usd.get_stage_next_free_path(stage, path, False)
    sphere_geom = UsdGeom.Sphere.Define(stage, path)
    sphere_geom.GetRadiusAttr().Set(radius)
    setXformOps(sphere_geom)
    refineShape(stage, path, refinement)
    return path, sphere_geom


def createCylinder(
    stage: Usd.Stage,
    path: str,
    radius: float,
    height: float,
    refinement: int,
) -> Tuple[str, UsdGeom.Cylinder]:
    """
    Creates a cylinder.

    Args:
        stage (Usd.Stage): The stage to create the cylinder.
        path (str): The path of the cylinder.
        radius (float): The radius of the cylinder.
        height (float): The height of the cylinder.
        refinement (int): The number of times to refine the cylinder.

    Returns:
        Tuple[str, UsdGeom.Cylinder]: The path and the prim of the cylinder.
    """

    path = omni.usd.get_stage_next_free_path(stage, path, False)
    cylinder_geom = UsdGeom.Cylinder.Define(stage, path)
    cylinder_geom.GetRadiusAttr().Set(radius)
    cylinder_geom.GetHeightAttr().Set(height)
    setXformOps(cylinder_geom)
    refineShape(stage, path, refinement)
    return path, cylinder_geom


def createCapsule(
    stage: Usd.Stage,
    path: str,
    radius: float,
    height: float,
    refinement: int,
) -> Tuple[str, UsdGeom.Capsule]:
    """
    Creates a capsule.

    Args:
        stage (Usd.Stage): The stage to create the capsule.
        path (str): The path of the capsule.
        radius (float): The radius of the capsule.
        height (float): The height of the capsule.
        refinement (int): The number of times to refine the capsule.

    Returns:
        Tuple[str, UsdGeom.Capsule]: The path and the prim of the capsule.
    """

    path = omni.usd.get_stage_next_free_path(stage, path, False)
    capsule_geom = UsdGeom.Capsule.Define(stage, path)
    capsule_geom.GetRadiusAttr().Set(radius)
    capsule_geom.GetHeightAttr().Set(height)
    setXformOps(capsule_geom)
    refineShape(stage, path, refinement)
    return path, capsule_geom


def createCube(
    stage: Usd.Stage,
    path: str,
    depth: float,
    width: float,
    height: float,
    refinement: int,
) -> Tuple[str, UsdGeom.Cube]:
    """
    Creates a cube.

    Args:
        stage (Usd.Stage): The stage to create the cube.
        path (str): The path of the cube.
        depth (float): The depth of the cube.
        width (float): The width of the cube.
        height (float): The height of the cube.
        refinement (int): The number of times to refine the cube.

    Returns:
        Tuple[str, UsdGeom.Cube]: The path and the prim of the cube.
    """

    path = omni.usd.get_stage_next_free_path(stage, path, False)
    cube_geom = UsdGeom.Cube.Define(stage, path)
    cube_geom.GetSizeAttr().Set(1)
    setXformOps(cube_geom, scale=Gf.Vec3d([depth, width, height]))
    refineShape(stage, path, refinement)
    return path, cube_geom


def createCone(
    stage: Usd.Stage,
    path: str,
    radius: float,
    height: float,
    refinement: int,
) -> Tuple[str, UsdGeom.Cone]:
    """
    Creates a cone.

    Args:
        stage (Usd.Stage): The stage to create the cone.
        path (str): The path of the cone.
        radius (float): The radius of the cone.
        height (float): The height of the cone.
        refinement (int): The number of times to refine the cone.

    Returns:
        Tuple[str, UsdGeom.Cone]: The path and the prim of the cone.
    """

    path = omni.usd.get_stage_next_free_path(stage, path, False)
    cone_geom = UsdGeom.Cone.Define(stage, path)
    cone_geom.GetRadiusAttr().Set(radius)
    cone_geom.GetHeightAttr().Set(height)
    setXformOps(cone_geom)
    refineShape(stage, path, refinement)
    return path, cone_geom


def createArrow(
    stage: Usd.Stage,
    path: int,
    radius: float,
    length: float,
    offset: list,
    refinement: int,
) -> None:
    """
    Creates an arrow.

    Args:
        stage (Usd.Stage): The stage to create the arrow.
        path (str): The path of the arrow.
        radius (float): The radius of the arrow.
        length (float): The length of the arrow.
        offset (list): The offset of the arrow.
        refinement (int): The number of times to refine the arrow.

    Returns:
        Tuple[str, UsdGeom.Cone]: The path and the prim of the arrow.
    """

    length = length / 2
    body_path, body_geom = createCylinder(
        stage, path + "/arrow_body", radius, length, refinement
    )
    setTranslate(body_geom, Gf.Vec3d([offset[0] + length * 0.5, 0, offset[2]]))
    setOrient(body_geom, Gf.Quatd(0.707, Gf.Vec3d(0, 0.707, 0)))
    head_path, head_geom = createCone(
        stage, path + "/arrow_head", radius * 1.5, length, refinement
    )
    setTranslate(head_geom, Gf.Vec3d([offset[0] + length * 1.5, 0, offset[2]]))
    setOrient(head_geom, Gf.Quatd(0.707, Gf.Vec3d(0, 0.707, 0)))


def createThrusterShape(
    stage: Usd.Stage,
    path: str,
    radius: float,
    height: float,
    refinement: int,
) -> None:
    """
    Creates a thruster.

    Args:
        stage (Usd.Stage): The stage to create the thruster.
        path (str): The path of the thruster.
        radius (float): The radius of the thruster.
        height (float): The height of the thruster.
        refinement (int): The number of times to refine the thruster.

    Returns:
        Tuple[str, UsdGeom.Cone]: The path and the prim of the thruster.
    """

    height /= 2
    # Creates a cylinder
    cylinder_path, cylinder_geom = createCylinder(
        stage, path + "/cylinder", radius, height, refinement
    )
    cylinder_prim = stage.GetPrimAtPath(cylinder_geom.GetPath())
    applyCollider(cylinder_prim)
    setTranslate(cylinder_geom, Gf.Vec3d([0, 0, height * 0.5]))
    setScale(cylinder_geom, Gf.Vec3d([1, 1, 1]))
    # Create a cone
    cone_path, cone_geom = createCone(stage, path + "/cone", radius, height, refinement)
    cone_prim = stage.GetPrimAtPath(cone_geom.GetPath())
    applyCollider(cone_prim)
    setTranslate(cone_geom, Gf.Vec3d([0, 0, height * 1.5]))
    setRotateXYZ(cone_geom, Gf.Vec3d([0, 180, 0]))


def createColor(
    stage: Usd.Stage,
    material_path: str,
    color: list,
) -> UsdShade.Material:
    """
    Creates a color material.

    Args:
        stage (Usd.Stage): The stage to create the color material.
        material_path (str): The path of the material.
        color (list): The color of the material

    Returns:
        UsdShade.Material: The material.
    """

    material_path = omni.usd.get_stage_next_free_path(stage, material_path, False)
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(color))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def createPhysicsMaterial(
    stage: Usd.Stage,
    material_path: str,
    static_friction: float,
    dynamic_friction: float,
    restitution: float,
    friction_combine_mode: str = "average",
    restitution_combine_mode: str = "average",
) -> UsdPhysics.MaterialAPI:
    """
    Creates a physics material.

    Args:
        stage (Usd.Stage): The stage to create the physics material.
        material_path (str): The path of the material.
        static_friction (float): The static friction of the material.
        dynamic_friction (float): The dynamic friction of the material.
        restitution (float): The restitution of the material.
        friction_combine_mode (str, optional): The way the friction between two surfaces is combined.
        restitution_combine_mode (str, optional): The way the friction between two surfaces is combined.

    Returns:
        UsdPhysics.MaterialAPI: The physics material.
    """

    if not friction_combine_mode in ["multiply", "average", "min", "max"]:
        raise ValueError("average_friction_mode must be average, multiply, min or max")
    if not restitution_combine_mode in ["multiply", "average", "min", "max"]:
        raise ValueError(
            "average_restitution_mode must be average, multiply, min or max"
        )

    material_path = omni.usd.get_stage_next_free_path(stage, material_path, False)
    visual_material = UsdShade.Material.Define(stage, material_path)
    prim = stage.GetPrimAtPath(material_path)
    material = UsdPhysics.MaterialAPI.Apply(prim)
    material.CreateStaticFrictionAttr().Set(static_friction)
    material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    material.CreateRestitutionAttr().Set(restitution)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(prim)
    physx_material.CreateFrictionCombineModeAttr().Set(friction_combine_mode)
    physx_material.CreateRestitutionCombineModeAttr().Set(restitution_combine_mode)
    return material


def createArticulation(
    stage: Usd.Stage,
    path: str,
) -> Tuple[str, Usd.Prim]:
    """
    Creates an ArticulationRootAPI on a prim.

    Args:
        stage (Usd.Stage): The stage to create the ArticulationRootAPI.
        path (str): The path of the ArticulationRootAPI.

    Returns:
        Tuple[str, Usd.Prim]: The path and the prim of the ArticulationRootAPI.
    """

    # Creates the Xform of the platform
    path, prim = createXform(stage, path)
    setXformOps(prim)
    # Creates the Articulation root
    root = UsdPhysics.ArticulationRootAPI.Apply(prim)
    return path, prim


def createFixedJoint(
    stage: Usd.Stage,
    path: str,
    body_path1: str = None,
    body_path2: str = None,
) -> UsdPhysics.FixedJoint:
    """
    Creates a fixed joint between two bodies.

    Args:
        stage (Usd.Stage): The stage to create the fixed joint.
        path (str): The path of the fixed joint.
        body_path1 (str, optional): The path of the first body.
        body_path2 (str, optional): The path of the second body.

    Returns:
        UsdPhysics.FixedJoint: The fixed joint.
    """

    # Create fixed joint
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    # Set body targets
    if body_path1 is not None:
        joint.CreateBody0Rel().SetTargets([body_path1])
    if body_path2 is not None:
        joint.CreateBody1Rel().SetTargets([body_path2])

    if (body_path1 is not None) and (body_path2 is not None):
        # Get from the simulation the position/orientation of the bodies
        body_1_prim = stage.GetPrimAtPath(body_path1)
        body_2_prim = stage.GetPrimAtPath(body_path2)
        xform_body_1 = UsdGeom.Xformable(body_1_prim)
        xform_body_2 = UsdGeom.Xformable(body_2_prim)
        transform_body_1 = xform_body_1.ComputeLocalToWorldTransform(0.0)
        transform_body_2 = xform_body_2.ComputeLocalToWorldTransform(0.0)
        t12 = np.matmul(
            np.linalg.inv(transform_body_1).T, np.array(transform_body_2).T
        ).T
        translate_body_12 = Gf.Vec3f([t12[3][0], t12[3][1], t12[3][2]])
        Q_body_12 = Gf.Transform(Gf.Matrix4d(t12.tolist())).GetRotation().GetQuat()

        # Set the transform between the bodies inside the joint
        joint.CreateLocalPos0Attr().Set(translate_body_12)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(Q_body_12))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    else:
        # Set the transform between the bodies inside the joint
        joint.CreateLocalPos0Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    return joint


def createRevoluteJoint(
    stage: Usd.Stage,
    path: str,
    body_path1: str = None,
    body_path2: str = None,
    axis: str = "Z",
    limit_low: float = None,
    limit_high: float = None,
    enable_drive: bool = False,
    damping: float = 1e3,
    stiffness: float = 1e6,
    force_limit: float = None,
) -> UsdPhysics.RevoluteJoint:
    """
    Creates a revolute joint between two bodies.

    Args:
        stage (Usd.Stage): The stage to create the revolute joint.
        path (str): The path of the revolute joint.
        body_path1 (str, optional): The path of the first body.
        body_path2 (str, optional): The path of the second body.
        axis (str, optional): The axis of rotation.
        limit_low (float, optional): The lower limit of the joint.
        limit_high (float, optional): The upper limit of the joint.
        enable_drive (bool, optional): Enable or disable the drive.
        damping (float, optional): The damping of the drive.
        stiffness (float, optional): The stiffness of the drive.
        force_limit (float, optional): The force limit of the drive.

    Returns:
        UsdPhysics.RevoluteJoint: The revolute joint.
    """

    # Create revolute joint
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)

    # Set body targets
    if not body_path1 is None:
        joint.CreateBody0Rel().SetTargets([body_path1])
    if not body_path2 is None:
        joint.CreateBody1Rel().SetTargets([body_path2])

    if (body_path1 is not None) and (body_path2 is not None):
        # Get from the simulation the position/orientation of the bodies
        body_1_prim = stage.GetPrimAtPath(body_path1)
        body_2_prim = stage.GetPrimAtPath(body_path2)
        xform_body_1 = UsdGeom.Xformable(body_1_prim)
        xform_body_2 = UsdGeom.Xformable(body_2_prim)
        transform_body_1 = xform_body_1.ComputeLocalToWorldTransform(0.0)
        transform_body_2 = xform_body_2.ComputeLocalToWorldTransform(0.0)
        t12 = np.matmul(
            np.linalg.inv(transform_body_1).T, np.array(transform_body_2).T
        ).T
        translate_body_12 = Gf.Vec3f([t12[3][0], t12[3][1], t12[3][2]])
        Q_body_12 = Gf.Transform(Gf.Matrix4d(t12.tolist())).GetRotation().GetQuat()

        # Set the transform between the bodies inside the joint
        joint.CreateLocalPos0Attr().Set(translate_body_12)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(Q_body_12))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    else:
        # Set the transform between the bodies inside the joint
        joint.CreateLocalPos0Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    if axis in ["X", "Y", "Z"]:
        joint.CreateAxisAttr(axis)
    else:
        raise ValueError("Axis must be X, Y or Z")

    if limit_low is not None:
        joint.CreateLowerLimitAttr(limit_low)
    if limit_high is not None:
        joint.CreateUpperLimitAttr(limit_high)

    if enable_drive:
        joint_prim = stage.GetPrimAtPath(joint.GetPath())
        createDrive(
            joint_prim,
            token="angular",
            damping=damping,
            stiffness=stiffness,
            max_force=force_limit,
        )
    return joint


def createPrismaticJoint(
    stage: Usd.Stage,
    path: str,
    body_path1: str = None,
    body_path2: str = None,
    axis: str = "Z",
    limit_low: float = None,
    limit_high: float = None,
    enable_drive: bool = False,
    damping: float = 1e3,
    stiffness: float = 1e6,
    force_limit: float = None,
) -> UsdPhysics.PrismaticJoint:
    """
    Creates a prismatic joint between two bodies.

    Args:
        stage (Usd.Stage): The stage to create the revolute joint.
        path (str): The path of the revolute joint.
        body_path1 (str, optional): The path of the first body.
        body_path2 (str, optional): The path of the second body.
        axis (str, optional): The axis of rotation.
        limit_low (float, optional): The lower limit of the joint.
        limit_high (float, optional): The upper limit of the joint.
        enable_drive (bool, optional): Enable or disable the drive.
        damping (float, optional): The damping of the drive.
        stiffness (float, optional): The stiffness of the drive.
        force_limit (float, optional): The force limit of the drive.

    Returns:
        UsdPhysics.PrismaticJoint: The prismatic joint.
    """

    # Create revolute joint
    joint = UsdPhysics.PrismaticJoint.Define(stage, path)
    # Set body targets
    if body_path1 is not None:
        joint.CreateBody0Rel().SetTargets([body_path1])
    if body_path2 is not None:
        joint.CreateBody1Rel().SetTargets([body_path2])

    if (body_path1 is not None) and (body_path2 is not None):
        # Get from the simulation the position/orientation of the bodies
        body_1_prim = stage.GetPrimAtPath(body_path1)
        body_2_prim = stage.GetPrimAtPath(body_path2)
        xform_body_1 = UsdGeom.Xformable(body_1_prim)
        xform_body_2 = UsdGeom.Xformable(body_2_prim)
        transform_body_1 = xform_body_1.ComputeLocalToWorldTransform(0.0)
        transform_body_2 = xform_body_2.ComputeLocalToWorldTransform(0.0)
        t12 = np.matmul(
            np.linalg.inv(transform_body_1).T, np.array(transform_body_2).T
        ).T
        translate_body_12 = Gf.Vec3f([t12[3][0], t12[3][1], t12[3][2]])
        Q_body_12 = Gf.Transform(Gf.Matrix4d(t12.tolist())).GetRotation().GetQuat()

        # Set the transform between the bodies inside the joint
        joint.CreateLocalPos0Attr().Set(translate_body_12)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(Q_body_12))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateAxisAttr(axis)
    else:
        # Set the transform between the bodies inside the joint
        joint.CreateLocalPos0Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateAxisAttr(axis)

    if axis in ["X", "Y", "Z"]:
        joint.CreateAxisAttr(axis)
    else:
        raise ValueError("Axis must be X, Y or Z")

    if limit_low is not None:
        joint.CreateLowerLimitAttr(limit_low)
    if limit_high is not None:
        joint.CreateUpperLimitAttr(limit_high)

    if enable_drive:
        joint_prim = stage.GetPrimAtPath(joint.GetPath())
        createDrive(
            joint_prim,
            token="linear",
            damping=damping,
            stiffness=stiffness,
            max_force=force_limit,
        )
    return joint


def createP3Joint(
    stage: Usd.Stage,
    path: str,
    body_path1: str,
    body_path2: str,
    damping: float = 1e3,
    stiffness: float = 1e6,
    articulation_root: str = None,
    prefix: str = "",
    enable_drive: bool = False,
) -> Tuple[
    UsdPhysics.PrismaticJoint, UsdPhysics.PrismaticJoint, UsdPhysics.PrismaticJoint
]:
    """
    Creates 3 Prismatic joints between two bodies. One for each axis (X,Y,Z).
    To create this joint, it needs to add two dummy bodies, to do this it
    needs to create them at the same position as the 1st body, and then
    apply a RigidBodyAPI and a MassAPI to them. The addition of these bodies is
    automated, and can fail to recover the position of the 1st body correctly.

    Args:
        stage (Usd.Stage): The stage to create the prismatic joint.
        path (str): The path of the prismatic joint.
        body_path1 (str): The path of the first body.
        body_path2 (str): The path of the second body.
        damping (float, optional): The damping of the drive.
        stiffness (float, optional): The stiffness of the drive.
        articulation_root (str, optional): The path of the articulation root.
        enable_drive (bool, optional): Enable or disable the drive.

    Returns:
        Tuple[UsdPhysics.PrismaticJoint, UsdPhysics.PrismaticJoint, UsdPhysics.PrismaticJoint]: The prismatic joints.
    """

    # Get the position/orientation of the two bodies
    body_1_prim = stage.GetPrimAtPath(body_path1)
    body_2_prim = stage.GetPrimAtPath(body_path2)
    if articulation_root is not None:
        root_prim = stage.GetPrimAtPath(articulation_root)
        transform_body_1 = getTransform(body_1_prim, root_prim)
        transform_body_2 = getTransform(body_2_prim, root_prim)
    else:
        xform_body_1 = UsdGeom.Xformable(body_1_prim)
        xform_body_2 = UsdGeom.Xformable(body_2_prim)
        transform_body_1 = xform_body_1.ComputeLocalToWorldTransform(0.0)
        transform_body_2 = xform_body_2.ComputeLocalToWorldTransform(0.0)

    translate_body_1 = Gf.Vec3f(
        [transform_body_1[3][0], transform_body_1[3][1], transform_body_1[3][2]]
    )
    Q_body_1d = Gf.Transform(transform_body_1).GetRotation().GetQuat()

    # Generates dummy bodies for the joints at the position of the 1st body
    xaxis_body_path, xaxis_body_prim = createXform(stage, path + "/x_axis_body")
    yaxis_body_path, yaxis_body_prim = createXform(stage, path + "/y_axis_body")
    setTranslate(xaxis_body_prim, translate_body_1)
    setTranslate(yaxis_body_prim, translate_body_1)
    setOrient(xaxis_body_prim, Q_body_1d)
    setOrient(yaxis_body_prim, Q_body_1d)
    applyRigidBody(xaxis_body_prim)
    applyRigidBody(yaxis_body_prim)
    applyMass(xaxis_body_prim, 0.0000001)
    applyMass(yaxis_body_prim, 0.0000001)

    # Create the 3 prismatic joints
    xaxis_joint = createPrismaticJoint(
        stage, path + "/" + prefix + "x_axis_joint", body_path1, xaxis_body_path, "X"
    )
    yaxis_joint = createPrismaticJoint(
        stage,
        path + "/" + prefix + "y_axis_joint",
        xaxis_body_path,
        yaxis_body_path,
        "Y",
    )
    zaxis_joint = createPrismaticJoint(
        stage, path + "/" + prefix + "z_axis_joint", yaxis_body_path, body_path2, "Z"
    )

    # Get the delta transform between the 1st and 2nd body
    t12 = np.matmul(np.linalg.inv(transform_body_1), transform_body_2)
    translate_body_12 = Gf.Vec3f([t12[3][0], t12[3][1], t12[3][2]])
    Q_body_12 = Gf.Transform(Gf.Matrix4d(t12.tolist())).GetRotation().GetQuat()

    # Set the transform between the bodies inside the joints
    xaxis_joint.CreateLocalPos0Attr().Set(Gf.Vec3f([0, 0, 0]))
    yaxis_joint.CreateLocalPos0Attr().Set(Gf.Vec3f([0, 0, 0]))
    zaxis_joint.CreateLocalPos0Attr().Set(translate_body_12)
    xaxis_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    yaxis_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    zaxis_joint.CreateLocalRot0Attr().Set(Gf.Quatf(Q_body_12))

    xaxis_joint.CreateLocalPos1Attr().Set(Gf.Vec3f([0, 0, 0]))
    yaxis_joint.CreateLocalPos1Attr().Set(Gf.Vec3f([0, 0, 0]))
    zaxis_joint.CreateLocalPos1Attr().Set(Gf.Vec3f([0, 0, 0]))
    xaxis_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    yaxis_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    zaxis_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    # Add drives to the joints
    if enable_drive:
        xaxis_drive = createDrive(
            stage.GetPrimAtPath(path + "/" + prefix + "x_axis_joint"),
            token="linear",
            damping=damping,
            stiffness=stiffness,
        )
        yaxis_drive = createDrive(
            stage.GetPrimAtPath(path + "/" + prefix + "y_axis_joint"),
            token="linear",
            damping=damping,
            stiffness=stiffness,
        )
        zaxis_drive = createDrive(
            stage.GetPrimAtPath(path + "/" + prefix + "z_axis_joint"),
            token="linear",
            damping=damping,
            stiffness=stiffness,
        )
    return (xaxis_joint, yaxis_joint, zaxis_joint)


def createP2Joint(
    stage: Usd.Stage,
    path: str,
    body_path1: str,
    body_path2: str,
    damping: float = 1e3,
    stiffness: float = 1e6,
    articulation_root: str = None,
    prefix: str = "",
    enable_drive: bool = False,
) -> Tuple[UsdPhysics.PrismaticJoint, UsdPhysics.PrismaticJoint]:
    """
    Creates 2 Prismatic joints between two bodies. One for each axis (X,Y).
    To create this joint, it needs to add one dummy body, to do this it
    needs to create it at the same position as the 1st body, and then
    apply a RigidBodyAPI and a MassAPI to it. The addition of these bodies is
    automated, and can fail to recover the position of the 1st body correctly.

    Args:
        stage (Usd.Stage): The stage to create the prismatic joint.
        path (str): The path of the prismatic joint.
        body_path1 (str): The path of the first body.
        body_path2 (str): The path of the second body.
        damping (float, optional): The damping of the drive.
        stiffness (float, optional): The stiffness of the drive.
        articulation_root (str, optional): The path of the articulation root.
        enable_drive (bool, optional): Enable or disable the drive.

    Returns:
        Tuple[UsdPhysics.PrismaticJoint, UsdPhysics.PrismaticJoint]: The prismatic joints.
    """

    # Get the position/orientation of the two bodies
    body_1_prim = stage.GetPrimAtPath(body_path1)
    body_2_prim = stage.GetPrimAtPath(body_path2)
    if articulation_root is not None:
        root_prim = stage.GetPrimAtPath(articulation_root)
        transform_body_1 = getTransform(body_1_prim, root_prim)
        transform_body_2 = getTransform(body_2_prim, root_prim)
    else:
        xform_body_1 = UsdGeom.Xformable(body_1_prim)
        xform_body_2 = UsdGeom.Xformable(body_2_prim)
        transform_body_1 = xform_body_1.ComputeLocalToWorldTransform(0.0)
        transform_body_2 = xform_body_2.ComputeLocalToWorldTransform(0.0)

    translate_body_1 = Gf.Vec3f(
        [transform_body_1[3][0], transform_body_1[3][1], transform_body_1[3][2]]
    )
    Q_body_1d = Gf.Transform(transform_body_1).GetRotation().GetQuat()

    # Generates dummy body for the joints at the position of the 1st body
    xaxis_body_path, xaxis_body_prim = createXform(stage, path + "/x_axis_body")
    setTranslate(xaxis_body_prim, translate_body_1)
    setOrient(xaxis_body_prim, Q_body_1d)
    applyRigidBody(xaxis_body_prim)
    applyMass(xaxis_body_prim, 0.0000001)

    # Create the 3 prismatic joints
    xaxis_joint = createPrismaticJoint(
        stage, path + "/" + prefix + "x_axis_joint", body_path1, xaxis_body_path, "X"
    )
    yaxis_joint = createPrismaticJoint(
        stage, path + "/" + prefix + "y_axis_joint", xaxis_body_path, body_path2, "Y"
    )

    # Get the delta transform between the 1st and 2nd body
    t12 = np.matmul(np.linalg.inv(transform_body_1), transform_body_2)
    translate_body_12 = Gf.Vec3f([t12[3][0], t12[3][1], t12[3][2]])
    Q_body_12 = Gf.Transform(Gf.Matrix4d(t12.tolist())).GetRotation().GetQuat()

    # Set the transform between the bodies inside the joints
    xaxis_joint.CreateLocalPos0Attr().Set(Gf.Vec3f([0, 0, 0]))
    yaxis_joint.CreateLocalPos0Attr().Set(translate_body_12)
    xaxis_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    yaxis_joint.CreateLocalRot0Attr().Set(Gf.Quatf(Q_body_12))

    xaxis_joint.CreateLocalPos1Attr().Set(Gf.Vec3f([0, 0, 0]))
    yaxis_joint.CreateLocalPos1Attr().Set(Gf.Vec3f([0, 0, 0]))
    xaxis_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    yaxis_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    # Add drives to the joints
    if enable_drive:
        xaxis_drive = createDrive(
            stage.GetPrimAtPath(path + "/" + prefix + "x_axis_joint"),
            token="linear",
            damping=damping,
            stiffness=stiffness,
        )
        yaxis_drive = createDrive(
            stage.GetPrimAtPath(path + "/" + prefix + "y_axis_joint"),
            token="linear",
            damping=damping,
            stiffness=stiffness,
        )

    return (xaxis_joint, yaxis_joint)


def create3DOFJoint(
    stage: Usd.Stage,
    path: str,
    body_path1: str,
    body_path2: str,
) -> UsdPhysics.FixedJoint:
    """
    Creates a D6 joint with limits between two bodies to constrain motionin in 2D plane.

    Args:
        stage (Usd.Stage): The stage to create the fixed joint.
        path (str): The path of the fixed joint.
        body_path1 (str): The path of the first body.
        body_path2 (str): The path of the second body.

    Returns:
        UsdPhysics.FixedJoint: The fixed joint.
    """

    # Create fixed joint
    joint = UsdPhysics.Joint.Define(stage, path)
    # Set body targets
    joint.CreateBody0Rel().SetTargets([body_path1])
    joint.CreateBody1Rel().SetTargets([body_path2])
    # Get from the simulation the position/orientation of the bodies
    translate = Gf.Vec3d(
        stage.GetPrimAtPath(body_path2).GetAttribute("xformOp:translate").Get()
    )
    Q = stage.GetPrimAtPath(body_path2).GetAttribute("xformOp:orient").Get()
    quat0 = Gf.Quatf(
        Q.GetReal(), Q.GetImaginary()[0], Q.GetImaginary()[1], Q.GetImaginary()[2]
    )
    # Set the transform between the bodies inside the joint
    joint.CreateLocalPos0Attr().Set(translate)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
    joint.CreateLocalRot0Attr().Set(quat0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    d6prim = stage.GetPrimAtPath(path)
    for dof in ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]:
        if dof in ["transZ", "rotX", "rotY"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6prim, dof)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)
    return joint
