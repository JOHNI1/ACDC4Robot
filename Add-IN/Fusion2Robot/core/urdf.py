# -*- coding: utf-8 -*-
# Author: Nuofan
# 20230811
# Contains functions for generating urdf
import adsk, adsk.core, adsk.fusion
from .link import Link
from .joint import Joint
from xml.etree.ElementTree import Element, SubElement
from . import utils
from ..commands.Fusion2Robot import constants

def get_link_name(link: Link) -> str:
    """
    Return 
    ---------
    name: str
        link's full path name
    """
    name = link.get_name()
    return name

def get_link_inertial_origin(link: Link) -> list[float]:
    """
    Return
    ---------
    inertial_origin: [x, y, z, roll, pitch, yaw]
        Pose (translation, rotation) of link's CoM frame C w.r.t link-frame L(parent-joint frame J)
        unit: m, radian
    """
    # the link is the first link(base_link)
    if link.get_parent_joint() is None:
        # for the first link which does not have parent joint
        # do not know the CoM is w.r.t world frame or the link frame
        # do not found the details from the description, but according to the figure from:
        # http://wiki.ros.org/urdf/XML/model
        # it seems to be the link frame

        # Let the orientation of center-of-mass frame C is same as link-frame L
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        w_CoM_x = link.phyPro.centerOfMass.x # point CoM's x coordinate w.r.t world-frame
        w_CoM_y = link.phyPro.centerOfMass.y
        w_CoM_z = link.phyPro.centerOfMass.z 
        w_Lo_x = link.pose.translation.x    # link-frame's origin point's x coordinat w.r.t world-frame
        w_Lo_y = link.pose.translation.y
        w_Lo_z = link.pose.translation.z
        # represent vector Lo-CoM which start from link-fram's origin point to CoM point w.r.t world-frame
        w_Lo_CoM = [[(w_CoM_x-w_Lo_x)*0.01], [(w_CoM_y-w_Lo_y)*0.01], [(w_CoM_z-w_Lo_z)*0.01]] # cm -> m
        # represent world-frame's orientation w.r.t link-frame
        L_R_w = [[link.pose.getCell(0, 0), link.pose.getCell(1, 0), link.pose.getCell(2, 0)],
                 [link.pose.getCell(0, 1), link.pose.getCell(1, 1), link.pose.getCell(2, 1)],
                 [link.pose.getCell(0, 2), link.pose.getCell(1, 2), link.pose.getCell(2, 2)]]

        L_Lo_CoM = utils.changeOrientation(L_R_w, w_Lo_CoM)
        inertial_origin = [L_Lo_CoM[0][0], L_Lo_CoM[1][0], L_Lo_CoM[2][0], roll, pitch, yaw]
    # the link is not the first link
    else:
        # for the link is not the first link, CoM frame is w.r.t parent-joint frame J
        joint = Joint(link.get_parent_joint())
        parent_joint_frame: adsk.core.Matrix3D = joint.get_joint_frame()
        from_origin, from_xAxis, from_yAxis, from_zAxis = parent_joint_frame.getAsCoordinateSystem()
        
        w_CoM_x = link.phyPro.centerOfMass.x # point CoM's x coordinate w.r.t world-frame
        w_CoM_y = link.phyPro.centerOfMass.y
        w_CoM_z = link.phyPro.centerOfMass.z 

        # Let the CoM-frame C has the same orientation with parent link frame
        CoM_frame_O: adsk.core.Point3D = adsk.core.Point3D.create()
        CoM_frame_O.set(w_CoM_x, w_CoM_y, w_CoM_z)
        CoM_frame: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
        CoM_frame_x: adsk.core.Vector3D = from_xAxis
        CoM_frame_y: adsk.core.Vector3D = from_yAxis
        CoM_frame_z: adsk.core.Vector3D = from_zAxis
        CoM_frame.setWithCoordinateSystem(CoM_frame_O, CoM_frame_x, CoM_frame_y, CoM_frame_z)
        to_origin, to_xAsix, to_yAxis, to_zAxis = CoM_frame.getAsCoordinateSystem()
        transform = adsk.core.Matrix3D.create()
        # transform.setToAlignCoordinateSystems(from_origin, from_xAxis, from_yAxis, from_zAxis, 
        #                                     to_origin, to_xAsix, to_yAxis, to_zAxis)
        transform = utils.coordinateTransform(parent_joint_frame, CoM_frame)

        inertial_origin = utils.matrix3d_2_pose(transform)

    return inertial_origin

def get_link_mass(link: Link) -> float:
    """
    Return
    ---------
    mass: float
        unit: kg
    """
    mass = link.phyPro.mass
    return mass

def get_link_inertia(link: Link) -> list[float]:
    """
    Get the inertia tensor elements of the link w.r.t the CoM frame C,
    CoM frame C has the same orientation with the parent frame:
    link-frame L for the first link, else the parent joint frame J

    Return
    ---------
    inertia_list: [ixx, iyy, izz, ixy, iyz, ixz]
        unit: kg*m^2
    """
    # First, get the inertia tensor and CoM w.r.t the world frame
    (_, w_ixx, w_iyy, w_izz, w_ixy, w_iyz, w_ixz) = link.phyPro.getXYZMomentsOfInertia() # unit: kg*cm^2
    x = link.phyPro.centerOfMass.x * 0.01 # get the x coordinate w.r.t world-frame, unit: m
    y = link.phyPro.centerOfMass.y * 0.01
    z = link.phyPro.centerOfMass.z * 0.01

    # Use parallel axis theorem to change inertia from w.r.t world-frame's origin to w.r.t CoM
    mass = link.phyPro.mass # unit: kg
    com_ixx = w_ixx*0.0001 - mass*(y**2+z**2) # kg*cm^2 -> kg*m^2
    com_iyy = w_iyy*0.0001 - mass*(x**2+z**2)
    com_izz = w_izz*0.0001 - mass*(x**2+y**2)
    com_ixy = w_ixy*0.0001 + mass*(x*y)
    com_iyz = w_iyz*0.0001 + mass*(y*z)
    com_ixz = w_ixz*0.0001 + mass*(x*z)
    inertia_tensor = [[com_ixx, com_ixy, com_ixz],
                        [com_ixy, com_iyy, com_iyz],
                        [com_ixz, com_iyz, com_izz]]
    
    # get the parent frame
    parent_joint = link.get_parent_joint()
    # parent frame for a link has parent joint, is the parent joint frame
    # else, is the link frame itselt
    if parent_joint is None:
        parent_frame: adsk.core.Matrix3D = link.pose
    else:
        joint = Joint(parent_joint)
        parent_frame: adsk.core.Matrix3D = joint.get_joint_frame()

    # Change inertia tensor to the orientation of parent frame
    R = utils.getRotationMatrix(parent_frame)
    R_T = utils.matrixTranspose(R)

    I = utils.matrixMul(utils.matrixMul(R_T, inertia_tensor), R)
    inertia_list = [I[0][0], I[1][1], I[2][2], I[0][1], I[1][2], I[0][2]]

    return inertia_list # unit: kg*m^2

def get_link_visual_name(link: Link) -> str:
    """
    Return
    ---------
    visual_name: str
    """
    visual_name = link.get_name() + "_visual"
    return visual_name

def get_link_collision_name(link: Link) -> str:
    """
    Return
    --------
    collision_name: str
    """
    collision_name = link.get_name() + "_collision"
    return collision_name

def get_mesh_origin(link: Link) -> list[float]:
    """
    The reference frame for mesh element of visual and collision w.r.t link frame L
    Return
    ---------
    mesh_origin: [x, y, z, roll, pitch, yaw]
        unit: m, radian
    """
    if link.get_parent_joint() is None:
        # for the first link which does not have parent joint
        # the mesh frame coincides with the link frame L
        mesh_origin = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:
        joint = Joint(link.get_parent_joint())
        parent_joint_frame: adsk.core.Matrix3D = joint.get_joint_frame()
        # link frame coincides with the mesh's frame
        link_frame: adsk.core.Matrix3D = link.pose
        from_origin, from_xAxis, from_yAxis, from_zAxis = parent_joint_frame.getAsCoordinateSystem()
        to_origin, to_xAsix, to_yAxis, to_zAxis = link_frame.getAsCoordinateSystem()

        transform = adsk.core.Matrix3D.create()
        # transform.setToAlignCoordinateSystems(from_origin, from_xAxis, from_yAxis, from_zAxis, 
        #                                     to_origin, to_xAsix, to_yAxis, to_zAxis)
        transform = utils.coordinateTransform(parent_joint_frame, link_frame)
        mesh_origin = utils.matrix3d_2_pose(transform)

    return mesh_origin

def get_link_visual_geo(link: Link) -> str:
    """
    A trimesh file for visual geometry
    Return:
    ---------
    mesh_loc: str
        the path to find mesh file
    """
    mesh_loc = "meshes/" + link.get_name() + ".stl"
    return mesh_loc

def get_link_collision_geo(link: Link) -> str:
    """
    A trimesh file for collision geometry
    Return:
    ---------
    mesh_loc: str
        the path to find mesh file
    """
    mesh_loc = "meshes/" + link.get_name() + ".stl"
    return mesh_loc

def get_link_element(link: Link) -> Element:
    """
    Return
    ---------
    link_ele: Element
        xml elements contains informations for link
    """
    # create a link element
    link_ele = Element("link")
    link_ele.attrib = {"name": get_link_name(link)}

    # add inertia sub-element
    inertial = SubElement(link_ele, "inertial")
    origin = SubElement(inertial, "origin")
    origin.attrib = {"xyz": "{} {} {}".format(get_link_inertial_origin(link)[0], get_link_inertial_origin(link)[1], get_link_inertial_origin(link)[2]),
                     "rpy": "{} {} {}".format(get_link_inertial_origin(link)[3], get_link_inertial_origin(link)[4], get_link_inertial_origin(link)[5])}
    mass = SubElement(inertial, "mass")
    mass.attrib = {"value": "{}".format(get_link_mass(link))}
    inertia = SubElement(inertial, "inertia")
    inertia.attrib = {"ixx": "{}".format(get_link_inertia(link)[0]),
                      "iyy": "{}".format(get_link_inertia(link)[1]),
                      "izz": "{}".format(get_link_inertia(link)[2]),
                      "ixy": "{}".format(get_link_inertia(link)[3]),
                      "iyz": "{}".format(get_link_inertia(link)[4]),
                      "ixz": "{}".format(get_link_inertia(link)[5])}
    
    # add visual sub-element
    visual = SubElement(link_ele, "visual")
    visual.attrib = {"name": "{}".format(get_link_visual_name(link))}
    origin_v = SubElement(visual, "origin")
    origin_v.attrib = {"xyz": "{} {} {}".format(get_mesh_origin(link)[0], get_mesh_origin(link)[1], get_mesh_origin(link)[2]),
                       "rpy": "{} {} {}".format(get_mesh_origin(link)[3], get_mesh_origin(link)[4], get_mesh_origin(link)[5])}
    geometry_v = SubElement(visual, "geometry")
    mesh_v = SubElement(geometry_v, "mesh")
    mesh_v.attrib = {"filename": get_link_visual_geo(link)}

    # add collision sub-element
    collision = SubElement(link_ele, "collision")
    collision.attrib = {"name": "{}".format(get_link_collision_name(link))}
    origin_c = SubElement(collision, "origin")
    origin_c.attrib = {"xyz": "{} {} {}".format(get_mesh_origin(link)[0], get_mesh_origin(link)[1], get_mesh_origin(link)[2]),
                       "rpy": "{} {} {}".format(get_mesh_origin(link)[3], get_mesh_origin(link)[4], get_mesh_origin(link)[5])}
    geometry_c = SubElement(collision, "geometry")
    mesh_c = SubElement(geometry_c, "mesh")
    mesh_c.attrib = {"filename": get_link_collision_geo(link)}

    return link_ele

def get_joint_name(joint: Joint) -> str:
    """
    Return
    ---------
    joint_name: str
    """
    joint_name = joint.get_name()
    return joint_name

def get_joint_type(joint: Joint) -> str:
    """
    Return
    ---------
    joint_type: str
        fixed, revolute, prismatic, continuous
        currently, only support urdf joint types above
    """
    urdf_joint_type_list = ["fixed", "revolute", "prismatic"]
    if joint.joint.jointMotion.jointType <= 2:
        urdf_joint_type = urdf_joint_type_list[joint.joint.jointMotion.jointType]
        if urdf_joint_type == "revolute" and (get_joint_limit(joint) is None):
            urdf_joint_type = "continuous"
    else:
        # other joint types are not supported yet
        pass
    return urdf_joint_type

def get_joint_origin(joint: Joint) -> list[float]:
    """
    Return
    ---------
    joint_origin: [x, y, z, roll, pitch, yaw]
        describe the pose of the joint frame J w.r.t parent frame(parent link frame L or parent link's parent jont frame J)
        unit: m, radian
    """
    parent_link: adsk.fusion.Occurrence = joint.parent
    parent_link: Link = Link(parent_link)
    parent_joint: adsk.fusion.Joint = parent_link.get_parent_joint()

    # get the parent frame
    if parent_joint is None:
        # if the parent link does not have parent joint(which means the root link), 
        # then the parent link frame is the parent frame
        parent_frame: adsk.core.Matrix3D = parent_link.pose
    else:
        parent_joint: Joint = Joint(parent_joint)
        parent_frame = parent_joint.get_joint_frame()
    
    # get joint frame w.r.t world frame
    joint_frame = joint.get_joint_frame()

    transform = adsk.core.Matrix3D.create()
    transform = utils.coordinateTransform(parent_frame, joint_frame)
    joint_origin = utils.matrix3d_2_pose(transform)

    return joint_origin

def get_joint_parent(joint: Joint) -> Link:
    """
    Return
    ---------
    parent_link: Link
        parent link of the joint
    """
    joint_parent: adsk.fusion.Occurrence = joint.parent
    parent_link: Link = Link(joint_parent)
    return parent_link

def get_joint_child(joint: Joint) -> Link:
    """
    Return
    ---------
    child_link: Link
        child link of the joint
    """
    joint_child: adsk.fusion.Occurrence = joint.child
    child_link: Link = Link(joint_child)
    return child_link

def get_joint_axis(joint: Joint) -> list[float]:
    """
    Return
    ---------
    axis: [x, y, z]
        joint axis specified in the joint frame
        if joint type is rigid, return None
    """
    # urdf joint axis is expressed w.r.t parent frame, which is the link frame or the parent joint frame
    if joint.joint.jointMotion.jointType == 0: # RigidJointType
        w_axis = None 
        axis = None
        return axis
    elif joint.joint.jointMotion.jointType == 1: # RevoluteJointType
        w_axis = [round(i, 6) for i in joint.joint.jointMotion.rotationAxisVector.asArray()] # In Fusion360, returned axis is normalized
    elif joint.joint.jointMotion.jointType == 2: # SliderJointType
        w_axis = [round(i, 6) for i in joint.joint.jointMotion.slideDirectionVector.asArray()] # In Fusion360, returned axis is normalized
    
    J_axis = None
    joint_frame: adsk.core.Matrix3D = joint.get_joint_frame()
    w_R_J = utils.getRotationMatrix(joint_frame) # represent joint-frame J's orientation w.r.t world-frame w
    J_R_w = utils.matrixTranspose(w_R_J) # for a rotation matrix, its inverse is its transpose

    w_axis = [[w_axis[0]], [w_axis[1]], [w_axis[2]]] # transform from 1*3 to 3*1 list
    J_axis = utils.changeOrientation(J_R_w, w_axis)
    axis = [J_axis[0][0], J_axis[1][0], J_axis[2][0]]

    return axis

def get_joint_limit(joint: Joint) -> list[float]:
    """
    required only for revolute and prismatic joint
    Return
    ---------
    limit: [lower, upper, effort, velocity]
        lower: optional, lower joint limit; unit: radian or m
        upper: optional, upper joint limit; unit: radian or m
        effort: required, the maximum joint effort, default: 1,000,000, unit: N or Nm
        velocity: required, the maximum joint velocity, default: 1,000,000, unit: m/s or rad/s
        if joint type is rigid, return None
    """
    if joint.joint.jointMotion.jointType == 0: # RigidJointType
        return None
    elif joint.joint.jointMotion.jointType == 1: # RevoluteJointType
        max_enabled = joint.joint.jointMotion.rotationLimits.isMaximumValueEnabled
        min_enabled = joint.joint.jointMotion.rotationLimits.isMinimumValueEnabled
        if max_enabled and min_enabled:
            # unit: radians
            lower_limit = joint.joint.jointMotion.rotationLimits.minimumValue
            upper_limit = joint.joint.jointMotion.rotationLimits.maximumValue
            return [round(lower_limit, 6), round(upper_limit, 6), 1_000_000, 1_000_000]
        else:
            return None
    elif joint.joint.jointMotion.jointType == 2: # SliderJointType
        max_enabled = joint.joint.jointMotion.slideLimits.isMaximumValueEnabled
        min_enabled = joint.joint.jointMotion.slideLimits.isMinimumValueEnabled
        if max_enabled and min_enabled:
            lower_limit = joint.joint.jointMotion.slideLimits.minimumValue * 0.01 # cm -> m
            upper_limit = joint.joint.jointMotion.slideLimits.maximumValue * 0.01 # cm -> m
            return [round(lower_limit, 6), round(upper_limit, 6), 1_000_000, 1_000_000]
        else:
            return None
    

def get_joint_element(joint: Joint) -> Element:
    """
    Return
    ---------
    joint_ele: Element
        xml elements contains informations for joint
    """
    joint_ele = Element("joint")
    joint_ele.attrib = {"name": get_joint_name(joint),
                        "type": get_joint_type(joint)}
    
    # add joint origin element
    origin = SubElement(joint_ele, "origin")
    origin.attrib = {"xyz": "{} {} {}".format(get_joint_origin(joint)[0], get_joint_origin(joint)[1], get_joint_origin(joint)[2]),
                     "rpy": "{} {} {}".format(get_joint_origin(joint)[3], get_joint_origin(joint)[4], get_joint_origin(joint)[5])}
    
    # add parent and child element
    parent = SubElement(joint_ele, "parent")
    parent.attrib = {"link": get_link_name(get_joint_parent(joint))}
    child = SubElement(joint_ele, "child")
    child.attrib = {"link": get_link_name(get_joint_child(joint))}

    # add axis1, urdf only has one axis for joint
    axis = get_joint_axis(joint)
    if axis is not None:
        axis_ele = SubElement(joint_ele, "axis")
        axis_ele.attrib = {"xyz": "{} {} {}".format(axis[0], axis[1], axis[2])}

    # add limits
    limit = get_joint_limit(joint)
    if limit is not None:
        limit_ele = SubElement(joint_ele, "limit")
        limit_ele.attrib = {"lower": "{}".format(limit[0]),
                            "upper": "{}".format(limit[1]),
                            "effort": "{}".format(limit[2]),
                            "velocity": "{}".format(limit[3])}
    
    return joint_ele

def get_urdf(joint: Joint, link: Link) -> Element:
    """
    
    """
    pass