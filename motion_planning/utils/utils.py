import os
import sys

import numpy as np
from autolab_core import RigidTransform
from hydra.utils import to_absolute_path

import motion_planning
import motion_planning.pybullet_tools.utils as pb_utils
from ..models.object_geometry import PointCloud, Box


def find_pb_tools_path_from_module():
    module_path = find_motion_planning_module()
    pb_tools_path = os.path.join(module_path, "../deps/pybullet-planning")
    return pb_tools_path


def find_motion_planning_module():
    module_path = os.path.dirname(motion_planning.__file__)
    return module_path


def find_robot_urdf(urdf_path):
    module_path = find_motion_planning_module()
    return os.path.join(module_path, "../", urdf_path)


def add_ompl_to_sys_path():
    path_to_ompl = to_absolute_path('deps/ompl-1.5.2/py-bindings')
    sys.path.insert(0, path_to_ompl)


def joint_names_to_joint_numbers(robot_model, joint_names):
    all_joints = pb_utils.get_joints(robot_model.object_index)
    all_joint_names = pb_utils.get_joint_names(robot_model.object_index, all_joints)
    joint_numbers = []
    for joint_name in joint_names:
        joint_number = all_joint_names.index(joint_name)
        joint_numbers.append(joint_number)
    return joint_numbers


def link_names_to_link_numbers(robot_model, link_names):
    all_links = pb_utils.get_all_links(robot_model.object_index)
    all_link_names = pb_utils.get_link_names(robot_model.object_index, all_links)
    link_numbers = []
    for link_name in link_names:
        link_number = all_link_names.index(link_name)
        link_numbers.append(link_number)
    return link_numbers


def pb_pose_to_RigidTransform(pb_pose):
    translation = pb_pose[0]
    xyzw_quaternion = pb_pose[1]
    wxyz_quaternion = [xyzw_quaternion[-1], ] + xyzw_quaternion[:-1]
    rotation = RigidTransform.rotation_from_quaternion(wxyz_quaternion)
    return RigidTransform(translation=translation, rotation=rotation)


def RigidTransform_to_pb_pose(rt):
    position = rt.translation
    wxyz_quaternion = rt.quaternion
    xyzw_quaternion = np.hstack([wxyz_quaternion[1:], [wxyz_quaternion[0]]])
    return (position, xyzw_quaternion)


def object_geometry_to_pybullet_object(object_geometry):
    if isinstance(object_geometry, PointCloud):
        mesh = pb_utils.mesh_from_points(object_geometry.points)
        obj_from_mesh = pb_utils.create_mesh(mesh)
        return obj_from_mesh
    elif isinstance(object_geometry, Box):
        return pb_utils.create_box(*object_geometry.dims)
    else:
        raise ValueError(f"Invalid object geometry type: {object_geometry} ")


def get_pb_pose_from_pillar_state(pillar_state, obj_name):
    pose_arr = pillar_state.get_values_as_vec([f"frame:{obj_name}:pose/position", f"frame:{obj_name}:pose/quaternion"])
    return pb_pose_from_pose_arr(pose_arr)


def pb_pose_from_pose_arr(pose_arr):
    position = pose_arr[:3]
    wxyz_quaternion = pose_arr[3:]
    xyzw_quaternion = wxyz_quaternion[1:] + [wxyz_quaternion[0]]
    return (position, xyzw_quaternion)


def joint_conf_from_pillar_state(pillar_state, robot_name, active_joint_numbers):
    joint_conf = np.array(pillar_state.get_values_as_vec([f"frame:{robot_name}:joint_positions"]))[
        active_joint_numbers]
    return joint_conf
