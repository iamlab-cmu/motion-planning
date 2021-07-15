import sys
import numpy as np
import os
import motion_planning
from autolab_core import RigidTransform
from hydra.utils import to_absolute_path

from motion_planning.models.object_geometry import PointCloud, Box


def find_pb_tools_path_from_module():
    module_path = find_motion_planning_module()
    pb_tools_path = os.path.join(module_path, "../deps/pybullet-planning")
    return pb_tools_path


def find_motion_planning_module():
    module_path = os.path.dirname(motion_planning.__file__)
    return module_path


def add_pb_tools_if_not_on_path():
    path = sys.path
    for dir in path:
        if "pybullet_tools" in dir:
            return
    print("Did not find pybullet_tools. Adding to path")
    pb_tools_path = find_pb_tools_path_from_module()
    sys.path.append(pb_tools_path)


add_pb_tools_if_not_on_path()
import pybullet_tools.utils as pb_utils


def find_robot_urdf(urdf_path):
    module_path = find_motion_planning_module()
    return os.path.join(module_path, "../", urdf_path)


def add_ompl_to_sys_path():
    path_to_ompl = to_absolute_path('deps/ompl-1.5.2/py-bindings')
    sys.path.insert(0, path_to_ompl)


def joint_names_to_link_numbers(robot_model, joint_names):
    # TODO lagrassa not sure how robust this is outside the franka
    link_names = [joint_name.replace("joint", "link") for joint_name in joint_names]
    all_links = pb_utils.get_all_links(robot_model.object_index)
    all_links.remove(-1)  # Not counting base link
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
    wxyz_quaternion = rt.quaternion[3:]
    xyzw_quaternion = wxyz_quaternion[1:] + [wxyz_quaternion[0]]
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
    position = pose_arr[:3]
    wxyz_quaternion = pose_arr[3:]
    xyzw_quaternion = wxyz_quaternion[1:] + [wxyz_quaternion[0]]
    return (position, xyzw_quaternion)


def joint_conf_from_pillar_state(pillar_state, robot_name, active_joint_numbers):
    joint_conf = np.array(pillar_state.get_values_as_vec([f"frame:{robot_name}:joint_positions"]))[
        active_joint_numbers]
    return joint_conf
