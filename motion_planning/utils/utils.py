import sys
import os
import motion_planning
from hydra.utils import to_absolute_path


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


def find_franka_urdf():
    franka_path = "assets/franka_description/robots/franka_panda_dynamics.urdf"
    return find_robot_urdf(franka_path)


def find_robot_urdf(urdf_path):
    module_path = find_motion_planning_module()
    return os.path.join(module_path, "../", urdf_path)


def add_ompl_to_sys_path():
    path_to_ompl = to_absolute_path('deps/ompl-1.5.2/py-bindings')
    sys.path.insert(0, path_to_ompl)


def joint_names_to_joint_numbers(robot, joint_names):
    # TODO lagrassa not sure how robust this is outside the franka
    link_names = [joint_name.replace("joint", "link") for joint_name in joint_names]
    all_links = pb_utils.get_all_links(robot)
    all_links.remove(-1)  # Not counting base link
    all_link_names = pb_utils.get_link_names(robot, all_links)
    link_numbers = []
    for link_name in link_names:
        link_number = all_link_names.index(link_name)
        link_numbers.append(link_number)
    return link_numbers
