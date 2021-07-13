import sys
import os
import motion_planning

def add_pb_tools_if_not_on_path():
    path = sys.path
    found_pb_tools = False
    for dir in path:
        if "pybullet_tools" in dir:
            found_pb_tools = True
            return
    if not found_pb_tools:
        print("Did not find pybullet_tools. Adding to path")
        pb_tools_path = find_pb_tools_path_from_module()
        sys.path.append(pb_tools_path)


def find_franka_urdf():
    franka_path = "../assets/franka_description/robots/franka_panda_dynamics.urdf"
    module_path = find_motion_planning_module()
    return os.path.join(module_path, franka_path)


def find_motion_planning_module():
    module_path = os.path.dirname(motion_planning.__file__)
    return module_path


def find_pb_tools_path_from_module():
    module_path = find_motion_planning_module()
    pb_tools_path = os.path.join(module_path, "../deps/pybullet-planning")
    return pb_tools_path
