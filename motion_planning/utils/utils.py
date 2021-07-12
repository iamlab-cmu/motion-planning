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
    franka_path = "/models/franka_description/robots/panda_arm_hand.urdf"
    pb_tools_path = find_pb_tools_path_from_module()
    return pb_tools_path + franka_path



def find_pb_tools_path_from_module():
    module_path = os.path.dirname(motion_planning.__file__)
    pb_tools_path = f"{module_path}/../deps/pybullet-planning"
    return pb_tools_path
