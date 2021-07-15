import pybullet as p
from motion_planning.utils import add_pb_tools_if_not_on_path
from collections import namedtuple

add_pb_tools_if_not_on_path()
import pybullet_tools.utils as pb_utils
import time

from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics, check_ik_solver, is_ik_compiled
from motion_planning.envs.pybullet_robot_env import PyBulletRobotEnv
from pillar_state import State
import numpy as np
from omegaconf import OmegaConf

cfg = OmegaConf.load("cfg/collision_checker.yaml")


def make_env():
    start_pillar_state = State()
    start_joints = [0, -np.pi / 4, 0, -2.85496998, 0, 2.09382820, np.pi / 4]
    joint_nums = list(range(len(start_joints)))
    robot_urdf_fn = cfg.robot.path_to_urdf
    env = PyBulletRobotEnv(start_pillar_state, {}, robot_urdf_fn, vis=cfg.collision_checking.gui)
    env.set_conf(joint_nums, start_joints)
    return env


def test_ik():
    env = make_env()
    IKFastInfo = namedtuple('IKFastInfo', ['module_name', 'base_link', 'ee_link', 'free_joints'])
    info = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0', ee_link='panda_link7',
                      free_joints=['panda_joint7'])
    tool_link = 7
    ik_joints = get_ik_joints(env.robot, info, tool_link)
    start_pose = pb_utils.get_link_pose(env.robot, tool_link)
    end_pose = pb_utils.multiply(start_pose, pb_utils.Pose(pb_utils.Point(x=0.02, z=-0.14)))
    pos_tolerance = 1e-3
    ori_tolerance = 1e-3 * np.pi
    pb_kwargs = {"pos_tolerance": pos_tolerance / 2, "ori_tolerance": ori_tolerance, "max_attempts": 5,
                 "max_time": 500000000, "fixed_joints": []}
    conf = run_and_time_ik(end_pose, env, tool_link, info, use_pybullet=True, **pb_kwargs)
    pb_utils.set_joint_positions(env.robot, ik_joints, conf)
    tool_pose_using_conf = pb_utils.get_link_pose(env.robot, tool_link)
    pos_distance = np.linalg.norm(np.array(tool_pose_using_conf[0]) - np.array(end_pose[0]))
    assert pb_utils.is_point_close(end_pose[0], tool_pose_using_conf[0], tolerance=pos_tolerance)
    assert pb_utils.is_quat_close(end_pose[1], tool_pose_using_conf[1], tolerance=ori_tolerance)


def run_and_time_ik(end_pose, env, tool_link, info, use_pybullet=False, **kwargs):
    ikfast_compiled = is_ik_compiled(info)
    using_ikfast = ikfast_compiled and not use_pybullet
    if not use_pybullet and not ikfast_compiled:
        print("IKfast supposed to be used but not compiled or found")
    start_time = time.time()
    conf = next(either_inverse_kinematics(env.robot, info, tool_link, end_pose, use_pybullet=use_pybullet, **kwargs),
                None)
    end_time = time.time()
    print(f"Time to compute IK. Using ikfast? {using_ikfast}", end_time - start_time)
    return conf


test_ik()
