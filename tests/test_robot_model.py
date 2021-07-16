import pybullet as p
from motion_planning.utils import add_pb_tools_if_not_on_path
from collections import namedtuple

add_pb_tools_if_not_on_path()
import pybullet_tools.utils as pb_utils
import time

from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics, check_ik_solver, is_ik_compiled
from motion_planning.models.pybullet_robot_model import PyBulletRobotModel
from motion_planning.models.pybullet_robot_env import PyBulletRobotEnv
from pillar_state import State
import numpy as np
from omegaconf import OmegaConf

cfg = OmegaConf.load("cfg/collision_checker.yaml")


def make_env(robot_model):
    start_pillar_state = State()
    start_joints = [0, -np.pi / 4, 0, -2.85496998, 0, 2.09382820, np.pi / 4]
    joint_nums = list(range(len(start_joints)))
    env = PyBulletRobotEnv(start_pillar_state, {}, robot_model, vis=cfg.collision_checking.gui)
    robot_model.set_conf(joint_nums, start_joints)
    return env


def test_ik():
    robot_model = PyBulletRobotModel(cfg.robot.path_to_urdf)
    env = make_env(robot_model)
    pos_tolerance = 1e-3
    ori_tolerance = 1e-3 * np.pi
    tool_link = 7
    start_pose = pb_utils.get_link_pose(robot_model.object_index, tool_link)
    end_pose = pb_utils.multiply(start_pose, pb_utils.Pose(pb_utils.Point(x=0.02, z=-0.14)))
    ik_joints, conf = robot_model.inverse_kinematics(end_pose, return_ik_joint_indices=True)
    robot_model.set_conf(ik_joints, conf)
    tool_pose_using_conf = pb_utils.get_link_pose(robot_model.object_index, tool_link)  # avoiding FK for testing
    assert pb_utils.is_point_close(end_pose[0], tool_pose_using_conf[0], tolerance=pos_tolerance)
    assert pb_utils.is_quat_close(end_pose[1], tool_pose_using_conf[1], tolerance=ori_tolerance)


def test_joint_limits():
    robot_model = PyBulletRobotModel(cfg.robot.path_to_urdf)
    env = make_env(robot_model)
    # See https://frankaemika.github.io/docs/control_parameters.html
    manually_parsed_joint_limits_low = [-2.9671, -1.8326, -2.9671, -3.0, -2.9671, -0.0873, -2.9671]
    manually_parsed_joint_limits_high = [2.9671, 1.8326, 2.9671, 0.087, 2.9671, 3.0, 2.9671]
    for joint_idx in range(7):
        low, high = robot_model.get_joint_limits(joint_idx)
        assert low == manually_parsed_joint_limits_low[joint_idx]
        assert high == manually_parsed_joint_limits_high[joint_idx]


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
