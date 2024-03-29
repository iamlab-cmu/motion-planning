import numpy as np
from omegaconf import OmegaConf
from pillar_state import State

from motion_planning.collision_checker.pybullet_collision_checker import PyBulletCollisionChecker
from motion_planning.models.object_geometry import Box, PointCloud
from motion_planning.models.pybullet_robot_env import PyBulletRobotEnv
from motion_planning.models.pybullet_robot_model import PyBulletRobotModel

cfg = OmegaConf.load("cfg/collision_checker.yaml")


def make_box_pointcloud():
    box_height = 0.1
    box_y_len = 0.05
    box_x_len = 0.05
    points = np.array([
        [0, 0, 0],
        [box_x_len, 0, 0],
        [0, box_y_len, 0],
        [box_x_len, box_y_len, 0],
        [box_x_len, box_y_len, box_height],
        [0, box_y_len, box_height],
        [0, 0, box_height],
        [box_x_len, 0, box_height],
    ])
    return points


def make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg, robot_model=None,
                           attached_object_names=[]):
    pillar_state = State()
    joint_positions = [0., 0., 0., -1.5708, 0., 1.8675, 0., 0.02, 0.02]
    joint_positions = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
    pillar_state.update_property("frame:franka:joint_positions", joint_positions)
    if robot_model is None:
        robot_model = PyBulletRobotModel(cfg.robot)
    for object_name in object_name_to_pose.keys():
        pillar_state.update_property(f"frame:{object_name}:pose/position", object_name_to_pose[object_name][:3])
        pillar_state.update_property(f"frame:{object_name}:pose/quaternion", object_name_to_pose[object_name][3:])
    active_joints = [f"panda_joint{i}" for i in range(1, 8)]
    env = PyBulletRobotEnv(robot_model, vis=cfg.collision_checking.gui)
    env.initialize_workspace(pillar_state, object_name_to_geometry)
    return PyBulletCollisionChecker(env, pillar_state, object_name_to_geometry, active_joints, cfg,
                                    robot_model=robot_model,
                                    attached_object_names=attached_object_names)


def test_2_boxes_not_in_collision():
    object_name_to_pose = {"box1": [0.1, 0, 0.05, 0, 0, 0, 1],
                           "box2": [0.3, 0, 0.05, 0, 0, 0, 1]}
    cube_length = 0.05
    object_name_to_geometry = {"box1": Box([cube_length, cube_length, cube_length]),
                               "box2": Box([cube_length, cube_length, cube_length])}
    collision_checker = make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg)
    assert not collision_checker.pillar_state_in_collision()
    collision_checker.close()


def test_3_boxes_in_collision():
    object_name_to_pose = {"box1": [0.1, 0, 0.05, 0, 0, 0, 1],
                           "box2": [0.103, 0, 0.05, 0, 0, 0, 1],
                           "box3": [0.3, 0, 0.05, 0, 0, 0, 1]}
    cube_length = 0.02
    object_name_to_geometry = {"box1": Box([cube_length, cube_length, cube_length]),
                               "box2": Box([cube_length, cube_length, cube_length]),
                               "box3": Box([cube_length, cube_length, cube_length])}
    collision_checker = make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg)
    assert collision_checker.pillar_state_in_collision()
    collision_checker.close()


def test_2_meshes_not_in_collision():
    points = make_box_pointcloud()
    object_name_to_pose = {"mesh1": [0.5, 0, 0.05, 0, 0, 0, 1],
                           "mesh2": [0.8, 0, 0.05, 0, 0, 0, 1]}
    object_name_to_geometry = {"mesh1": PointCloud(points),
                               "mesh2": PointCloud(points)}
    collision_checker = make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg)
    assert not collision_checker.pillar_state_in_collision()
    collision_checker.close()


def test_2_meshes_in_collision():
    points = make_box_pointcloud()
    object_name_to_pose = {"mesh1": [0.5, 0, 0.05, 0, 0, 0, 1],
                           "mesh2": [0.5, 0.00, 0.05, 0, 0, 0, 1]}
    object_name_to_geometry = {"mesh1": PointCloud(points),
                               "mesh2": PointCloud(points)}
    collision_checker = make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg)
    assert collision_checker.pillar_state_in_collision()
    collision_checker.close()


def test_attachments():
    object_name_to_pose = {"box1": [0.31, 0, 0.45, 0, 0, 0, 1],
                           "box2": [0.52, 0, 0.32, 0, 0, 0, 1]}
    side_length_short = 0.02
    side_length_long = 0.08
    height = 0.03

    object_name_to_geometry = {"box1": Box([side_length_short, side_length_long, height]),
                               "box2": Box([0.05, 0.05, 0.05])}
    attached_object_names = ["box1"]
    collision_checker = make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg,
                                               attached_object_names=attached_object_names)
    assert not collision_checker.pillar_state_in_collision()
    new_joint_conf = [0, -.15, 0, -2.25, 0, 2.3, .79]
    assert collision_checker.joint_conf_in_collision(new_joint_conf)
    collision_checker.close()


def test_disabled_collisions():
    pass
