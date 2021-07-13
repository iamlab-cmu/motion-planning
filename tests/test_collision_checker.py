from omegaconf import OmegaConf
import numpy as np
from motion_planning.collision_checker.object_geometry import Box, PointCloud
from motion_planning.collision_checker.pybullet_collision_checker import PyBulletCollisionChecker
from pillar_state import State

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


def make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg):
    pillar_state = State()
    joint_positions = [0., 0., 0., -1.5708, 0., 1.8675, 0., 0.02, 0.02]
    pillar_state.update_property("frame:franka:joint_positions", joint_positions)
    for object_name in object_name_to_pose.keys():
        pillar_state.update_property(f"frame:{object_name}:pose/position", object_name_to_pose[object_name][:3])
        pillar_state.update_property(f"frame:{object_name}:pose/quaternion", object_name_to_pose[object_name][3:])
    return PyBulletCollisionChecker(pillar_state, object_name_to_geometry, cfg)


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


def test_disabled_collisions():
    pass
