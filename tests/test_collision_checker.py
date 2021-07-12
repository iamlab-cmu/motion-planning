from autolab_core import YamlConfig
from motion_planning.collision_checker.object_geometry import Box, PointCloud
from motion_planning.collision_checker.pybullet_collision_checker import PyBulletCollisionChecker
from pillar_state import State
cfg = YamlConfig("cfg/collision_checker.yaml") #TODO change to hydra

def make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg):
    pillar_state = State()
    pillar_state.update_property("frame:franka:joint_positions", [0,0,0,0,0,0,0])
    for object_name in object_name_to_pose.keys():
        pillar_state.update_property(f"frame:{object_name}:pose/position", object_name_to_pose[object_name][:3])
        pillar_state.update_property(f"frame:{object_name}:pose/quaternion",  object_name_to_pose[object_name][3:])


    return PyBulletCollisionChecker(pillar_state, object_name_to_geometry, cfg)

def test_2_boxes_not_in_collision():
    object_name_to_pose = {"box1": [0.1,0,0.05,0,0,0,1],
                           "box2": [0.3, 0, 0.05, 0, 0, 0, 1]}
    cube_length = 0.05
    object_name_to_geometry = {"box1":Box([cube_length, cube_length, cube_length]),
                           "box2":Box([cube_length, cube_length, cube_length])}
    collision_checker = make_collision_checker(object_name_to_pose, object_name_to_geometry, cfg)
    assert not collision_checker.pillar_state_in_collision()


def test_disabled_collisions():
    pass

test_2_boxes_not_in_collision()