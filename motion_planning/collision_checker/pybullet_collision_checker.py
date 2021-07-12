from abc import ABC, abstractmethod

import sys
sys.path.insert(0, '/home/lagrassa/git/pybullet-planning')

import pybullet as p
import pybullet_tools.utils as pb_utils
from .base_collision_checker import BaseCollisionChecker
from .collision_check_utils import object_geometry_to_pybullet_object, get_pb_pose_from_pillar_state


class PyBulletCollisionChecker(BaseCollisionChecker):
    def __init__(self, pillar_state, object_name_to_geometry, cfg):
        super().__init__(pillar_state, object_name_to_geometry, cfg)
        self._cfg = cfg
        self._object_name_to_object_id = {}
        self._robot_name = cfg.get("robot_name", "franka")
        self._object_name_to_object_id = {}
        self._setup_env(vis=cfg["gui"])


    def _joint_conf_in_collision(self, conf, disabled_collisions=()):
        """

        :param conf:
        :return: whether the robot with configuration conf will collide
        with obstacles in the scene (besides those in disabled_collisions)
        """
        raise NotImplementedError

    def _setup_env(self, vis=False):
        # load robot TODO lagrassa load any robot from urdf
        pb_utils.connect(use_gui=vis)
        pb_utils.add_data_path()

        plane = p.loadURDF("plane.urdf")
        with pb_utils.LockRenderer():
            with pb_utils.HideOutput(True):
                self.robot = pb_utils.load_pybullet(self._robot_urdf, fixed_base=True)
        for object_name in self._object_name_to_geometry.keys():
            self._object_name_to_object_id[object_name] = object_geometry_to_pybullet_object(
                self._object_name_to_geometry[object_name])
            obj_pose = get_pb_pose_from_pillar_state(self._pillar_state, object_name)
            pb_utils.set_pose(self._object_name_to_object_id[object_name], obj_pose)
        input("Workspace OK?")

    def pillar_state_in_collision(self):
        joint_conf = self._pillar_state.get_values_as_vec([f"frame:{self._robot_name}:joint_positions"])
        return self._joint_conf_in_collision(joint_conf)

