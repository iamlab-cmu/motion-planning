from abc import ABC, abstractmethod
from ..utils.utils import add_pb_tools_if_not_on_path, joint_names_to_link_numbers

add_pb_tools_if_not_on_path()
import pybullet as p
from itertools import product
import numpy as np

import pybullet_tools.utils as pb_utils
from .base_collision_checker import BaseCollisionChecker
from motion_planning.utils import object_geometry_to_pybullet_object, get_pb_pose_from_pillar_state
from ..envs.pybullet_robot_env import PyBulletRobotEnv


class PyBulletCollisionChecker(BaseCollisionChecker):
    def __init__(self, pillar_state, object_name_to_geometry, active_joints, cfg, disabled_collisions=[]):
        """

        :param pillar_state: TODO lagrassa fill in docs
        :param object_name_to_geometry:
        :param cfg:
        :param disabled_collisions:
        """
        super().__init__(pillar_state, object_name_to_geometry, active_joints, cfg)
        self._cfg = cfg
        self._max_distance = 0
        self._robot_name = cfg["robot"]["robot_name"]
        self._env = PyBulletRobotEnv(pillar_state,
                                     object_name_to_geometry,
                                     self._robot_urdf_fn,
                                     vis=cfg["collision_checking"]["gui"])
        self._disabled_collisions = disabled_collisions
        self._active_joint_numbers = joint_names_to_link_numbers(self._env.robot, self._active_joints)
        self._update_collision_fn()

    def _workspace_collisions(self):
        """
        Collisions between objects in the workspace (not caused by robot pose)
        """
        for body1, body2 in product(self._obstacles, self._obstacles):
            if (body1 == body2):
                continue
            if pb_utils.pairwise_link_collision(body1, -1, body2, -1):  # -1 for base link
                return True
        return False

    def _joint_conf_in_collision(self, conf, disabled_collisions=()):
        """
        TODO turn on disabled_collisions
        :param conf:
        :return: whether the robot with configuration conf will collide
        with obstacles in the scene (besides those in disabled_collisions)
        """
        return self._pb_robot_collision_fn(conf)

    def pillar_state_in_collision(self):  # mostly for testing
        joint_conf = np.array(self._pillar_state.get_values_as_vec([f"frame:{self._robot_name}:joint_positions"]))[
            self._active_joint_numbers]
        return self._joint_conf_in_collision(joint_conf) or self._workspace_collisions()

    def ompl_state_in_collision(self, ompl_state):
        joint_conf = [ompl_state[i] for i in range(len(self._active_joint_numbers))]
        return self._joint_conf_in_collision(joint_conf) or self._workspace_collisions()

    def _update_collision_fn(self):
        self._obstacles = self._env.object_name_to_object_id.values()
        self._pb_robot_collision_fn = pb_utils.get_collision_fn(self._env.robot, self._active_joint_numbers,
                                                                obstacles=self._obstacles, attachments=[],
                                                                self_collisions=True, disabled_collisions=set(),
                                                                custom_limits={},
                                                                max_distance=self._max_distance)  # TODO lagrassa pass in disabled colisions

    def close(self):
        self._env.close()
