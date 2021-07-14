from abc import ABC, abstractmethod
from ..utils.utils import add_pb_tools_if_not_on_path, joint_names_to_link_numbers

add_pb_tools_if_not_on_path()
import pybullet as p
from itertools import product
import numpy as np

import pybullet_tools.utils as pb_utils
from .base_collision_checker import BaseCollisionChecker
from motion_planning.utils import object_geometry_to_pybullet_object, get_pb_pose_from_pillar_state, \
    joint_conf_from_pillar_state
from ..envs.pybullet_robot_env import PyBulletRobotEnv


class PyBulletCollisionChecker(BaseCollisionChecker):
    def __init__(self, pillar_state, object_name_to_geometry, active_joints, cfg, disabled_collisions=[],
                 attached_object_names=[]):
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
        self._attached_object_names = attached_object_names
        self._grasp_link = joint_names_to_link_numbers(self._env.robot, [cfg.robot.grasp_link])[0]
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

    def joint_conf_in_collision(self, conf, disabled_collisions=()):
        """
        TODO turn on disabled_collisions
        :param conf:
        :return: whether the robot with configuration conf will collide
        with obstacles in the scene (besides those in disabled_collisions)
        """
        return self._pb_robot_collision_fn(conf)

    def pillar_state_in_collision(self):  # mostly for testing
        joint_conf = joint_conf_from_pillar_state(self._pillar_state, self._robot_name, self._active_joint_numbers)
        return self.joint_conf_in_collision(joint_conf) or self._workspace_collisions()

    def ompl_state_in_collision(self, ompl_state):
        joint_conf = [ompl_state[i] for i in range(len(self._active_joint_numbers))]
        return self.joint_conf_in_collision(joint_conf) or self._workspace_collisions()

    def make_attachments(self):
        attachments = []
        robot_to_world = pb_utils.get_link_pose(self._env.robot, self._grasp_link)
        for obj_name in self._attached_object_names:
            obj_to_world = pb_utils.get_link_pose(self._env.object_name_to_object_id[obj_name], -1)
            grasp = pb_utils.multiply(pb_utils.invert(robot_to_world), obj_to_world)
            attachment = pb_utils.Attachment(self._env.robot, self._grasp_link, grasp,
                                             self._env.object_name_to_object_id[obj_name])
            attachment.assign()
            attachments.append(attachment)
        return attachments

    def _update_collision_fn(self, new_attachment_names=None):
        if new_attachment_names is not None:
            self._attached_object_names = new_attachment_names
        start_joint_positions = joint_conf_from_pillar_state(self._pillar_state, self._robot_name,
                                                             self._active_joint_numbers)
        self._env.set_conf(self._active_joint_numbers, start_joint_positions)
        self._obstacles = self._env.object_name_to_object_id.values()
        attachments = self.make_attachments()
        self._pb_robot_collision_fn = pb_utils.get_collision_fn(self._env.robot, self._active_joint_numbers,
                                                                obstacles=self._obstacles, attachments=attachments,
                                                                self_collisions=True, disabled_collisions=set(),
                                                                custom_limits={},
                                                                max_distance=self._max_distance)  # TODO lagrassa pass in disabled colisions

    def close(self):
        self._env.close()
