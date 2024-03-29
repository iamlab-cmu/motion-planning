from copy import deepcopy
from itertools import product

import motion_planning.pybullet_tools.utils as pb_utils
from motion_planning.utils import joint_conf_from_pillar_state
from .base_collision_checker import BaseCollisionChecker


class PyBulletCollisionChecker(BaseCollisionChecker):
    def __init__(self, pybullet_env, pillar_state, object_name_to_geometry, active_joints, cfg, robot_model=None,
                 disabled_collisions=[],
                 attachment_names_to_grasp = {},
                 attached_object_names=[]):
        """

        :param pillar_state: TODO lagrassa fill in docs
        :param object_name_to_geometry:
        :param cfg:
        :param disabled_collisions:
        :param attachment_names_to_grasp: Dictionary of object name to a tuple {obj_name:(position, quat: xyzw)}. If empty, will use
        pillar_state to compute grasp based on current transform from the grasp_link to the attached object
        """
        super().__init__(pillar_state, object_name_to_geometry, active_joints, cfg, robot_model)
        self._cfg = cfg
        self._max_distance = 0
        self._robot_name = cfg["robot"]["robot_name"]
        self._robot_model = robot_model
        self._env = pybullet_env
        self._disabled_collisions = disabled_collisions
        self._active_joint_numbers = self._robot_model.joint_names_to_joint_numbers(self._active_joints)
        self._attached_object_names = attached_object_names
        self._attachment_names_to_grasp = attachment_names_to_grasp
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

    def make_attachments(self, attachment_name_to_grasp=None):
        attachments = []
        robot_to_world = pb_utils.get_link_pose(self._robot_model.object_index, self._robot_model.grasp_link_index)
        for obj_name in self._attached_object_names:
            if obj_name not in attachment_name_to_grasp.keys():
                obj_to_world = pb_utils.get_link_pose(self._env.object_name_to_object_id[obj_name], -1)
                grasp = pb_utils.multiply(pb_utils.invert(robot_to_world), obj_to_world)
            else:
                grasp = attachment_name_to_grasp[obj_name]
            attachment = pb_utils.Attachment(self._robot_model.object_index, self._robot_model.grasp_link_index, grasp,
                                             self._env.object_name_to_object_id[obj_name])
            attachment.assign()
            attachments.append(attachment)
        return attachments

    def update_state(self, pillar_state, new_attachment_names=None):
        self._pillar_state = deepcopy(pillar_state)
        self._update_collision_fn(new_attachment_names=new_attachment_names)

    def _update_collision_fn(self, new_attachment_names=None, new_attachment_names_to_grasp=None):
        if new_attachment_names is not None:
            self._attached_object_names = new_attachment_names
        if new_attachment_names_to_grasp is not None:
            self._attachment_names_to_grasp = new_attachment_names_to_grasp
        start_joint_positions = joint_conf_from_pillar_state(self._pillar_state, self._robot_name,
                                                             self._active_joint_numbers)
        self._robot_model.set_conf(self._active_joint_numbers, start_joint_positions)
        self._obstacles = self._env.object_name_to_object_id.values()
        attachments = self.make_attachments(attachment_name_to_grasp=self._attachment_names_to_grasp)
        self._pb_robot_collision_fn = pb_utils.get_collision_fn(self._robot_model.object_index,
                                                                self._active_joint_numbers,
                                                                obstacles=self._obstacles, attachments=attachments,
                                                                self_collisions=True, disabled_collisions=set(),
                                                                custom_limits={},
                                                                max_distance=self._max_distance)  # TODO lagrassa pass in disabled colisions

    def close(self):
        self._env.close()
