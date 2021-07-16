from collections import namedtuple

import numpy as np
from autolab_core import RigidTransform

import motion_planning.pybullet_tools.utils as pb_utils
from motion_planning.pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics
from ..utils.utils import find_robot_urdf, \
    RigidTransform_to_pb_pose


class PyBulletRobotModel:
    def __init__(self, robot_urdf_fn):
        self._robot_urdf_fn = find_robot_urdf(robot_urdf_fn)
        self._obj_index = None

    @property
    def robot_urdf_fn(self):
        return self._robot_urdf_fn

    @property
    def object_index(self):
        return self._obj_index

    def set_conf(self, joints, joint_positions):
        pb_utils.set_joint_positions(self._obj_index, joints, joint_positions)

    def set_pybullet_obj_index(self, obj_index):
        self._obj_index = obj_index

    def forward_kinematics(self, joint_indices, joint_values, tool_link_idx):
        pb_utils.set_joint_positions(self.object_index, joint_indices, joint_values)
        link_pose = pb_utils.get_link_pose(tool_link_idx)
        return link_pose

    def inverse_kinematics(self, goal_ee_pose, pos_tolerance=1e-3, ori_tolerance=np.pi * 1e-3, tool_link=7,
                           return_ik_joint_indices=False):
        if isinstance(goal_ee_pose, RigidTransform):
            goal_ee_pose = RigidTransform_to_pb_pose(goal_ee_pose)
        IKFastInfo = namedtuple('IKFastInfo', ['module_name', 'base_link', 'ee_link', 'free_joints'])
        info = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0', ee_link='panda_link7',
                          free_joints=['panda_joint6'])
        ik_joints = get_ik_joints(self.object_index, info, tool_link)
        pb_kwargs = {"pos_tolerance": pos_tolerance, "ori_tolerance": ori_tolerance, "max_attempts": 5,
                     "max_time": 500000000, "fixed_joints": []}
        conf = next(
            either_inverse_kinematics(self.object_index, info, tool_link, goal_ee_pose, use_pybullet=True, **pb_kwargs),
            None)
        if return_ik_joint_indices:
            return ik_joints, conf
        return conf

    def get_joint_limits(self, joint_name):
        joint_idx = self.joint_names_to_joint_numbers([joint_name])[0]
        return pb_utils.get_joint_limits(self.object_index, joint_idx)

    def joint_names_to_joint_numbers(self, joint_names):
        all_joints = pb_utils.get_joints(self.object_index)
        all_joint_names = pb_utils.get_joint_names(self.object_index, all_joints)
        joint_numbers = []
        for joint_name in joint_names:
            joint_number = all_joint_names.index(joint_name)
            joint_numbers.append(joint_number)
        return joint_numbers

    def link_names_to_link_numbers(self, link_names):
        all_links = pb_utils.get_all_links(self.object_index)
        all_link_names = pb_utils.get_link_names(self.object_index, all_links)
        link_numbers = []
        for link_name in link_names:
            link_number = all_link_names.index(link_name)
            link_numbers.append(link_number)
        return link_numbers
