from ..utils.utils import add_pb_tools_if_not_on_path, object_geometry_to_pybullet_object, find_robot_urdf, \
    RigidTransform_to_pb_pose

add_pb_tools_if_not_on_path()
import pybullet_tools.utils as pb_utils
import pybullet as p
import numpy as np
from collections import namedtuple
from autolab_core import RigidTransform
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics, check_ik_solver, is_ik_compiled


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

    def get_joint_limits(self, joint):
        return pb_utils.get_joint_limits(self.object_index, joint)
