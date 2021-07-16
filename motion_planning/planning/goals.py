import logging
from abc import ABC, abstractmethod

from ..utils import add_ompl_to_sys_path, pb_pose_from_pose_arr

add_ompl_to_sys_path()
from ompl import base as ob

logger = logging.getLogger(__name__)
logger.setLevel("WARN")


class Goal(ABC):
    def __init__(self, goal_data):
        self._goal_data = goal_data

    @property
    def goal_data(self):
        return self._goal_data

    @abstractmethod
    def get_ompl_state(self, pspace):
        pass


class JointGoal(Goal):
    def __init__(self, joint_array_goal):
        super().__init__(joint_array_goal)

    def get_ompl_state(self, pspace):
        ompl_state = ob.State(pspace)
        joint_positions = self.goal_data
        for i, joint in enumerate(joint_positions):
            ompl_state[i] = joint
        return ompl_state


class CartesianGoal(Goal):
    def __init__(self, pose_array_goal):
        pb_pose_array_goal = pb_pose_from_pose_arr(pose_array_goal)
        super().__init__(pb_pose_array_goal)

    def get_ompl_state(self, pspace, robot_model):
        ompl_state = ob.State(pspace)
        ee_pose = self.goal_data
        joint_indices, joint_values = robot_model.inverse_kinematics(ee_pose, tool_link=robot_model.grasp_link_index,
                                                                     return_ik_joint_indices=True)
        for i, joint in enumerate(joint_indices):
            ompl_state[i] = joint_values[i]
        return ompl_state
