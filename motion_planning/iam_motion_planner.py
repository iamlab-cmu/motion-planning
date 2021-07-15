from hydra.utils import to_absolute_path

from motion_planning.utils import add_ompl_to_sys_path
from motion_planning.collision_checker import PyBulletCollisionChecker
from motion_planning.utils import (add_ompl_to_sys_path, add_pb_tools_if_not_on_path)
add_ompl_to_sys_path(), add_pb_tools_if_not_on_path()
from ompl import base as ob
from ompl import geometric as og
import pybullet_tools.utils as pb_utils


class IAMMotionPlanner():
    def __init__(self, cfg, collision_checker=None):
        self._ndims = len(cfg.active_joints)
        self._planner = cfg.planner
        self._collision_checker = collision_checker
        self._active_joints = cfg.active_joints
        self._cfg = cfg

        # robot
        # self._robot = ?

        # env
        # self._env = ?

        # planning space
        self._pspace = self._setup_state_space(cfg)

        # TODO constraints

    def replan(self, start_pillar_state, goal_pillar_state):
        if self._collision_checker is None:
            self._collision_checker = PyBulletCollisionChecker(
                start_pillar_state, {}, self._active_joints, self._cfg)

    def _init_state_space(self, cfg):
        njoints = len(cfg.active_joints)
        state_space = ob.RealVectorStateSpace(njoints)
        # robot = URDF.load(to_absolute_path(cfg.path_to_urdf))

        # set lower and upper bounds
        bounds = ob.RealVectorBounds(njoints)
        # joint_dict = {joint.name: joint for joint in robot.joints}
        # return dict with joint name as key and 
        lower_joints_limits = self._robot.get_joint_lower_limits()
        upper_joints_limits = self._robot.get_joint_upper_limits()
        for i, active_joint in enumerate(cfg.active_joints):
            bounds.setLow(i, lower_joints_limits[active_joint])
            bounds.setHigh(i, upper_joints_limits[active_joint])
        state_space.setBounds(bounds)

        return state_space

