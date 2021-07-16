from hydra.utils import to_absolute_path

from motion_planning.collision_checker import PyBulletCollisionChecker
from motion_planning.models.pybullet_robot_model import PyBulletRobotModel
from motion_planning.models.pybullet_robot_env import PyBulletRobotEnv
from motion_planning.utils import (add_ompl_to_sys_path)
add_ompl_to_sys_path()
from ompl import base as ob
from ompl import geometric as og


class IAMMotionPlanner():
    def __init__(self, cfg, collision_checker=None):
        self._robot_cfg = cfg.robot
        self._active_joints = self._robot_cfg.active_joints
        self._ndims = len(self._active_joints)
        # self._planner = cfg.planner
        self._collision_checker = collision_checker
        self._cfg = cfg

        # robot
        self._robot_model = PyBulletRobotModel(self._robot_cfg.path_to_urdf)
        self._env = None

        # planning space
        self._pspace = self._init_state_space(cfg)

        # TODO constraints

    def replan(self, start_pillar_state, goal_pillar_state):
        if self._collision_checker is None:
            self._collision_checker = PyBulletCollisionChecker(
                start_pillar_state, {}, self._active_joints, self._cfg)
        if self._env is None:
            self._env = PyBulletRobotEnv(start_pillar_state, {}, self._robot_model,
                                         self._cfg.gui)

        # if solved:
            # ss.simplifySolution()
            # return ss.getSolutionPath()
        # else:
            # return None

    def _init_state_space(self, cfg):
        state_space = ob.RealVectorStateSpace(self._ndims)

        # set lower and upper bounds
        bounds = ob.RealVectorBounds(self._ndims)
        for i, active_joint in enumerate(self._active_joints):
            lower, upper = self._robot_model.get_joint_limits(active_joint)
            bounds.setLow(i, lower)
            bounds.setHigh(i, upper)
        state_space.setBounds(bounds)

        return state_space
