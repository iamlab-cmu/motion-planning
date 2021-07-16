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
        self._cfg = cfg

        # robot
        self._robot_model = PyBulletRobotModel(self._robot_cfg.path_to_urdf)
        self._env = PyBulletRobotEnv(self._robot_model, self._cfg.gui)

        # planning space
        self._pspace = self._init_state_space(cfg)
        self._ompl_simple_setup = og.SimpleSetup(self._pspace)
        self._set_collision_checker(collision_checker)

        # TODO constraints

    def replan(self, start_pillar_state, goal_pillar_state, max_planning_time, object_name_to_geometry=None):
        if self._collision_checker is None:
            collision_checker = PyBulletCollisionChecker(
                start_pillar_state, {}, self._active_joints, self._cfg)
            self._set_collision_checker(collision_checker)
        else:
            self._collision_checker.update_state(start_pillar_state)
        self._env.initialize_workspace(start_pillar_state, object_name_to_geometry)
        start_ompl_state = self._pillar_state_to_ompl_state(start_pillar_state)
        goal_ompl_state = self._pillar_state_to_ompl_state(goal_pillar_state)
        self._ompl_simple_setup.setStartAndGoalStates(start_ompl_state,
                                                      goal_ompl_state)
        solved = self._ompl_simple_setup.solve(max_planning_time)
        if solved:
            self._ompl_simple_setup.simplifySolution()
            return self._ompl_simple_setup.getSolutionPath()
        else:
            return None

    def close(self):
        self._collision_checker.close()

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

    def _set_collision_checker(self, collision_checker):
        self._collision_checker = collision_checker
        self._ompl_simple_setup.setStateValidityChecker(ob.StateValidityCheckerFn(
            lambda ompl_state: not collision_checker.ompl_state_in_collision(ompl_state)))

    def _pillar_state_to_ompl_state(self, pillar_state):
        """Extracts robot joints from the pillar state."""
        ompl_state = pillar_state.get_values_as_vec(
            [f"frame:{self._robot_cfg.robot_name}:joint_positions"])
        return ompl_state
