from motion_planning.collision_checker import PyBulletCollisionChecker
from motion_planning.models.pybullet_robot_env import PyBulletRobotEnv
from motion_planning.models.pybullet_robot_model import PyBulletRobotModel
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

        # robot
        self._robot_model = PyBulletRobotModel(self._robot_cfg.path_to_urdf)
        self._env = PyBulletRobotEnv(self._robot_model, cfg.gui)

        # planning space
        self._pspace = self._init_state_space(cfg)
        self._ompl_simple_setup = og.SimpleSetup(self._pspace)
        self._set_collision_checker(collision_checker)

        # planner
        self._motion_planner_type = cfg.planner_type
        si = self._ompl_simple_setup.getSpaceInformation()
        self._motion_planner = self._allocate_planner(si, cfg.planner_type)
        self._ompl_simple_setup.setPlanner(self._motion_planner)

        # TODO constraints
        self._cfg = cfg

    def replan(self, start_pillar_state, goal_pillar_state, max_planning_time, object_name_to_geometry={}):
        if self._collision_checker is None:
            collision_checker = PyBulletCollisionChecker(
                self._env, start_pillar_state, {}, self._active_joints,
                self._cfg, robot_model=self._robot_model)
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

    def visualize_plan(self, solution_path, block=True):
        active_joint_numbers = self._robot_model.joint_names_to_joint_numbers(self._active_joints)
        for i in range(solution_path.getStateCount()):
            joint_positions = [solution_path.getState(i)[state_idx] for state_idx in range(len(self._active_joints))]
            self._robot_model.set_conf(active_joint_numbers, joint_positions)
            if block:
                input("OK?")

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

    def _allocate_planner(self, si, planner_type):
        if planner_type.lower() == "rrt":
            return og.RRT(si)
        elif planner_type.lower() == "rrtstar":
            return og.RRTstar(si)
        elif planner_type.lower() == "bfmtstar":
            return og.BFMT(si)
        elif planner_type.lower() == "bitstar":
            return og.BITstar(si)
        elif planner_type.lower() == "fmtstar":
            return og.FMT(si)
        elif planner_type.lower() == "informedrrtstar":
            return og.InformedRRTstar(si)
        elif planner_type.lower() == "prmstar":
            return og.PRMstar(si)
        else:
            raise NotImplementedError("Planner-type is not implemented in allocation function.")

    def _pillar_state_to_ompl_state(self, pillar_state):
        """Extracts robot joints from the pillar state."""
        state = pillar_state.get_values_as_vec(
            [f"frame:{self._robot_cfg.robot_name}:joint_positions"])
        ompl_state = ob.State(self._pspace)
        for i, joint in enumerate(state):
            ompl_state[i] = joint
        return ompl_state
