import hydra
from hydra.utils import to_absolute_path
import numpy as np

from urdfpy import URDF

from pillar_state import State

from motion_planning.utils import add_ompl_to_sys_path
from motion_planning.collision_checker import PyBulletCollisionChecker

add_ompl_to_sys_path()
from ompl import base as ob
from ompl import geometric as og


def get_state_space(cfg):
    njoints = len(cfg.active_joints)
    state_space = ob.RealVectorStateSpace(njoints)
    robot = URDF.load(to_absolute_path(cfg.path_to_urdf))

    # set lower and upper bounds
    bounds = ob.RealVectorBounds(njoints)
    joint_dict = {joint.name: joint for joint in robot.joints}
    for i, active_joint in enumerate(cfg.active_joints):
        joint_limits = joint_dict[active_joint].limit
        bounds.setLow(i, joint_limits.lower)
        bounds.setHigh(i, joint_limits.upper)
    state_space.setBounds(bounds)

    return state_space


def get_start_and_goal(space, cfg):
    start = ob.State(space)
    joints = cfg.start_joints
    for i, joint in enumerate(joints):
        start[i] = joint

    # TODO Implement GoalRegion
    goal = ob.State(space)
    if cfg.goal.type == 'jointspace':
        joints = cfg.goal.jointspace.joints
    else:
        raise NotImplementedError
    for i, joint in enumerate(joints):
        goal[i] = joint

    return start, goal


@hydra.main(config_path="../cfg", config_name="run_franka_planner")
def main(cfg):
    space = get_state_space(cfg.robot)

    # create a simple setup object
    pillar_state = State()  # TODO lagrassa make sure synced with state
    pillar_state.update_property(f"frame:{cfg.robot.robot_name}:joint_positions", cfg.task.start_joints)
    active_joints = cfg.robot.active_joints
    collision_checker = PyBulletCollisionChecker(pillar_state, {}, active_joints, cfg)
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(
        ob.StateValidityCheckerFn(lambda ompl_state: not collision_checker.ompl_state_in_collision(ompl_state)))

    start, goal = get_start_and_goal(space, cfg.task)
    ss.setStartAndGoalStates(start, goal)

    solved = ss.solve(1.0)

    if solved:
        ss.simplifySolution()
    return ss.getSolutionPath()


# def state_to_joints(state, num_joints=7):
# return [state[i] for i in range(num_joints)]


# def show_plan(plan):
# connect(use_gui=True)
# add_data_path()
# draw_pose(Pose(), length=1.)
# set_camera_pose(camera_point=[1, -1, 1])

# plane = p.loadURDF("plane.urdf")
# with LockRenderer():
# with HideOutput(True):
# robot = load_pybullet(FRANKA_URDF, fixed_base=True)
# import ipdb; ipdb.set_trace()
# for i in range(plan.getStateCount()):
# state = plan.getState(i)
# conf = state_to_joints(state)
# robot.set_joints(conf)


if __name__ == "__main__":
    main()