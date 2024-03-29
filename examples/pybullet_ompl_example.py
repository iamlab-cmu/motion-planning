#!/usr/bin/env python
import sys
sys.path.insert(0,'/home/lagrassa/git/pybullet-planning')
#import pybullet_planning as pp
import pybullet as p
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.utils import load_pybullet, LockRenderer, HideOutput, Pose, draw_pose, set_camera_pose, add_data_path, connect
FRANKA_URDF = "/home/lagrassa/git/pybullet-planning/models/franka_description/robots/panda_arm_hand.urdf"
try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og

def isStateValid(state):
    return state[0] > -1000

def run_planner():
    num_joints = 7
    space = ob.RealVectorStateSpace(num_joints)

    # set lower and upper bounds
    bounds = ob.RealVectorBounds(num_joints)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)
    space_info = ob.SpaceInformation(space)
    space_info.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    space_info.setup()


    start = ob.State(space)
    # we can pick a random start state...
    start.random()

    goal = ob.State(space)
    # we can pick a random goal state...
    goal.random()

        # Create a problem instance
    pdef = ob.ProblemDefinition(space_info)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    planner = og.RRTstar(space_info)
    planner.setProblemDefinition(pdef)
    planner.setup()
    solved = planner.solve(3)
    #solved = ss.solve(1.0, planner="rrt")
    assert solved
    return pdef.getSolutionPath()

def state_to_joints(state, num_joints=7):
    return [state[i] for i in range(num_joints)]
def show_plan(plan):
    connect(use_gui=True)
    add_data_path()
    draw_pose(Pose(), length=1.)
    set_camera_pose(camera_point=[1, -1, 1])

    plane = p.loadURDF("plane.urdf")
    with LockRenderer():
        with HideOutput(True):
            robot = load_pybullet(FRANKA_URDF, fixed_base=True)
    for i in range(plan.getStateCount()):
        state = plan.getState(i)
        conf = state_to_joints(state)
        robot.set_joints(conf)


    


if __name__ == "__main__":
    plan = run_planner()
    show_plan(plan)
