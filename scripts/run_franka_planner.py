import hydra
from pillar_state import State

from motion_planning.iam_motion_planner import IAMMotionPlanner
from motion_planning.models.object_geometry import Box
from motion_planning.utils import add_ompl_to_sys_path

add_ompl_to_sys_path()
from ompl import base as ob


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
    planner = IAMMotionPlanner(cfg)
    start = make_simple_pillar_state(cfg.robot.robot_name,
                                          cfg.task.start_joints)[0]
    goal = make_simple_pillar_state(cfg.robot.robot_name,
                                         cfg.task.goal.jointspace.joints)[0]
    solution_path = planner.replan(start, goal, 5)
    if solution_path is not None:
        planner.visualize_plan(solution_path)
    planner.close()


def make_simple_pillar_state(robot_name, start_conf):
    pillar_state = State()
    pillar_state.update_property(f"frame:{robot_name}:joint_positions", start_conf)
    return pillar_state, {}


def make_constrained_pillar_state(robot_name, start_conf):
    pillar_state = State()
    pillar_state.update_property(f"frame:{robot_name}:joint_positions", start_conf)
    short_side_len = 0.05
    height = 0.3
    box_xy_locs = [[0.4, 0], [0.4, 0.1]]
    box_z = height / 2 + 0.001
    dims = [short_side_len, short_side_len, height]
    object_name_to_geometry = {f"box{i}": Box(dims) for i, _ in enumerate(box_xy_locs)}
    for i, loc in enumerate(box_xy_locs):
        pillar_state.update_property(f"frame:box{i}:pose/position", loc + [box_z, ])
        pillar_state.update_property(f"frame:box{i}:pose/quaternion", [1, 0, 0, 0])

    return pillar_state, object_name_to_geometry


if __name__ == "__main__":
    main()
