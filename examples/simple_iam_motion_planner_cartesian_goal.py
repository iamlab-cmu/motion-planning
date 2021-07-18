import hydra
import numpy as np
from pillar_state import State

from motion_planning.iam_motion_planner import IAMMotionPlanner
from motion_planning.planning.goals import CartesianGoal
from motion_planning.utils import add_ompl_to_sys_path

add_ompl_to_sys_path()


@hydra.main(config_path="../cfg", config_name="run_franka_planner")
def main(cfg):
    planner = IAMMotionPlanner(cfg)
    start = make_simple_pillar_state(cfg.robot.robot_name,
                                     cfg.task.start_joints)[0]
    position = [0.5, 0.2, 0.3]
    quat_wxyz = [0, 1, 0, 0]
    goal_arr_pos_wxyz = position + quat_wxyz
    goal = CartesianGoal(goal_arr_pos_wxyz)
    solution_path = planner.replan(start, goal, 5, object_name_to_geometry={})

    if solution_path is not None:
        last_joints = planner.visualize_plan(solution_path)
    # Check
    end_pose = planner._robot_model.forward_kinematics(planner._collision_checker._active_joint_numbers,
                                                       last_joints,
                                                       planner._robot_model.grasp_link_index)
    assert np.allclose(end_pose[0], goal.goal_data[0], atol=2e-3)
    assert np.allclose(end_pose[1], goal.goal_data[1], atol=2e-3 * np.pi)  # TODO lagrassa quaternion distance
    planner.close()


def make_simple_pillar_state(robot_name, start_conf):
    pillar_state = State()
    pillar_state.update_property(f"frame:{robot_name}:joint_positions", start_conf)
    return pillar_state, {}


if __name__ == "__main__":
    main()
