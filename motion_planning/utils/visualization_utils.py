from motion_planning.envs.pybullet_robot_env import PyBulletRobotEnv
from motion_planning.utils import find_robot_urdf, joint_names_to_link_numbers


def show_plan(plan, start_pillar_state, object_name_to_geometry, active_joints, cfg, block=True):
    robot_urdf_fn = find_robot_urdf(cfg["robot"]["path_to_urdf"])
    display_env = PyBulletRobotEnv(start_pillar_state, object_name_to_geometry, robot_urdf_fn, vis=True)
    active_joint_numbers = joint_names_to_link_numbers(display_env.robot, active_joints)
    print(plan.length())
    for i in range(plan.getStateCount()):
        joint_positions = [plan.getState(i)[state_idx] for state_idx in range(len(active_joints))]
        display_env.set_conf(active_joint_numbers, joint_positions)
        if block:
            input("OK?")
