from ..models.pybullet_robot_env import PyBulletRobotEnv


def show_plan(plan, start_pillar_state, object_name_to_geometry, active_joints, robot_model, block=True):
    display_env = PyBulletRobotEnv(start_pillar_state, object_name_to_geometry, robot_model, vis=True)
    active_joint_numbers = robot_model.joint_names_to_joint_numbers(active_joints)
    print(plan.length())
    for i in range(plan.getStateCount()):
        joint_positions = [plan.getState(i)[state_idx] for state_idx in range(len(active_joints))]
        robot_model.set_conf(active_joint_numbers, joint_positions)
        if block:
            input("OK?")
