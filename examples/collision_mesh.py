import pybullet as p
import time

import numpy as np

import motion_planning.pybullet_tools.utils as pb_utils
from motion_planning.utils.utils import find_robot_urdf


def load_robot():
    pb_utils.add_data_path()
    franka_urdf = find_robot_urdf("assets/franka_description/robots/franka_panda_dynamics.urdf")
    plane = p.loadURDF("plane.urdf")
    with pb_utils.LockRenderer():
        with pb_utils.HideOutput(True):
            robot = pb_utils.load_pybullet(franka_urdf, fixed_base=True)
    return robot


def load_mesh():
    bunny_vertices = np.load("data/test_bunny_pts.npy")
    mesh = pb_utils.mesh_from_points(bunny_vertices)
    bunny_from_mesh = pb_utils.create_mesh(mesh)
    return bunny_from_mesh


def main():
    pb_utils.connect(use_gui=True)
    obj_mesh = load_mesh()
    pb_utils.set_point(obj_mesh, (0.0, 0.0, 0.1))
    robot = load_robot()
    obstacles = [obj_mesh]
    start_time = time.time()
    collisions = pb_utils.pairwise_collisions(robot, obstacles, link=None)
    end_time = time.time()
    print("Time to detect collisions", end_time - start_time)
    print("Has collisions?", collisions)


if __name__ == "__main__":
    main()
    input("OK?")
