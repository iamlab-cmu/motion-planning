import sys
sys.path.insert(0,'/home/lagrassa/git/pybullet-planning')
FRANKA_URDF = "/home/lagrassa/git/pybullet-planning/models/franka_description/robots/panda_arm_hand.urdf"
import pybullet_tools.utils as pb_utils
import numpy as np
import pybullet as p
import time
#import pybullet_utils.utils as pb_utils

def load_robot():
    pb_utils.add_data_path()

    plane = p.loadURDF("plane.urdf")
    with pb_utils.LockRenderer():
        with pb_utils.HideOutput(True):
            robot = pb_utils.load_pybullet(FRANKA_URDF, fixed_base=True)
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

if __name__=="__main__":
    main()
    input("OK?")


