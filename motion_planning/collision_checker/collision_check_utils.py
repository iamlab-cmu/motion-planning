import pybullet_tools.utils as pb_utils
from .object_geometry import *
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt


def object_geometry_to_pybullet_object(object_geometry):
    if isinstance(object_geometry, PointCloud):
        mesh = pb_utils.mesh_from_points(object_geometry.points)
        obj_from_mesh = pb_utils.create_mesh(mesh)
        return obj_from_mesh
    elif isinstance(object_geometry, Box):
        return pb_utils.create_box(*object_geometry.dims)
    else:
        raise ValueError(f"Invalid object geometry type: {object_geometry} ")


def get_pb_pose_from_pillar_state(pillar_state, obj_name):
    pose_arr = pillar_state.get_values_as_vec([f"frame:{obj_name}:pose/position", f"frame:{obj_name}:pose/quaternion"])
    position = pose_arr[:3]
    wxyz_quaternion = pose_arr[3:]
    xyzw_quaternion = wxyz_quaternion[1:] + [wxyz_quaternion[0]]
    return (position, xyzw_quaternion)


def plot_points(pts):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(pts[:, 0], pts[:, 1], pts[:, 2], 'green')
    ax.set_title('pointcloud')
    plt.show()
