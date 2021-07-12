from .object_geometry import *

def object_geometry_to_pybullet_object(object_geometry):
    if isinstance(object_geometry, PointCloud):
        mesh = pb_utils.mesh_from_points(bunny_vertices)
        obj_from_mesh = pb_utils.create_mesh(mesh)
        return obj_from_mesh
