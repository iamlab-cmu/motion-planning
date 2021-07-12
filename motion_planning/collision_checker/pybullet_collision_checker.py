from abc import ABC, abstractmethod

import sys
sys.path.insert(0, '/home/lagrassa/git/pybullet-planning')

import pybullet_tools.utils as pb_utils
import pybullet as p
from .collision_utils import object_geometry_to_pybullet_object


class CollisionChecker(ABC):
    def __init__(self, pillar_state, object_geometries, robot_urdf, cfg):
        super().__init__(pillar_state, object_geometries, robot_urdf, cfg)
        self._cfg = cfg
        self._setup_env(vis=cfg["gui"])
        self._object_name_to_object_id = {}

    def _is_joint_conf_in_collision(self, conf, disabled_collisions=()):
        """

        :param conf:
        :return: whether the robot with configuration conf will collide
        with obstacles in the scene (besides those in disabled_collisions)
        """
        raise NotImplementedError

    def _setup_env(self, vis=False):
        # load robot TODO lagrassa load any robot from urdf
        pb_utils.add_data_path()

        plane = p.loadURDF("plane.urdf")
        with pb_utils.LockRenderer():
            with pb_utils.HideOutput(True):
                self.robot = pb_utils.load_pybullet(self._robot_urdf, fixed_base=True)
        for object_name in self._object_geometries.keys():
            self._object_name_to_object_id[object_name] = object_geometry_to_pybullet_object(
                self._object_geometries[object_name])
        input("Workspace OK?")
