from abc import ABC, abstractmethod
import pybullet_tools.utils as pb_utils

class CollisionChecker(ABC):
    def __init__(self, pillar_state, object_geometries, robot_urdf, cfg):
        self._pillar_state = pillar_state
        self._object_geometries = object_geometries
        self._robot_urdf = robot_urdf
        self._cfg = cfg




