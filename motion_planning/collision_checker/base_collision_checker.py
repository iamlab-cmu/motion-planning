from abc import ABC, abstractmethod
from ..utils import find_robot_urdf


class BaseCollisionChecker(ABC):
    def __init__(self, pillar_state, object_name_to_geometry, active_joints, cfg, robot_model):
        self._pillar_state = pillar_state
        self._object_name_to_geometry = object_name_to_geometry
        self._robot_model = robot_model
        self._active_joints = active_joints
        self._cfg = cfg

    def update_pillar_state(self, new_pillar_state):
        raise NotImplementedError

    @abstractmethod
    def pillar_state_in_collision(self):
        raise NotImplementedError

    @abstractmethod
    def ompl_state_in_collision(self, ompl_state):
        raise NotImplementedError
