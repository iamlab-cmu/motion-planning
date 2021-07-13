from abc import ABC, abstractmethod


class BaseCollisionChecker(ABC):
    def __init__(self, pillar_state, object_name_to_geometry, cfg):
        self._pillar_state = pillar_state
        self._object_name_to_geometry = object_name_to_geometry
        self._robot_urdf = cfg["robot"]["path_to_urdf"]
        self._cfg = cfg

    def update_pillar_state(self, new_pillar_state):
        raise NotImplementedError

    @abstractmethod
    def pillar_state_in_collision(self, pillar_state):
        raise NotImplementedError
