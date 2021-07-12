from abc import ABC, abstractmethod
class ObjectGeometry(ABC):
    def __init__(self, cfg):
        self._cfg = cfg

class PointCloud(ObjectGeometry):
    def __init__(self, pts):
        self._pts = pts

    @property
    def points(self):
        return self._pts.copy()

class Box(ObjectGeometry):
    def __init__(self, dims):
        self._dims = dims

    @property
    def dims(self):
        return self._dims.copy()
