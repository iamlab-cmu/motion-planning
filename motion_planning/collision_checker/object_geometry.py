from abc import ABC, abstractmethod
class ObjectGeometry(ABC):
    def __init__(self, cfg):
        self._cfg = cfg

class PointCloud(ObjectGeometry):
    def __init__(self, pts):
        self._pts = pts
