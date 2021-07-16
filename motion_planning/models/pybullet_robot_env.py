from ..utils.utils import add_pb_tools_if_not_on_path, get_pb_pose_from_pillar_state, \
    object_geometry_to_pybullet_object

add_pb_tools_if_not_on_path()
import pybullet_tools.utils as pb_utils
import pybullet as p


class PyBulletRobotEnv:
    def __init__(self, robot_model, vis):
        self._object_name_to_object_id = {}
        pb_utils.connect(use_gui=vis)
        pb_utils.add_data_path()
        p.loadURDF("plane.urdf")
        self.initialize_robot(robot_model)

    @property
    def object_name_to_object_id(self):
        return self._object_name_to_object_id.copy()

    def initialize_robot(self, robot_model):
        robot_urdf_fn = robot_model.robot_urdf_fn
        with pb_utils.LockRenderer():
            with pb_utils.HideOutput(True):
                robot_idx = pb_utils.load_pybullet(robot_urdf_fn, fixed_base=True)
                robot_model.set_pybullet_obj_index(robot_idx)

    def initialize_workspace(self, pillar_state, object_name_to_geometry):
        for object_name in object_name_to_geometry.keys():
            self._object_name_to_object_id[object_name] = object_geometry_to_pybullet_object(
                object_name_to_geometry[object_name])
            obj_pose = get_pb_pose_from_pillar_state(pillar_state, object_name)
            pb_utils.set_pose(self._object_name_to_object_id[object_name], obj_pose)

    def close(self):
        pb_utils.disconnect()
