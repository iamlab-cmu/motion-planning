from ..utils.utils import add_pb_tools_if_not_on_path, joint_names_to_link_numbers, get_pb_pose_from_pillar_state, \
    object_geometry_to_pybullet_object

add_pb_tools_if_not_on_path()
import pybullet_tools.utils as pb_utils
import pybullet as p


class PyBulletRobotEnv:
    def __init__(self, pillar_state, object_name_to_geometry, robot_urdf_fn, vis):
        self._object_name_to_object_id = {}
        pb_utils.connect(use_gui=vis)
        pb_utils.add_data_path()
        p.loadURDF("plane.urdf")
        self.initialize_robot(robot_urdf_fn)
        self.initialize_workspace(pillar_state, object_name_to_geometry)

    @property
    def object_name_to_object_id(self):
        return self._object_name_to_object_id.copy()

    @property
    def robot(self):
        return self._robot

    def initialize_robot(self, robot_urdf_fn):
        with pb_utils.LockRenderer():
            with pb_utils.HideOutput(True):
                self._robot = pb_utils.load_pybullet(robot_urdf_fn, fixed_base=True)

    def initialize_workspace(self, pillar_state, object_name_to_geometry):
        for object_name in object_name_to_geometry.keys():
            self._object_name_to_object_id[object_name] = object_geometry_to_pybullet_object(
                object_name_to_geometry[object_name])
            obj_pose = get_pb_pose_from_pillar_state(pillar_state, object_name)
            pb_utils.set_pose(self._object_name_to_object_id[object_name], obj_pose)

    def close(self):
        pb_utils.disconnect()

    def set_conf(self, joints, joint_positions):
        pb_utils.set_joint_positions(self.robot, joints, joint_positions)
