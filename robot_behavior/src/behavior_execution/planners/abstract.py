import abc

from enum import Enum


class PlannerStatus(Enum):
    PENDING = 'pending'
    ERROR = 'error'
    INVALID = 'invalid'
    VALID = 'valid'


class Planner(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, end_effectors={"manipulator":"gripper"}):
        self._end_effectors = end_effectors

    @property
    def end_effectors(self):
        return self._end_effectors

    @abc.abstractmethod
    def status(self, ee_group):
        pass

    @abc.abstractmethod
    def stop(self, ee_group):
        pass

    @abc.abstractmethod
    def execute(self, ee_group, plan, wait=False, **kwargs):
        pass

    @abc.abstractmethod
    def plan_ee_pose(self, ee_group, pose, **kwargs):
        pass

    @abc.abstractmethod
    def set_ee_pose(self, ee_group, pose, wait=False, **kwargs):
        pass

    @abc.abstractmethod
    def get_ee_pose(self, ee_group):
        pass

    @abc.abstractmethod
    def validate_ee_pose(self, ee_group, pose, **kwargs):
        pass

    @abc.abstractmethod
    def calculate_ee_tof(self, move_group, pose_start, pose_end):
        pass

    @abc.abstractmethod
    def plan_gripper_state(self, ee_group, joints, **kwargs):
        pass

    @abc.abstractmethod
    def set_gripper_state(self, ee_group, joints, wait=False, **kwargs):
        pass

    @abc.abstractmethod
    def get_gripper_state(self, ee_group):
        pass

    @abc.abstractmethod
    def validate_gripper_state(self, ee_group, state, **kwargs):
        pass

    @abc.abstractmethod
    def calculate_gripper_tof(self, ee_group, joints_start, joints_end):
        pass
