from behavior_execution.planners.abstract import Planner, PlannerStatus
from behavior_execution.utility.interp import interp1d, interp3d, cartesian_distance, interp3d_bound, interp1d_bound
from behavior_execution.utility.marker import SimpleMarkerServer
from geometry_msgs.msg import Pose, Quaternion, Vector3
import math
from authr_tools.elements import Type
import rospy

ORIGINAL_MARKER_SIZE = 0.025
NEW_MARKER_SIZE = 0.4

class FakePlanner(Planner):

    def __init__(self, end_effectors={"human":"Not Applicable"}, gripper_relative_scale=1, gripper_relative_offset=0):
        super(FakePlanner,self).__init__(end_effectors)
        self.pose = Pose(position=Vector3(x=1,y=1,z=1),orientation=Quaternion(x=0,y=0,z=0,w=1))
        self.grip = 1
        self.effort_scale = gripper_relative_scale
        self.effort_offset = gripper_relative_offset

        self.marker_server = SimpleMarkerServer("/authr_sim/simple_markers/")
        self.marker_server.add_marker("fake", Type.HAND, self.pose, name='Human')
        self.marker_server.set_size("fake",NEW_MARKER_SIZE,disableRefresh=True)
        self.marker_server.set_color("fake",200,200,200,disableRefresh=True)
        self.marker_server.refresh()

        self._status = {ee: PlannerStatus.VALID for ee in self.end_effectors.keys()}

    def status(self, ee_group):
        return self._status[ee_group]

    def stop(self, ee_group):
        self.marker_server.set_pose("fake",self.pose,disableRefresh=True)

    def execute(self, ee_group, plan, wait=False, duration=None, **kwargs):
        prevpose = self.pose
        prevgrip = self.grip

        for step in plan:
            if type(step) == Pose:
                self.marker_server.set_pose("fake",step,disableRefresh=True)
                self.marker_server.refresh()
                time_delta = cartesian_distance(prevpose,step)*1 # Since time_scale is 1
                prevpose = self.pose = step
            else:
                time_delta = math.fabs(step-prevgrip)*1 # Since time_scale is 1
                prevgrip = self.grip = step

            if duration != None:
                time_delta = duration/len(plan)

            rospy.sleep(time_delta)
        return True

    def plan_ee_pose(self, ee_group, pose, duration=None, **kwargs):
        if duration == None:
            (traj, duration) = interp3d(self.pose,pose,0.05,1)     #0.0005
        else:
            traj = interp3d_bound(self.pose,pose,duration*10) #2000
        return traj

    def set_ee_pose(self, ee_group, pose, wait=False, duration=None, **kwargs):
        if duration == None:
            (traj, duration) = interp3d(self.pose,pose,0.05,1) #0.0005
        else:
            traj = interp3d_bound(self.pose,pose,duration*10) #2000
        return self.execute("human",traj,wait,duration)

    def get_ee_pose(self, ee_group):
        return self.pose

    def validate_ee_pose(self, ee_group, pose, **kwargs):
        # Always returns true
        return True

    def calculate_ee_tof(self, move_group, pose_start, pose_end):
        (traj, duration) = interp3d(pose_start,pose_end,0.05,1) #0.0005
        return duration, traj

    def plan_gripper_state(self, ee_group, joints, duration=None, **kwargs):

        for i in range(0,len(joints)):
            joints[i] = joints[i] *  self.effort_scale + self.effort_offset

        if duration == None:
            (traj, duration) = interp1d(self.grip,joints[0],0.05,1) #0.0005
        else:
            traj = interp1d_bound(self.grip,joints[0],duration*10) #2000
        return traj

    def set_gripper_state(self, ee_group, joints, wait=False, duration=None, **kwargs):
        if duration == None:
            (traj, duration) = interp1d(self.grip,joints[0],0.05,1) #0.0005
        else:
            traj = interp1d_bound(self.grip,joints[0],duration*10) #2000
        return self.execute("human",traj,wait,duration)

    def get_gripper_state(self, ee_group):
        return [self.grip]

    def validate_gripper_state(self, ee_group, state, **kwargs):
        # Always returns true
        return True

    def calculate_gripper_tof(self, ee_group, joints_start, joints_end):
        (traj, duration) = interp1d(joints_start[0],joints_end[0],0.05,1) #0.0005
        return duration, traj
