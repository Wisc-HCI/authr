import sys
import rospy
import moveit_commander
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, MoveGroupActionGoal, MoveGroupActionResult
from behavior_execution.planners.abstract import Planner, PlannerStatus
from actionlib_msgs.msg import GoalStatus

class MoveItPlanner(Planner):

    def __init__(self, end_effectors={"manipulator":"gripper"}, gripper_relative_scale=1, gripper_relative_offset=0):
        super(MoveItPlanner,self).__init__(end_effectors)

        self.effort_scale = gripper_relative_scale
        self.effort_offset = gripper_relative_offset

        # Initialize MoveIt and ROS Node
        moveit_commander.roscpp_initialize(sys.argv)

        # Create a Robot Commander Attribute
        self.robot_commander = moveit_commander.RobotCommander()

        # Create a Scene Attribute
        self.scene_interface = moveit_commander.PlanningSceneInterface()

        # Create Move Group Commanders
        self.move_group_commanders = {}
        self.move_group_commanders.update({move_group: moveit_commander.MoveGroupCommander(move_group) for move_group in self._end_effectors.keys()})
        self.move_group_commanders.update({move_group: moveit_commander.MoveGroupCommander(move_group) for move_group in self._end_effectors.values() if move_group != None})

        # move_group subscriptions for traking usage
        self._status_goal = rospy.Subscriber("move_group/goal", MoveGroupActionGoal, self._status_goal)
        self._status_result = rospy.Subscriber("move_group/result", MoveGroupActionResult, self._status_result)

        # Status state setup
        self._status = {ee: PlannerStatus.VALID for ee in self.end_effectors}
        self._pending_list = [] # {'ee_group': str, 'type': str}
        self._active_requests = {} # {'id': str, 'ee_group': str, 'type': str}

    def status(self, ee_group):
        # states are 'valid', 'pending','invalid','error'
        return self._status[ee_group]

    def _status_append_pending(self, ee_group, type):
        self._pending_list.append({'ee_group':ee_group,'type':type})

    def _status_goal(self, msg):
        id = msg.goal_id.id

        print '\n\n In Status Goal',

        if len(self._pending_list) > 0:
            item = self._pending_list.pop(0)
            item['id'] = id
            self._status[item['ee_group']] = PlannerStatus.PENDING
            self._active_requests[id] = item

    def _status_result(self, msg):
        id = msg.status.goal_id.id
        status = msg.status.status

        print '\n\n In Status Result', status

        item = self._active_requests.pop(id, None)
        if item != None:

            # determine final state of the request
            state = PlannerStatus.ERROR
            if status == GoalStatus.PENDING:
                state = PlannerStatus.ERROR #this is nota result state
            elif status == GoalStatus.ACTIVE:
                state = PlannerStatus.ERROR #this is not a result state
            elif status == GoalStatus.PREEMPTED:
                state = PlannerStatus.INVALID
            elif status == GoalStatus.SUCCEEDED:
                state = PlannerStatus.VALID
            elif status == GoalStatus.ABORTED:
                state = PlannerStatus.INVALID
            elif status == GoalStatus.REJECTED:
                state = PlannerStatus.INVALID
            elif status == GoalStatus.PREEMPTING:
                state = PlannerStatus.ERROR #this is not a result state
            elif status == GoalStatus.RECALLING:
                state = PlannerStatus.ERROR #this is not a result state
            elif status == GoalStatus.RECALLED:
                state = PlannerStatus.INVALID
            elif status == GoalStatus.LOST:
                state = PlannerStatus.ERROR #should not be sent by server

            # update global ee status and trigger callback in available
            self._status[item['ee_group']] = state
            print state

    def _check_for_empty_plan(self, plan):
        try:
            if (len(plan.joint_trajectory.joint_names) == 0 and len(plan.joint_trajectory.points) == 0
               and len(plan.multi_dof_joint_trajectory.joint_names) == 0 and len(plan.multi_dof_joint_trajectory.points) == 0):
                return True
            else:
                return False
        except:
            return False

    def stop(self, ee_group):
        self._status_append_pending(ee_group,'stop')
        self.move_group_commanders[ee_group].clear_pose_targets()
        self.move_group_commanders[ee_group].stop()

    def execute(self, ee_group, plan, wait=True, **kwargs):
        self.stop(ee_group)
        self._status_append_pending(ee_group,'execute')
        self.move_group_commanders[ee_group].clear_pose_targets()

        if not 'joint_target' in kwargs.keys() and not kwargs['joint_target']:
            status = self.move_group_commanders[ee_group].execute(plan, wait=wait)
        else:
            index = len(plan.joint_trajectory.points) - 1
            if index < 0:
                status = False # No end state
            joints = plan.joint_trajectory.points[index].positions
            status = self.move_group_commanders[ee_group].go(joints, wait=True)
        print '\n\nStatus = ', status
        return status

    def plan_ee_pose(self, ee_group, pose, **kwargs):
        self._status_append_pending(ee_group,'plan_pose')
        self.move_group_commanders[ee_group].clear_pose_targets()
        self.move_group_commanders[ee_group].set_pose_target(pose)
        return self.move_group_commanders[ee_group].plan()

    def set_ee_pose(self, ee_group, pose, wait=False, **kwargs):
        self._status_append_pending(ee_group,'set_pose')
        self.move_group_commanders[ee_group].clear_pose_targets()
        self.move_group_commanders[ee_group].set_pose_target(pose)
        status = self.move_group_commanders[ee_group].go(wait=wait)
        return status

    def get_ee_pose(self, ee_group):
        poseStamped = self.move_group_commanders[ee_group].get_current_pose()
        return poseStamped.pose

    def validate_ee_pose(self, ee_group, pose, **kwargs):
        plan = self.plan_ee_pose(ee_group,pose,**kwargs)
        return not self._check_for_empty_plan(plan)

    def calculate_ee_tof(self, ee_group, pose_start, pose_end):

        # get initial joint-state
        self._status_append_pending(ee_group,'tof_init')
        self.move_group_commanders[ee_group].clear_pose_targets()
        self.move_group_commanders[ee_group].set_pose_target(pose_start)
        plan_to_start = self.move_group_commanders[ee_group].plan()

        initial_joint_state = JointState()
        initial_joint_state.header = Header()
        initial_joint_state.header.stamp = rospy.Time.now()
        initial_joint_state.name = plan_to_start.joint_trajectory.joint_names
        index = len(plan_to_start.joint_trajectory.points) - 1
        if index < 0:
            return None, None # No time could be computed invalid start state
        initial_joint_state.position = plan_to_start.joint_trajectory.points[index].positions
        initial_robot_state = RobotState()
        initial_robot_state.joint_state = initial_joint_state

        self.move_group_commanders[ee_group].set_start_state(initial_robot_state)

        # compute plan to pose_end
        self._status_append_pending(ee_group,'tof_calc')
        self.move_group_commanders[ee_group].clear_pose_targets()
        self.move_group_commanders[ee_group].set_pose_target(pose_end)
        plan_to_end = self.move_group_commanders[ee_group].plan()

        # get tof
        index = len(plan_to_end.joint_trajectory.points) - 1
        if index < 0:
            return None, None # No time could be computed invalid end state
        end_state = plan_to_end.joint_trajectory.points[index]
        duration = end_state.time_from_start.secs + end_state.time_from_start.nsecs / 1000000000.0
        return duration, plan_to_end

    def plan_gripper_state(self, ee_group, joints, **kwargs):
        self._status_append_pending(ee_group,'plan_gripper')
        self.move_group_commanders[self._end_effectors[ee_group]].clear_pose_targets()
        return self.move_group_commanders[self._end_effectors[ee_group]].plan(joints)

    def set_gripper_state(self, ee_group, joints, wait=False, **kwargs):

        for i in range(0,len(joints)):
            print '\n\n\n Effort scale params', self.effort_scale, self.effort_offset, '\n\n\n'
            joints[i] = joints[i] * self.effort_scale + self.effort_offset

        self._status_append_pending(ee_group,'set_gripper')
        self.move_group_commanders[self._end_effectors[ee_group]].clear_pose_targets()
        return self.move_group_commanders[self._end_effectors[ee_group]].go(joints, wait=wait)

    def get_gripper_state(self, ee_group):
        return self.move_group_commanders[self._end_effectors[ee_group]].get_current_joint_values()

    def validate_gripper_state(self, ee_group, state, **kwargs):
        plan = self.plan_gripper_state(ee_group,state,**kwargs)
        return self._check_for_empty_plan(plan)

    def calculate_gripper_tof(self, ee_group, joints_start, joints_end):
        raise NotImplementedError('Method not implemented in current version of planner')
