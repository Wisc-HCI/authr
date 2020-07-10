#! /usr/bin/env python

import tf
import copy
import json
import math
import time
import uuid
import rospy
import random
import actionlib
import threading
import traceback
import numpy as np
import authr_tools.msg_conversion as msg_conv

from enum import Enum
from behavior_execution.planners.fake import FakePlanner
from behavior_execution.planners.moveit import MoveItPlanner
from authr_tools.agent_thread import AgentThread, ThreadState, PrimitiveType

from std_msgs.msg import Bool
from authr.msg import MarkerPose
from authr.srv import GetRequest
from authr.msg import EulerPose, JobResult
from authr.srv import CreateAgent, CreateAgentResponse
from authr.srv import DeleteAgent, DeleteAgentResponse
from authr.srv import UpdateAgent, UpdateAgentResponse
from geometry_msgs.msg import Pose, Quaternion, Vector3
from authr.srv import GetEE, GetEERequest, GetEEResponse
from authr.srv import DeleteRequest, DeleteRequestResponse
from authr.msg import JobSubmissionRequest, JobSubmissionResponse
from authr.srv import SetForceControlQuery, SetForceControlQueryResponse
from authr.msg import SimulateAction, SimulateGoal, SimulateFeedback, SimulateResult


NODE_SETUP_DELAY_TIME = 10
SIMULATION_TOTAL_TIME_GRACE_PERIOD = 10
SIMULATION_TIMESTEP = 0.05

HUMAN_AGENT_EE_OFFSET = Vector3(0,0,-0.06)
ROBOT_AGENT_EE_OFFSET = Vector3(0,0,-0.145)


class State(Enum):
    IDLE = 'idle'
    SIMULATE = 'simulate'
    TOF = 'tof'
    VALIDATE = 'validate'
    SET_EE = 'set_ee'


class AuthrSim(object):

    def __init__(self):

        # node state
        self.force_control_enabled = False
        self._lock = threading.Lock()
        self._state = State.IDLE
        self._agents = {}

        # job queues
        self._set_queue = []
        self._validate_queue = []
        self._tof_queue = []

        # Joint Trajectory Cache
        self._joint_trajectory_cache = {}

        # Planner Interface
        self._moveit_planner = MoveItPlanner(
            gripper_relative_scale=rospy.get_param('~robot_effort_scalar',1),
            gripper_relative_offset=rospy.get_param('~robot_effort_offset',0))
        self._fake_planner = FakePlanner(
            gripper_relative_scale=rospy.get_param('~human_effort_scalar',1),
            gripper_relative_offset=rospy.get_param('~human_effort_offset',0))

        ## Agent services
        self.create_agent_srv = rospy.Service("authr_sim/create_agent", CreateAgent, self.create_agent)
        self.delete_agent_srv = rospy.Service("authr_sim/delete_agent", DeleteAgent, self.delete_agent)
        self.update_agent_srv = rospy.Service("authr_sim/update_agent", UpdateAgent, self.update_agent)

        ## End effector service
        self.get_ee_srv = rospy.Service("authr_sim/get_ee", GetEE, self.get_ee)

        ## Force control service
        self.set_force_control_srv = rospy.Service("authr_sim/set_force_control", SetForceControlQuery, self.set_force_control)

        ## Action Servers
        self.simulate_server = actionlib.SimpleActionServer("authr_sim/simulate", SimulateAction, self.simulate, False)
        self.simulate_server.start()

        # Job processor
        self._job_submission_subscriber = rospy.Subscriber("authr_sim/job_submission/request", JobSubmissionRequest, self.job_request)
        self._job_submission_publisher = rospy.Publisher("authr_sim/job_submission/response", JobSubmissionResponse, queue_size=10)
        self._job_result_publisher = rospy.Publisher("authr_sim/job_result", JobResult, queue_size=10)
        self._job_processor_thread_kill = False
        self._job_processor_thread = threading.Thread(target=self._job_processor)
        self._job_processor_thread.start()

        # Joint Trajectory Cache
        self.delete_joint_trajectory_srv = rospy.Service("authr_sim/delete_joint_trajectory", DeleteRequest, self.delete_joint_trajectory)

        # Proxies for authr_plan assets
        self.get_element = rospy.ServiceProxy("authr_plan/get_element",GetRequest)
        self.set_thing_pose = rospy.Publisher("visualization_marker/set_pose", MarkerPose, queue_size=25)

    def _block_active_agent_threads(self, timeout=10):
        block = True
        start_t = time.time()

        while block and start_t + timeout > time.time():
            block = False

            for agent_key in self._agents.keys():
                if self._agents[agent_key]['thread'].state == 'active':
                    block = True
                    break
            rospy.sleep(0.1)

        # force stop threads
        for agent_key in self._agents.keys():
            self._agents[agent_key]['thread'].deactivate()

    def stop_threads(self):
        for agent in self._agents:
            self._agents[agent]['thread'].stop()
        self._job_processor_thread_kill = True

    def _agent_ee_position_offset(self,agent,pose,dir=1):
        Rm = tf.transformations.quaternion_matrix([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        Tp = tf.transformations.translation_matrix([pose.position.x,pose.position.y,pose.position.z,1])
        Ov = np.matrix([self._agents[agent]["offset"].x,self._agents[agent]["offset"].y,self._agents[agent]["offset"].z,1 / dir]).T * dir

        Nv = (Tp * (Rm * Ov)).A1
        Nv = Nv / Nv[3]

        return Pose(position=Vector3(x=Nv[0],y=Nv[1],z=Nv[2]),
                    orientation=pose.orientation)

    #===========================================================================
    #   Joint Trajectory Cache
    #===========================================================================

    def delete_joint_trajectory(self, request):
        if not request.id in self._joint_trajectory_cache.keys() or self._joint_trajectory_cache[request.id] == None:
            return DeleteRequestResponse(False,'Trajectory with ID does not exist in cache')
        else:
            msg = ''
            if request.type != 'joint_trajectory':
                msg = 'Warning: invalid type assuming joint trajectory'

            self._joint_trajectory_cache.pop(request.id)
            return DeleteRequestResponse(True,msg)

    #===========================================================================
    #   Agent Thread Services
    #===========================================================================

    def create_agent(self, message):
        valid = (not message.agent in self._agents.keys()) or (self._agents[message.agent] == None)
        if valid:

            # select applicable planner
            planner = None
            offset =  Vector3()
            if message.endeffector in self._moveit_planner.end_effectors:
                planner = self._moveit_planner
                offset = ROBOT_AGENT_EE_OFFSET
            elif message.endeffector in self._fake_planner.end_effectors:
                planner = self._fake_planner
                offset = HUMAN_AGENT_EE_OFFSET

            # create agent object
            self._agents[message.agent] = {
                'ee_group': message.endeffector,
                'thread': AgentThread(planner, message.endeffector),
                'planner': planner,
                'offset': offset
            }
            self._agents[message.agent]['thread'].start()

        return CreateAgentResponse(valid)

    def delete_agent(self, message):
        valid = (message.agent in self._agents.keys()) and (self._agents[message.agent] != None)

        if valid:

            # stop thread execution
            self._agents[message.agent]['thread'].stop()

            # remove agent entry
            del self._agents[message.agent]

        return DeleteAgentResponse(valid)

    def update_agent(self, message):
        valid = message.agent in self._agents.keys() and self._agents[message.agent] != None

        if valid:
            self._agents[message.agent]['ee_group'] = message.endeffector

            # select applicable planner
            planner = None
            offset =  Vector3()
            if message.endeffector in self._moveit_planner.end_effectors:
                planner = self._moveit_planner
                offset = ROBOT_AGENT_EE_OFFSET
            elif message.endeffector in self._fake_planner.end_effectors:
                planner = self._fake_planner
                offset = HUMAN_AGENT_EE_OFFSET

            self._agents[message.agent]['planner'] = planner
            self._agents[message.agent]['thread'].update_planner(planner)
            self._agents[message.agent]['thread'].update_ee_group(message.endeffector)
            self._agents[message.agent]['offset'] = offset

        return UpdateAgentResponse(valid)

    #===========================================================================
    #   End Effector Services
    #===========================================================================

    def set_force_control(self, message):
        # TODO: Do something to set the force control of the robot
        new_setting = not message.enabled
        return SetForceControlQueryResponse(new_setting)

    def get_ee(self, message):
        pose = EulerPose()
        effort = 0

        if (not message.agent in self._agents.keys()) or (self._agents[message.agent] == None):
            return GetEEResponse(pose,effort,GetEEResponse.BAD_AGENT_KEY)

        if message.motion_type == GetEERequest.TYPE_ARM:
            if self._agents[message.agent]['planner'] != None:
                pose = msg_conv.pose_eulerMsgFromQuaternionMsg(self._agents[message.agent]['planner'].get_ee_pose(self._agents[message.agent]['ee_group']))
                pose = self._agent_ee_position_offset(message.agent,pose,-1)
        elif message.motion_type == GetEERequest.TYPE_GRIPPER:
            if self._agents[message.agent]['planner'] != None:
                effort = self._agents[message.agent]['planner'].get_gripper_state(self._agents[message.agent]['ee_group'])

        return GetEEResponse(pose,effort,GetEEResponse.VALID)

    #===========================================================================
    #   Job processors
    #===========================================================================

    def _job_processor(self):
        job = None

        # thread used to execute tasks in queues
        while not self._job_processor_thread_kill:
            rospy.sleep(0.01)

            # check if idle and items in queues
            earlyContinue = False
            self._lock.acquire()
            if self._state != State.IDLE:
                earlyContinue = True
            elif len(self._set_queue) != 0:
                self._state = State.SET_EE
                job = self._set_queue.pop(0)
            elif len(self._validate_queue) != 0:
                self._state = State.VALIDATE
                job = self._validate_queue.pop(0)
            elif len(self._tof_queue) != 0:
                self._state = State.TOF
                job = self._tof_queue.pop(0)
            else: #empty
                earlyContinue = True
            self._lock.release()

            if earlyContinue:
                continue

            # process job
            result = JobResult()
            result.id = job['id']
            result.error = False
            result.msg = ''
            result.agent = job['agent']
            result.type = job['type']

            res = {}
            if job['type'] == 'tof_pose' or job['type'] == 'tof_gripper':
                res['tof'] = -2
            elif job['type'] == 'validate_pose' or job['type'] == 'validate_gripper':
                res['valid'] = False
            elif job['type'] == 'set_pose' or job['type'] == 'set_gripper':
                res['success'] = False

            if (job['agent'] in self._agents.keys()) and (self._agents[job['agent']] != None):
                if job['type'] == 'tof_pose' or job['type'] == 'tof_gripper':
                    res['tof'] = self._get_tof(job['agent'],job['type'],job['params'], job['id'] if job['type'] == 'tof_pose' else None)
                elif job['type'] == 'validate_pose' or job['type'] == 'validate_gripper':
                    res['valid'] = self._validate_ee_motion(job['agent'],job['type'],job['params'])
                elif job['type'] == 'set_pose' or job['type'] == 'set_gripper':
                    res['success'] = self._set_ee(job['agent'],job['type'],job['params'])
                else:
                    result.error = True
                    result.msg = 'Invalid type'
            else:
                result.error = True
                result.msg = 'Agent does not exist'
            result.result = json.dumps(res)

            self._job_result_publisher.publish(result)

            # release state
            self._lock.acquire()
            self._state = State.IDLE
            self._lock.release()

    def _validate_ee_motion(self, agent, type, params):

        # determine validation type then execute
        success = False
        if type == 'validate_pose':
            if self._agents[agent]['planner'] != None:
                pose = msg_conv.pose_quaternionMsgFromEulerDict(params['pose'])
                pose = self._agent_ee_position_offset(agent,pose,+1)
                success = self._agents[agent]['planner'].validate_ee_pose(self._agents[agent]['ee_group'],pose)
        elif type == 'validate_gripper':
            if self._agents[agent]['planner'] != None:
                success = self._agents[agent]['planner'].validate_gripper_state(self._agents[agent]['ee_group'],params['effort'])

        return success

    def _get_tof(self, agent, type, params, id=None):

        retries = 0
        if 'retries' in params.keys():
            retries = params['retries']

        # determine tof type, run calculation
        tof = None
        while tof == None and retries >= 0:
            if type == 'tof_pose':
                if self._agents[agent]['planner'] != None:
                    pose_start = msg_conv.pose_quaternionMsgFromEulerDict(params['pose_start'])
                    pose_start = self._agent_ee_position_offset(agent,pose_start,+1)
                    pose_end = msg_conv.pose_quaternionMsgFromEulerDict(params['pose_end'])
                    pose_end = self._agent_ee_position_offset(agent,pose_end,+1)
                    tof, plan = self._agents[agent]['planner'].calculate_ee_tof(self._agents[agent]['ee_group'],pose_start,pose_end)
                    if id and plan:
                        self._joint_trajectory_cache[id] = plan
            elif type == 'tof_gripper':
                if self._agents[agent]['planner'] != None:
                    effort_start = params['effort_start']
                    effort_end = params['effort_end']
                    tof, plan = self._agents[agent]['planner'].calculate_gripper_tof(self._agents[agent]['ee_group'],effort_start,effort_end)
                    if id and plan:
                        self._joint_trajectory_cache[id] = plan
            retries -= 1

        # Return time-of-flight
        if tof is None:
            tof = -1
        return tof

    def _set_ee(self, agent, type, params):
        blocking = not params['nonblocking']

        # command agent to perform task
        success = False
        if type == 'set_pose':
            if self._agents[agent]['planner'] != None:
                pose = msg_conv.pose_quaternionMsgFromEulerDict(params['pose'])
                pose = self._agent_ee_position_offset(agent,pose,+1)
                success = self._agents[agent]['planner'].set_ee_pose(self._agents[agent]['ee_group'],pose,wait=blocking)
        elif type == 'set_gripper':
            if self._agents[agent]['planner'] != None:
                success = self._agents[agent]['planner'].set_gripper_state(self._agents[agent]['ee_group'],params['effort'],wait=blocking)

        # spin while running
        if blocking:
            while self._agents[agent]['thread'].state == 'active':
                rospy.sleep(0.01)

        # determine final movement status
        return not blocking or success

    #===========================================================================
    #   Job Request
    #===========================================================================

    def job_request(self, request):

        # validate agent
        if (not request.agent in self._agents.keys()) or (self._agents[request.agent] == None):
            self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'invalid agent key'))

        # validate type and parameters
        params = json.loads(request.params)
        if request.type == 'tof_pose':
            if not ('pose_start' in params.keys() and 'pose_end' in params.keys()):
                self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'invalid parameters'))
        elif request.type == 'tof_gripper':
            if not ('effort_start' in params.keys() and 'effort_end' in params.keys()):
                self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'invalid parameters'))
        elif request.type == 'validate_pose':
            if not 'pose' in params.keys():
                self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'invalid parameters'))
        elif request.type == 'validate_gripper':
            if not 'effort' in params.keys():
                self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'invalid parameters'))
        elif request.type == 'set_pose':
            if not ('pose' in params.keys() and 'nonblocking' in params.keys()):
                self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'invalid parameters'))
        elif request.type == 'set_gripper':
            if not ('effort' in params.keys() and 'nonblocking' in params.keys()):
                self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'invalid parameters'))
        elif request.type != 'cancel':
            self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'invalid type'))

        # check if id exists in pending jobs
        #   if in pending then update
        #   if not in pending
        submitNew = False
        self._lock.acquire()
        if request.id != '':
            if request.id in [j['id'] for j in self._set_queue]:
                if request.type == 'set_pose' or request.type == 'set_gripper':
                    index = self._get_existing_job_index(self._set_queue,request.id)
                    self._set_queue[index]['agent'] = request.agent
                    self._set_queue[index]['params'] = params
                elif request.type == 'cancel':
                    index = self._get_existing_job_index(self._set_queue,request.id)
                    del self._set_queue[index]
                else:
                    self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'Cannot change job type of existing job'))
            elif request.id in [j['id'] for j in self._validate_queue]:
                if request.type == 'validate_pose' or request.type == 'validate_gripper':
                    index = self._get_existing_job_index(self._validate_queue,request.id)
                    self._validate_queue[index]['agent'] = request.agent
                    self._validate_queue[index]['params'] = params
                elif request.type == 'cancel':
                    index = self._get_existing_job_index(self._validate_queue,request.id)
                    del self._validate_queue[index]
                else:
                    self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'Cannot change job type of existing job'))
            elif request.id in [j['id'] for j in self._tof_queue]:
                if request.type == 'tof_pose' or request.type == 'tof_gripper':
                    index = self._get_existing_job_index(self._tof_queue,request.id)
                    self._tof_queue[index]['agent'] = request.agent
                    self._tof_queue[index]['params'] = params
                elif request.type == 'cancel':
                    index = self._get_existing_job_index(self._tof_queue,request.id)
                    del self._tof_queue[index]
                else:
                    self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'Cannot change job type of existing job'))
            else:
                submitNew = True
        else:
            if request.type == 'cancel':
                self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,True,'Cannot cancel job without id'))
            request.id = 'job-{0}'.format(uuid.uuid4())
            submitNew = True
        self._lock.release()

        # package job and append to appropriate queue, if required
        if submitNew:
            job = {
                'id': request.id,
                'type': request.type,
                'agent': request.agent,
                'params': params
            }

            self._lock.acquire()
            if request.type == 'tof_pose' or request.type == 'tof_gripper':
                self._tof_queue.append(job)
            elif request.type == 'validate_pose' or request.type == 'validate_gripper':
                self._validate_queue.append(job)
            elif request.type == 'set_pose' or request.type == 'set_gripper':
                self._set_queue.append(job)
            self._lock.release()

            self._job_submission_publisher.publish(JobSubmissionResponse(request.id,request.agent,request.type,False,''))

    def _get_existing_job_index(self,queue,id):
        for i in range(0,len(queue)):
            if queue[i]['id'] == id:
                return i
        return -1

    #===========================================================================
    #   Simulate Action
    #===========================================================================

    def simulate(self, message):
        '''
        Simulate callback handle - acts as a switchboard based on Simulate Action
        mode selected.

        Provides full simulation, pause, and reset capabilities

        Returns : None
        '''

        if message.mode == '' or message.mode == 'simulate':
            self._simulate(message)
        elif message.mode == 'pause':
            self._pause()
        elif message.mode == 'reset':
            self._initialize()
        else:
            self._simulate_publish_result(SimulateResult.ERROR,'Invalid simulation mode')

    def _initialize(self):
        '''
        Sets the initial configuration of the simulation state. This involves
        moving all things and agents to their program defined starting locations.

        Returns : None
        '''

        time = 0
        feedback_status = SimulateFeedback.PENDING
        result_status = SimulateResult.INVALID
        error_msg = ''

        while True:

            # Publish periodic feedback and delay
            self._simulate_publish_feedback(feedback_status,{},time,1,{})
            rospy.sleep(SIMULATION_TIMESTEP)

            # handle preempt by canceling
            if self.simulate_server.is_preempt_requested():
                feedback_status = SimulateFeedback.PREEMPTED
                result_status = SimulateResult.PREEMPTED
                error_msg = 'Server preempted'
                break # Get out of the simulation loop

            # wait for job processor to finish all current and pending jobs
            if feedback_status == SimulateFeedback.PENDING:
                self._lock.acquire()
                if self._state != State.IDLE and self._state != State.SIMULATE:
                    self._lock.release()
                    continue  # Start next cycle of the loop
                elif len(self._set_queue) == 0 and len(self._validate_queue) == 0 and len(self._tof_queue) == 0:
                    self._state = State.SIMULATE
                    feedback_status = SimulateFeedback.INITIALIZING
                    time += SIMULATION_TIMESTEP
                    self._lock.release()
                    continue # Start next cycle of the loop

            # Run initialization routine
            if feedback_status == SimulateFeedback.INITIALIZING:
                _ot, initial_state, total_time = self._simulate_generate_timeline()

                # set the environment back to its original form
                self._simulate_set_state(initial_state)

                # regardless of state, deactive the agent
                for agent in initial_state['agents'].keys():
                    self._agents[agent]['thread'].deactivate()

                feedback_status = SimulateFeedback.FINISHED
                result_status = SimulateResult.FINISHED
                time += SIMULATION_TIMESTEP
                break # Get out of the simulation loop

        # Publish Action final results
        self._simulate_publish_feedback(feedback_status,{},time,total_time,{})
        self._simulate_publish_result(result_status,'')

        # release lock to job processor
        self._lock.acquire()
        if self._state == State.SIMULATE:
            self._state = State.IDLE
        self._lock.release()

    def _pause(self):
        '''
        Pause ends the current action preserves the simulation state

        Returns : None
        '''

        self._simulate_publish_result(SimulateResult.FINISHED,'')

    def _simulate(self, message):
        '''
        Runs simulation of program from the start time specified until finished
        or paused.

        Simulation is run as a statemachine using the SimulateFeedback status
        flag as the state variable.

        message : Simulate Action Goal message

        Returns : None
        '''

        # simulation state
        start_time = message.start_time
        current_time = start_time
        total_time = 0
        timeline = {}
        initial_state = {}
        start_state = {}
        current = {}
        span = 0

        # start execution loop
        feedback_status = SimulateFeedback.PENDING
        result_status = SimulateResult.INVALID
        error_msg = ''
        try:
            while True:

                # publish periodic feedback and delay
                self._simulate_publish_feedback(feedback_status,timeline,current_time,total_time,current)
                rospy.sleep(SIMULATION_TIMESTEP)

                # handle preempt by canceling
                if self.simulate_server.is_preempt_requested():
                    feedback_status = SimulateFeedback.PREEMPTED
                    result_status = SimulateResult.PREEMPTED
                    error_msg = 'Server preempted'
                    break # end simulation loop

                # spin check on status until freed by job processor
                if feedback_status == SimulateFeedback.PENDING:

                    self._lock.acquire()
                    if self._state != State.IDLE:
                        self._lock.release()
                    elif len(self._set_queue) == 0 and len(self._validate_queue) == 0 and len(self._tof_queue) == 0:
                        self._state = State.SIMULATE
                        feedback_status = SimulateFeedback.TRANSLATING
                        self._lock.release()
                    continue # start next cycle of simulation loop

                # translate plan to core simulation pattern
                if feedback_status == SimulateFeedback.TRANSLATING:

                    # generate simulation states
                    orig_timeline, initial_state, total_time = self._simulate_generate_timeline()
                    timeline, start_state, span = self._simulate_filter_timeline(orig_timeline, initial_state, start_time)
                    current = {agent: 0 for agent in timeline.keys()}

                    # Check if start-state is the end-state
                    if (total_time - start_time) <= 0:
                        feedback_status = SimulateFeedback.FINISHED
                        result_status = SimulateResult.FINISHED
                        error_msg = ''
                        break # end simulation loop

                    feedback_status = SimulateFeedback.INITIALIZING
                    continue # start next cycle of simulation loop

                # spin check while initializing simulation
                if feedback_status == SimulateFeedback.INITIALIZING:
                    # publish before going into blocking call
                    self._simulate_publish_feedback(feedback_status,timeline,current_time,total_time,current)
                    self._simulate_set_state(start_state)
                    feedback_status = SimulateFeedback.RUNNING
                    continue # Continue to next cycle of simulation loop

                # spin check while running simulation
                if feedback_status == SimulateFeedback.RUNNING:
                    if self._simulate_has_more(timeline,current):

                        msg = self._simulate_update_state(current_time,current,timeline) #TODO this is blocking for some reason
                        if msg != '':
                            error_msg = msg
                            feedback_status = SimulateFeedback.ERROR
                            result_status = SimulateResult.ERROR
                            break # error occurred while updating state, end simulation

                        # update time
                        current_time += SIMULATION_TIMESTEP
                        #print start_time, current_time, total_time

                        # check if time exceeded
                        if current_time > (total_time + SIMULATION_TOTAL_TIME_GRACE_PERIOD * span):
                            error_msg = 'Total time exceeded expected and grace period'
                            feedback_status = SimulateFeedback.ERROR
                            result_status = SimulateResult.ERROR
                            break # time exceeded expected for simulation, error occurred, end simulation

                        continue  # continue to next loop cycle of simulation

                    else:
                        # like initialize, revert back to start on simulation
                        feedback_status = SimulateFeedback.FINISHED
                        result_status = SimulateResult.FINISHED
                        break # done with simulation

        except Exception, e:
            traceback.print_exc()
            feedback_status = SimulateFeedback.ERROR
            result_status = SimulateResult.ERROR
            error_msg = 'Exception raised while running simulation'

        # regardless of state, deactive the agent
        for agent in initial_state['agents'].keys():
            self._agents[agent]['thread'].deactivate()

        # publish final feedback
        self._simulate_publish_feedback(feedback_status,{},current_time,total_time,{})

        # publish result
        if result_status == SimulateResult.ERROR:
            error_msg = 'Error running simulation: ' + error_msg
        self._simulate_publish_result(result_status,error_msg)

        # release lock to job processor
        self._lock.acquire()
        if self._state == State.SIMULATE:
            self._state = State.IDLE
        self._lock.release()

    def _simulate_set_state(self, state):

        # process agents
        if 'agents' in state.keys():
            # issue pose commands to threads
            for agent_key in state['agents'].keys():
                if 'pose' in state['agents'][agent_key]:
                    self._agents[agent_key]['thread'].activate(type='move',pose=state['agents'][agent_key]['pose'])

            # make thread work blocking
            self._block_active_agent_threads()

            # issue gripper commands to threads
            for agent_key in state['agents'].keys():
                if 'effort' in state['agents'][agent_key]:
                    self._agents[agent_key]['thread'].activate(type='grip',effort=state['agents'][agent_key]['effort'])

            # make thread work blocking
            self._block_active_agent_threads()

        # process things
        if 'things' in state.keys():
            for thing in state['things'].keys():
                self.set_thing_pose.publish(MarkerPose(id=thing,pose=state['things'][thing]['pose']))

    def _simulate_update_state(self, current_time, current, timeline):
        error_msg = ''

        print '\n', current_time, '\n'

        for agent in current.keys():
            if current[agent] < len(timeline[agent]):
                # state machine to run therbligs
                activeTherblig = timeline[agent][current[agent]]
                if activeTherblig['state'] == 'PENDING':

                    conditionsMet = activeTherblig['conditions']['start_time'] <= current_time

                    if conditionsMet:
                        activeTherblig['state'] = 'RUNNING'
                        time = activeTherblig['conditions']['duration'] - activeTherblig['conditions']['start_time'] #time estimated to execute
                        self._agents[agent]['thread'].activate(type=activeTherblig['type'],time=time,**activeTherblig['params'])
                    else: # need to wait for all conditions to be met
                        pass

                elif activeTherblig['state'] == 'RUNNING':
                    # check if therblig is done, otherwise nothing to do
                    status = self._agents[agent]['thread'].state

                    if status == 'idle':

                        if self._agents[agent]['thread'].in_error:
                            activeTherblig['state'] = 'ERROR'
                            error_msg += 'Therblig for agent "{}" failed. '.format(agent)
                        else:
                            activeTherblig['state'] = 'FINISHED'
                            current[agent] += 1

                        # make sure thing ends up in correct location
                        if 'thing' in activeTherblig.keys() and activeTherblig['thing'] != None:
                            pose = self._agents[agent]['planner'].get_ee_pose(self._agents[agent]['ee_group'])
                            pose = self._agent_ee_position_offset(agent,pose,-1)
                            self.set_thing_pose.publish(MarkerPose(
                                id=activeTherblig['thing'],
                                pose=pose))

                    else: # still running ('active')
                        #process thing
                        if 'thing' in activeTherblig.keys() and activeTherblig['thing'] != None:
                            pose = self._agents[agent]['planner'].get_ee_pose(self._agents[agent]['ee_group'])
                            pose = self._agent_ee_position_offset(agent,pose,-1)
                            self.set_thing_pose.publish(MarkerPose(
                                id=activeTherblig['thing'],
                                pose=pose))

                else:
                    # should not be in this condition
                    activeTherblig['state'] = 'ERROR'
                    error_msg += 'Active therblig for agent "{}" has invalid state "{}" expecting "PENDING" or "RUNNING". '.format(agent,state)
        return error_msg

    #===========================================================================
    #   Simulate Utility Methods
    #===========================================================================

    def _simulate_generate_timeline(self):
        '''
        Converts expanded plan generated by Authr into an internal simulation
        timeline.

        Returns: timeline : Simulation Dictionary structure
        Returns: initial_state : Expected starting state of agents and things
        Returns: total_time : Duration expected to run program
        '''

        # get expanded plan and transform to simulate timeline
        response = self.get_element('expanded_plan_cached','').response
        #print response
        loaded_response = json.loads(response)
        if 'expanded_plan' not in loaded_response.keys():
            return {}, {}, 0
        expanded_plan = json.loads(response)['expanded_plan']
        timeline = {agent: [self._simulate_convert_expansion_element_to_sim_struct_element(element, agent) for element in expanded_plan[agent]] for agent in expanded_plan.keys()}

        # capture the state before simulation for all things and agents used
        initial_state = self._simulate_generate_initial_state(timeline)

        # determine total time to simulate
        total_time = 0
        current_time = 0
        for agent in timeline.keys():
            for element in timeline[agent]:
                current_time += element['conditions']['duration']
            if current_time > total_time:
                total_time = current_time

        return timeline, initial_state, total_time

    def _simulate_convert_expansion_element_to_sim_struct_element(self, expansion_element, agent):
        '''
        Converts an element from the expanded program generated by Authr into the
        internal simulation structure elements.

        expansion_element : Authr plan element structure
        agent : ID for agent in structure to convert pose

        Returns : Internal simulation dictionary structure
        '''

        type = None
        params = {}
        thing = None

        if expansion_element['therblig']['type'] in ['transport_empty','transport_loaded','position','preposition']:
            type = 'move'

            dest_id = expansion_element['therblig']['parameters']['destination']
            dest_dct = json.loads(self.get_element('destination',dest_id).response)
            params['pose'] = Pose(position=msg_conv.position_msgFromDict(dest_dct['position']),
                                  orientation=msg_conv.orientation_quaternionMsgFromEulerDict(dest_dct['orientation']))

            params['pose'] = self._agent_ee_position_offset(agent,params['pose'],+1)

            if 'thing' in expansion_element['therblig']['parameters'].keys():
                thing = expansion_element['therblig']['parameters']['thing']

                params['duration'] = expansion_element['duration']

            if 'trajectory_id' in expansion_element['therblig'].keys():
                tid = expansion_element['therblig']['trajectory_id']
                if tid in self._joint_trajectory_cache.keys():
                    params['plan'] = self._joint_trajectory_cache[tid]

        elif expansion_element['therblig']['type'] in ['grasp','release_load']:
            type = 'grip'

            if 'effort' in expansion_element['therblig']['parameters'].keys():
                params['effort'] = expansion_element['therblig']['parameters']['effort']
            else:
                params['effort'] = 0 #default open

            if 'thing' in expansion_element['therblig']['parameters'].keys():
                thing = expansion_element['therblig']['parameters']['thing']

                params['duration'] = expansion_element['duration']

        elif expansion_element['therblig']['type'] in ['hold','rest']:
            type = 'delay'

            params['duration'] = expansion_element['duration']

        sim_struct_element = {
            'id': expansion_element['therblig']['eid'],
            'state': 'PENDING',
            'type': type,
            'params': params,
            'conditions': {
                'start_time': expansion_element['start_time'],
                'duration': expansion_element['duration']
            },
            'thing': thing
        }
        return sim_struct_element

    def _simulate_generate_initial_state(self, timeline):
        '''
        Generates the initial simulation pose state for agents and things in
        the timeline program.

        timeline : Program dictionary structure

        Returns : Dictionary containing initial poses for agents and things.
        '''

        # generate initial agent structure
        agents = {}
        for agent in timeline.keys():
            dest_dct = json.loads(self.get_element('destination',agent).response)

            agents[agent] = {
                'pose': Pose(position=msg_conv.position_msgFromDict(dest_dct['position']),
                             orientation=msg_conv.orientation_quaternionMsgFromEulerDict(dest_dct['orientation'])),
                'effort': 0
            }

            agents[agent]['pose'] = self._agent_ee_position_offset(agent,agents[agent]['pose'],+1)

        # find all things referenced in timeline
        things = {}
        for agent in timeline.keys():
            for element in timeline[agent]:
                if 'thing' in element.keys() and element['thing'] != None and element['thing'] not in things.keys():
                    dest_dct = json.loads(self.get_element('destination',element['thing']).response)

                    things[element['thing']] = {
                        'pose': Pose(position=msg_conv.position_msgFromDict(dest_dct['position']),
                                     orientation=msg_conv.orientation_quaternionMsgFromEulerDict(dest_dct['orientation']))
                    }

        state = {'agents':agents,'things':things}
        return state

    def _simulate_filter_timeline(self, timeline, initial_state, start_time):
        '''
        Generate a new program timeline as a subset of the full timeline that
        starts relatively close to the start_time point.

        timeline : Full timeline dictionary
        initial_state : Original initial state for timeline
        start_time : time to start the subset timeline

        Returns : timeline_filtered : subset timeline
        Returns : start_state : new initial state for subset timeline
        Returns : span : Duration of subset timeline
        '''

        timeline_filtered = {agent: [] for agent in timeline.keys()}
        start_state = copy.deepcopy(initial_state)

        span = 0
        temp = 0
        for agent in timeline.keys():
            for element in timeline[agent]:
                if element['conditions']['start_time'] >= start_time:

                    # copy from this element to end as new timeline
                    timeline_filtered[agent] = timeline[agent][timeline[agent].index(element):len(timeline[agent])]
                    temp = len(timeline_filtered[agent])
                    break
                else:

                    # update start state with the parameters and thing
                    if 'pose' in element['params'].keys():
                        start_state['agents'][agent]['pose'] = element['params']['pose']

                        if element['thing'] != None:
                            start_state['things'][element['thing']]['pose'] = element['params']['pose']

                    if 'effort' in element['params'].keys():
                        start_state['agents'][agent]['effort'] = element['params']['effort']

            if temp > span:
                span = temp

        return timeline_filtered, start_state, span

    def _simulate_has_more(self, timeline, current):
        '''
        Checks if at least one agent still has work to do

        Returns : True if at least one agent has work else False
        '''

        for agent in current.keys():
            if current[agent] < len(timeline[agent]):
                return True
        return False

    def _simulate_publish_feedback(self, status, timeline, current_time, total_time, current_state):
        '''
        Publishes the feedback for the Simulate Action

        status : Integer status flag defined in Action
        timeline : Dictionary for all therbligs to run
        current_time : Number representing current elapsed time in timeline
        total_time : Number representing total duration of timeline
        current_state : Index for each agent in the timeline

        Returns : None
        '''

        feedback = SimulateFeedback()
        feedback.status = status
        feedback.elapsed_time = current_time
        feedback.percent_complete = math.floor(min(current_time/total_time,1)*1000)/10.0 if total_time != 0 else 0
        feedback.agent_therblig_map = json.dumps({agent: timeline[agent][current_state[agent]]['id'] if current_state[agent] < len(timeline[agent]) else '' for agent in current_state.keys()})
        self.simulate_server.publish_feedback(feedback)

    def _simulate_publish_result(self, status, msg):
        '''
        Publishes the results for the Simulate Action

        status : Integer status flag defined in Action
        msg: String message to display on frontend

        Returns : None
        '''

        result = SimulateResult()
        result.status = status
        result.msg = msg
        if status == 'preempted':
            self.simulate_server.set_preempted(result)
        elif status != 'error':
            self.simulate_server.set_succeeded(result)
        else:
            self.simulate_server.set_aborted(result)


if __name__ == "__main__":

    rospy.init_node("authr_sim")

    rospy.sleep(10) # delay until rest of ROS is up

    authr_sim = AuthrSim()
    rospy.loginfo("Authr Sim is running.")
    rospy.spin()

    authr_sim.stop_threads()
