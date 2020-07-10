import rospy
import threading
import traceback
from enum import Enum
from behavior_execution.planners.abstract import PlannerStatus

class ThreadState(Enum):
    IDLE = 'idle'
    ACTIVE = 'active'


class PrimitiveType(Enum):
    DELAY = 'delay'
    MOVE = 'move'
    GRIP = 'grip'


class AgentThread(object):

    THREAD_SLEEP_TIME = 0.005

    def __init__(self, planner, ee_group, finish_cb=None):
        self._state = ThreadState.IDLE
        self._kill = False
        self._ee_group = ee_group
        self._planner = planner
        self._thread = None
        self._finish_cb = finish_cb
        self._in_error = False
        self._activate = False
        self._staged_planner_change = None

    @property
    def state(self):
        return self._state.value

    @property
    def in_error(self):
        return self._in_error

    def set_finish_cb(self, cb):
        self._finish_cb = cb

    def update_planner(self, planner):
        self._staged_planner_change = planner

    def update_ee_group(self, ee_group):
        self._ee_group = ee_group

    def _execute(self):
        while not self._kill:
            if self._activate:

                # Run operation
                state = 'invalid'
                if self._params['type'] == PrimitiveType.DELAY.value:
                    rospy.sleep(self._params['duration'])
                    state = 'success'

                elif self._params['type'] == PrimitiveType.MOVE.value:
                    if self._planner != None:

                        if 'plan' in self._params:
                            try:
                                status = self._planner.execute(self._ee_group, self._params['plan'], wait=True, joint_target=True)
                                #state = 'success' if self._block_spin() else 'error'
                                state = 'success' if status else 'error'
                            except:
                                traceback.print_exc()
                                state = 'error'

                        else:
                            params = {'pose': self._params['pose'], 'wait':True}
                            if 'duration' in self._params:
                                params['duration'] = self._params['duration']

                            try:
                                status = self._planner.set_ee_pose(self._ee_group,**params)
                                #state = 'success' if self._block_spin() else 'error'
                                state = 'success' if status else 'error'
                            except:
                                traceback.print_exc()
                                state = 'error'

                elif self._params['type'] == PrimitiveType.GRIP.value:
                    if self._planner != None:

                        joints = self._planner.get_gripper_state(self._ee_group)
                        for index in range(0,len(joints)):
                            joints[index] = self._params['effort']

                        try:
                            status = self._planner.set_gripper_state(self._ee_group, joints, wait=True)
                            #self._time_spin()
                            state = 'success' if status else 'error'
                            #state = 'success' if self._block_spin() else 'error'
                        except:
                            traceback.print_exc()
                            state = 'error'

                # Update thread state
                self._in_error = (state == 'error')
                self._activate = False
                self._params = None
                self._state = ThreadState.IDLE

                # invoke callback
                if self._finish_cb !=None:
                    self._finish_cb(state)

            if self._staged_planner_change != None:
                self._planner = self._staged_planner_change
                self._staged_planner_change = None
            rospy.sleep(self.THREAD_SLEEP_TIME)

    def stop(self):
        self._kill = True
        self._params = None

    def start(self):
        self._activate = False
        self._kill = False
        self._params = None
        self._thread = threading.Thread(target=self._execute)
        self._thread.start()

    def deactivate(self):
        if self._planner != None:
            self._planner.stop(self._ee_group)
        self._state = ThreadState.IDLE
        self._activate = False
        self._params = None

    def activate(self, **kwargs):
        if not self._activate:
            self._in_error = False
            self._params = kwargs
            self._activate = True
            self._state = ThreadState.ACTIVE
        else:
            raise Exception('Already in active state')

    def _block_spin(self):
        success = None
        while success == None:
            rospy.sleep(self.THREAD_SLEEP_TIME)
            s = self._planner.status(self._ee_group)
            #print s
            success = (s == PlannerStatus.VALID) if s != PlannerStatus.PENDING else None
        return success
