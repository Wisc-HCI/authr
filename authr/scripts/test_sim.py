#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from authr.msg import Constraints
from authr.srv import OptimizeRequest


def tester():
    # pub = rospy.Publisher('optimization_queue', Constraints, queue_size=10)
    rospy.init_node('tester', anonymous=True)

    # Plan data is an example output from the optimizer
    plan_data = {'cost': 3037.0,
                 'time': 35.1,
                 'plan': {'grasp1':           {'duration': '5', 'cost': '5',   'agent': 'h', 'starttime': '5.02',  "type": "grasp", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'anytime2':         {'duration': '5', 'cost': '999', 'agent': 'h', 'starttime': '20.08', "type": "grasp", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'anytime1':         {'duration': '5', 'cost': '999', 'agent': 'h', 'starttime': '25.09', "type": "grasp", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'grasp2':           {'duration': '5', 'cost': '5',   'agent': 'r', 'starttime': '5.03',  "type": "grasp", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'release1':         {'duration': '5', 'cost': '5',   'agent': 'h', 'starttime': '15.06', "type": "release", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'release2':         {'duration': '5', 'cost': '5',   'agent': 'r', 'starttime': '15.07', "type": "release", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'moveempty1':       {'duration': '5', 'cost': '5',   'agent': 'h', 'starttime': '0.00',  "type": "transportempty", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'moveempty2':       {'duration': '5', 'cost': '5',   'agent': 'r', 'starttime': '0.01',  "type": "transportempty", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'transportloaded1': {'duration': '5', 'cost': '5',   'agent': 'h', 'starttime': '10.04', "type": "transportloaded", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'transportloaded2': {'duration': '5', 'cost': '5',   'agent': 'r', 'starttime': '10.05', "type": "transportloaded", "parameters": {"effort": 0.5, "arm": "manipulator"}},
                          'anytime3':         {'duration': '5', 'cost': '999', 'agent': 'h', 'starttime': '30.10', "type": "grasp", "parameters": {"effort": 0.5, "arm": "manipulator"}}}}

    # Test EndEffectorPoseQuery

    # Test SetForceControlQuery

    # Test TimeOfFlightQuery

    # Test Simulate Action


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass
