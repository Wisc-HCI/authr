#!/usr/bin/env python

#John Balis 2018
#for support email balisujohn@gmail.com


import sys
import rospy
from copy import deepcopy
from std_msgs.msg import String
from authr.srv import OptimizeQuery
import json

def convert(input):
    if isinstance(input, dict):
        return {convert(key): convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def tester(plan):
    # pub = rospy.Publisher('optimization_queue', Constraints, queue_size=10)
    rospy.init_node('tester', anonymous=True)
    optimizer = rospy.ServiceProxy("authr_opt/optimize", OptimizeQuery)

    message = OptimizeQuery()

    id = 1
    constraints= """{
		"name": "example",
		"timeweight" : 1,
		"costweight" : 1,
			"tasks": {
				"moveempty1": {
					"preconditions": "none",
					"duration": { "human" : 5,
						"robot" : 5
					},
					"cost": { "human" : 5,
						"robot" : 5
					},
					"constraint": "start : path1"
				},
				"grasp1": {
					"preconditions": "moveempty1",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 5,
						"robot" : 5					    },
					"constraint": "path1"
				},
				"transportloaded1": {
					"preconditions": "grasp1",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 5,
						"robot" : 5					    },
					"constraint": "path1"
				},
				"release1": {
					"preconditions": "transportloaded1",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 5,
						"robot" : 5					    },
					"constraint": "end : path1"
				},
				"release2": {
					"preconditions": "transportloaded2",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 5,
						"robot" : 5					    },
					"constraint": "end : path2"
				},
				"moveempty2": {
					"preconditions": "none",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 5,
						"robot" : 5					    },
					"constraint": "start : path2"
				},
				"grasp2": {
					"preconditions": "moveempty2",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 5,
						"robot" : 5					    },
					"constraint": "path2"
				},
				"transportloaded2": {
					"preconditions": "grasp2",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 5,
						"robot" : 5					    },
					"constraint": "path2"
				},
				"anytime1": {
					"preconditions": "none",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 999,
						"robot" : 5					    },

					"constraint": "none"
				},
				"anytime2": {
					"preconditions": "none",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 999,
						"robot" : 5					    },
					"constraint": "none"
				},
				"anytime3": {
					"preconditions": "none",
					"duration": { "human" : 5,
						"robot" : 5					    },
					"cost": { "human" : 999,
						"robot" : 5					    },

					"constraint": "none"
				}

			}

	}
	"""

    #rospy.loginfo(plan)
    response = optimizer(id,plan)
    rospy.loginfo(response)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("No JSON File Provided!")
    else:
        file = sys.argv[1]
        with open(file, "r") as read_file:
            data = convert(json.load(read_file))
        tester(json.dumps(data))
    # try:
    #     tester()
    # except rospy.ROSInterruptException:
    #     pass
