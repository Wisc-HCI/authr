#!/usr/bin/env python
import rospy
import json
from authr.srv import GetRequest, SetRequest, CreateRequest, DeleteRequest


def tester():

    rospy.init_node('tester', anonymous=True)
    # Setup Service Proxies
    get_service = rospy.ServiceProxy("authr_plan/get_element", GetRequest)
    set_service = rospy.ServiceProxy("authr_plan/set_element", SetRequest)
    create_service = rospy.ServiceProxy("authr_plan/create_element", CreateRequest)
    delete_service = rospy.ServiceProxy("authr_plan/delete_element", DeleteRequest)

    # Test initial setup
    rospy.loginfo("Testing initial values.")
    calls = [("task_keys",""),
             ("task",""),
             ("therblig",""),
             ("agent",""),
             ("thing",""),
             ("destination","")]
    for call in calls:
        rospy.loginfo(json.loads(get_service(*call).response))

    # Try Adding an Agent
    rospy.loginfo("Creating an Agent.")
    response = create_service("agent","{}")
    if not response.success:
        rospy.loginfo(response.message)
    response = json.loads(get_service("agent","").response)
    rospy.loginfo(response)
    agent_id = response.keys()[0]
    response = json.loads(get_service("agent",agent_id).response)
    rospy.loginfo(response)

    # Try adding a Therblig
    rospy.loginfo("Creating a Therblig.")
    tasks = json.loads(get_service("task_keys","").response)
    parameters = json.dumps({"index":0,"task_id":tasks[0],"type":"transport_empty"})
    response = create_service("therblig",parameters)
    if not response.success:
        rospy.loginfo(response.message)
    response = json.loads(get_service("therblig","").response)
    rospy.loginfo(response)

    # Try adding a Thing
    rospy.loginfo("Creating a Thing.")
    response = create_service("thing","{}")
    if not response.success:
        rospy.loginfo(response.message)
    response = json.loads(get_service("thing","").response)
    rospy.loginfo(response)
    response = json.loads(get_service("destination","").response)
    rospy.loginfo(response)

    # Try adding a Destination
    rospy.loginfo("Creating a Destination.")
    response = create_service("destination","{}")
    if not response.success:
        rospy.loginfo(response.message)
    response = json.loads(get_service("destination","").response)
    rospy.loginfo(response)

    rospy.loginfo("Deleting the Therblig")
    therbligs = json.loads(get_service("therblig_keys","").response)
    rospy.loginfo("therbligs before:")
    rospy.loginfo(therbligs)
    response = delete_service("therblig", therbligs[0])
    if not response.success:
        rospy.loginfo("Failed to delete the therblig")
    else:
        rospy.loginfo("Therblig successfully deleted")
    therbligs = json.loads(get_service("therblig_keys","").response)
    rospy.loginfo("therbligs after:")
    rospy.loginfo(therbligs)

    rospy.loginfo("Deleting the Task")
    tasks = json.loads(get_service("task_keys","").response)
    rospy.loginfo("tasks before:")
    rospy.loginfo(tasks)
    response = delete_service("task", tasks[0])
    if not response.success:
        rospy.loginfo("Failed to delete the task")
    else:
        rospy.loginfo("Task successfully deleted")
    tasks = json.loads(get_service("task_keys","").response)
    rospy.loginfo("tasks after:")
    rospy.loginfo(tasks)


if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass
