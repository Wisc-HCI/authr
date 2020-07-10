#!/usr/bin/env python

import sys
import json
import rospy
import moveit_commander

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String
from authr.srv import GetRequest, GetRequestResponse, SetRequest, SetRequestResponse
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped


moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


class Table:

    def __init__(self):
        self._count = 0
        self.position = Point(x=0, y=0.44 ,z=-0.01)
        self.orientation = Quaternion(x=0, y=0, z=0, w=1)
        self.size  = (2, 1, 0.01)
        self.color = ColorRGBA(0.65, 0.78, 0.9, 1.0)

        self.get_element_srv = rospy.Service("authr_env/get", GetRequest, self.get)
        self.set_element_srv = rospy.Service("authr_env/set", SetRequest, self.set)

        self.refresh_subscriber = rospy.Subscriber('/authr_env/refresh', String, self.refresh)

        self.marker_publisher = rospy.Publisher('/authr_env/table_marker', Marker, queue_size=10)
        rospy.sleep(0.5) # wait for topic to setup

        self._collision_object()
        self._marker()

    def refresh(self, message):
        if message.data == 'marker':
            self._marker()
        elif message.data == 'collision':
            self._collision_object()
        else:
            self._marker()
            self._collision_object()

    def set(self,message):
        dict = json.loads(message.settings)

        if message.type == 'position':
            self.position.x = dict['x']
            self.position.y = dict['y']
            self.position.z = dict['z']
            return SetRequestResponse(True,'')
        elif message.type == 'orientation':
            (x,y,z,w) = tf.transformations.quaternion_from_euler(
                dict['x'],
                dict['y'],
                dict['z'],
                axes='sxyz'
            )
            self.orientation.x = x
            self.orientation.y = y
            self.orientation.z = z
            self.orientation.w = w
            return SetRequestResponse(True,'')
        elif message.type == 'size':
            self.size = (dict['x'],dict['y'],dict['z'])
            return SetRequestResponse(True,'')
        elif message.type == 'color':
            self.color.r = dict['r']
            self.color.g = dict['g']
            self.color.b = dict['b']
            self.color.a = dict['a']
        else:
            return SetRequestResponse(False,'Error: invalid type')

    def get(self,message):
        if message.type == 'position':
            position_str = json.dumps({
                'x': self.position.x,
                'y': self.position.y,
                'z': self.position.z
            })
            return GetRequestResponse(position_str)
        elif message.type == 'orientation':
            (r,p,y) = tf.transformations.euler_from_quaternion([
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
                self.orientation.w
            ])
            orientation_str = json.dumps({
                'x': r,
                'y': p,
                'z': y
            })
            return GetRequestResponse(orientation_str)
        elif message.type == 'size':
            size_str = json.dumps({
                'x': self.size[0],
                'y': self.size[1],
                'z': self.size[2]
            })
            return GetRequestResponse(size_str)
        elif message.type == 'color':
            color_str = json.dumps({
                'r': self.color.r,
                'g': self.color.g,
                'b': self.color.b,
                'a': self.color.a
            })
            return GetRequestResponse(color_str)
        else:
            return GetRequestResponse('{}')

    def _collision_object(self):
        scene.remove_world_object("table")
        scene.add_box(
            name="table",
            pose=PoseStamped(
                header=Header(frame_id=robot.get_planning_frame()),
                pose=Pose(
                    position=Vector3(self.position.x,self.position.y,self.position.z-0.005),
                    orientation=self.orientation)),
            size=self.size)

        rospy.sleep(1)
        rospy.loginfo(scene.get_known_object_names())

    def _marker(self):
        marker = Marker(
            type=Marker.CUBE,
            id=0,
            pose=Pose(self.position, self.orientation),
            scale=Vector3(self.size[0], self.size[1], self.size[2]),
            header=Header(frame_id=robot.get_planning_frame()),
            color=self.color)
        self.marker_publisher.publish(marker)

    def loop(self):
        count = self.marker_publisher.get_num_connections()
        if count != self._count:
            self._count = count
            self._marker()


if __name__ == "__main__":

    rospy.init_node("authr_env")
    rospy.sleep(2)  # wait for rest of ROS to setup

    table = Table()

    while not rospy.is_shutdown():
        table.loop()
        rospy.sleep(1)
