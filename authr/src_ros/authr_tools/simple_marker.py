import tf
import rospy
import copy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from authr_tools.elements import Type
from geometry_msgs.msg import Pose, Quaternion

class SimpleMarkerServer(object):

    ARROW_ANGLE_OFFSET = Quaternion(0.0,-0.7071068,0.0,0.7071068)

    def __init__(self,topic):
        self.markers = {}
        self.current_id = 0
        self.pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
        self.sub = rospy.Subscriber(topic+"refresh", String, self.refresh)
        self._visible = True

    def visible(self, state):
        if state and not self._visible:
            marray = copy.deepcopy(self.marker_array)
            for m in marray:
                m.action = Marker.ADD
            self.pub.publish(marray)
        elif not state and self._visible:
            marray = copy.deepcopy(self.marker_array)
            for m in marray:
                m.action = Marker.DELETE
            self.pub.publish(marray)

        self._visible = state

    @property
    def marker_array(self):
        return [self.markers[marker]["object_marker"] for marker in self.markers]+\
               [self.markers[marker]["name_marker"] for marker in self.markers]

    def refresh(self,*kwargs):
        if self._visible:
            self.pub.publish(self.marker_array)

    def hard_refresh(self, *kwargs):
        if self._visible:
            # Need to delete marker in order to force type or size updates
            marray = copy.deepcopy(self.marker_array)
            for m in marray:
                m.action = Marker.DELETE
            self.pub.publish(marray)
            rospy.sleep(0.001)
            for m in marray:
                m.action = Marker.ADD
            self.pub.publish(marray)

    def set_name(self, id, name, disableRefresh=False):
        self.markers[id]["name_marker"].text = ''#name
        if not disableRefresh:
            self.refresh(id)

    def set_position(self, id, position, disableRefresh=False):
        self.set_pose(id,Pose(position=position,orientation=self.markers[id]["object_marker"].pose.orientation), disableRefresh)

    def set_orientation(self, id, orientation, disableRefresh=False):
        self.set_pose(id,Pose(position=self.markers[id]["object_marker"].pose.position,orientation=orientation), disableRefresh)

    def set_pose(self, id, pose, disableRefresh=False):

        if self.markers[id]["object_marker"].type == Marker.ARROW:
            orVect = tf.transformations.quaternion_multiply([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],[self.ARROW_ANGLE_OFFSET.x,self.ARROW_ANGLE_OFFSET.y,self.ARROW_ANGLE_OFFSET.z,self.ARROW_ANGLE_OFFSET.w])
            orientation = Quaternion(orVect[0],orVect[1],orVect[2],orVect[3])
        else:
            orientation = pose.orientation
        self.markers[id]["object_marker"].pose = Pose(position=pose.position,orientation=orientation)

        self.markers[id]["name_marker"].pose.position.x = pose.position.x
        self.markers[id]["name_marker"].pose.position.y = pose.position.y
        size = self.markers[id]["size"]
        if size >= 0.2: # Large Object
            self.markers[id]["name_marker"].pose.position.z = pose.position.z + 2.2 * size + 0.2
        elif size >= 0.05: # Medium Object
            self.markers[id]["name_marker"].pose.position.z = pose.position.z + 2.2 * size + 0.15
        else: # Small Object
            self.markers[id]["name_marker"].pose.position.z = pose.position.z + 0.31

        if not disableRefresh:
            self.refresh()

    def set_color(self, id, r, g, b, disableRefresh=False):
        self.markers[id]["object_marker"].color.r = r / 255.0
        self.markers[id]["object_marker"].color.g = g / 255.0
        self.markers[id]["object_marker"].color.b = b / 255.0
        if not disableRefresh:
            self.refresh()

    def set_size(self, id, size, disableRefresh=False):
        if size is None:
            size = 0
        self.markers[id]["size"] = size

        #TODO fix size of text
        self.markers[id]["object_marker"].scale.x = size
        self.markers[id]["object_marker"].scale.y = size
        self.markers[id]["object_marker"].scale.z = size
        pose = self.markers[id]["object_marker"].pose

        if size >= 0.2: # Large Object
            self.markers[id]["name_marker"].pose.position.z = pose.position.z + 2.2 * size + 0.2
        elif size >= 0.05: # Medium Object
            self.markers[id]["name_marker"].pose.position.z = pose.position.z + 2.2 * size + 0.15
        else: # Small Object
            self.markers[id]["name_marker"].pose.position.z = pose.position.z + 0.31

        #ignore disable refresh as this needs to force change
        self.hard_refresh()

    def set_type(self, id, type, disableRefresh=False):

        if type == Type.ROBOT:
            self.markers[id]["object_marker"].type = Marker.ARROW
            self.markers[id]["object_marker"].color.a = 0.5
            self.markers[id]["object_marker"].pose.orientation = self.ARROW_ANGLE_OFFSET
        elif type == Type.HUMAN:
            self.markers[id]["object_marker"].type = Marker.ARROW
            self.markers[id]["object_marker"].color.a = 0.5
            self.markers[id]["object_marker"].pose.orientation = self.ARROW_ANGLE_OFFSET
        elif type == Type.DESTINATION:
            self.markers[id]["object_marker"].type = Marker.ARROW
            self.markers[id]["object_marker"].color.a = 0.5
            self.markers[id]["object_marker"].pose.orientation = self.ARROW_ANGLE_OFFSET
        elif type == Type.CYLINDER:
            self.markers[id]["object_marker"].type = Marker.CYLINDER
            self.markers[id]["object_marker"].color.a = 1
        elif type == Type.SPHERE:
            self.markers[id]["object_marker"].type = Marker.SPHERE
            self.markers[id]["object_marker"].color.a = 1
        elif type == Type.CUBE:
            self.markers[id]["object_marker"].type = Marker.CUBE
            self.markers[id]["object_marker"].color.a = 1
        elif type == Type.CONTAINER:
            self.markers[id]["object_marker"].type = Marker.MESH_RESOURCE
            self.markers[id]["object_marker"].mesh_resource = "package://custom_meshes/container.stl"
            self.markers[id]["object_marker"].color.a = 1
        elif type == Type.HAND:
            self.markers[id]["object_marker"].type = Marker.MESH_RESOURCE
            self.markers[id]["object_marker"].mesh_resource = "package://custom_meshes/hand.dae"
            self.markers[id]["object_marker"].color.a = 1

        #ignore disable refresh as this needs to force change
        self.hard_refresh()

    def delete_marker(self,id):
        self.markers[id]["object_marker"].action = Marker.DELETE
        self.markers[id]["name_marker"].action = Marker.DELETE
        self.refresh()
        self.markers.pop(id)

    def add_marker(self,id,type,pose,name=''):
        self.markers[id] = {"name":'',#name,
                            "size": 0.1,
                            "object_marker":Marker(id=self.current_id, ns='simpleMarkers'),
                            "name_marker":Marker(id=self.current_id+1, ns='simpleMarkers')}
        self.current_id += 2

        # Setup the object marker
        self.markers[id]["object_marker"].header.frame_id = 'base_link'
        self.markers[id]["name_marker"].header.frame_id = 'base_link'
        self.markers[id]["name_marker"].text = ''#name
        self.markers[id]["name_marker"].type = Marker.TEXT_VIEW_FACING
        self.markers[id]["name_marker"].color.r = 1
        self.markers[id]["name_marker"].color.g = 1
        self.markers[id]["name_marker"].color.b = 1
        self.markers[id]["name_marker"].color.a = 0.75
        self.markers[id]["name_marker"].scale.x = 0.15
        self.markers[id]["name_marker"].scale.y = 0.15
        self.markers[id]["name_marker"].scale.z = 0.15

        self.set_type(id,type,disableRefresh=True)
        self.set_pose(id,pose,disableRefresh=True)
        self.set_size(id,0.1,disableRefresh=True)
        self.set_color(id,100,100,100,disableRefresh=True)
        self.refresh()
