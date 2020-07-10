import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from authr_tools.elements import Type

class SimpleMarkerServer(object):
    def __init__(self,topic):
        self.markers = {}
        self.current_id = 0
        self.pub = rospy.Publisher(topic, MarkerArray,queue_size=10)
        self.sub = rospy.Subscriber(topic+"refresh", String, self.refresh)

    @property
    def marker_array(self):
        return [self.markers[marker]["object_marker"] for marker in self.markers]+\
               [self.markers[marker]["name_marker"] for marker in self.markers]

    def refresh(self,*kwargs):
        self.pub.publish(self.marker_array)

    def set_pose(self, id, pose, disableRefresh=False):
        self.markers[id]["object_marker"].pose = pose
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

        if not disableRefresh:
            self.refresh()

    def set_type(self, id, type, disableRefresh=False):

        if type == Type.ROBOT:
            self.markers[id]["object_marker"].type = Marker.ARROW
            self.markers[id]["object_marker"].color.a = 0.5

            self.markers[id]["object_marker"].pose.orientation.x = 0.0
            self.markers[id]["object_marker"].pose.orientation.y = -0.7071068
            self.markers[id]["object_marker"].pose.orientation.z = 0.0
            self.markers[id]["object_marker"].pose.orientation.w = 0.7071068
        elif type == Type.HUMAN:
            self.markers[id]["object_marker"].type = Marker.ARROW
            self.markers[id]["object_marker"].color.a = 0.5

            self.markers[id]["object_marker"].pose.orientation.x = 0.0
            self.markers[id]["object_marker"].pose.orientation.y = -0.7071068
            self.markers[id]["object_marker"].pose.orientation.z = 0.0
            self.markers[id]["object_marker"].pose.orientation.w = 0.7071068
        elif type == Type.DESTINATION:
            self.markers[id]["object_marker"].type = Marker.ARROW
            self.markers[id]["object_marker"].color.a = 0.5
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
            self.markers[id]["object_marker"].mesh_resource = "package://custom_meshes/box.dae"
            self.markers[id]["object_marker"].color.a = 1
        elif type == Type.HAND:
            self.markers[id]["object_marker"].type = Marker.MESH_RESOURCE
            self.markers[id]["object_marker"].mesh_resource = "package://custom_meshes/hand.dae"
            self.markers[id]["object_marker"].color.a = 1

        if not disableRefresh:
            self.refresh(id)

    def delete_marker(self,id):
        self.markers.pop(id)
        self.refresh()


    def add_marker(self,id,type,pose,name=''):
        self.markers[id] = {"name":name,
                            "size": 0.1,
                            "object_marker":Marker(id=self.current_id),
                            "name_marker":Marker(id=self.current_id+1)}
        self.current_id += 2

        # Setup the object marker
        self.markers[id]["object_marker"].header.frame_id = 'base_link'
        self.markers[id]["name_marker"].header.frame_id = 'base_link'
        self.markers[id]["name_marker"].text = name
        self.markers[id]["name_marker"].type = Marker.TEXT_VIEW_FACING


        self.set_type(id,type,disableRefresh=True)
        self.set_pose(id,pose,disableRefresh=True)
        self.set_size(id,0.1,disableRefresh=True)
        self.set_color(id,100,100,100,disableRefresh=True)
        self.refresh()
