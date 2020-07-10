import rospy
import time
from std_msgs.msg import String
from authr_tools.elements import Type
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

TEXT_MARKER_APPEND = '--text--'

class MarkerServer(object):

    def __init__(self, topic):
        self.server = InteractiveMarkerServer(topic)
        self.markers = {}
        self.controls = [{"w":1,"x":1,"y":0,"z":0,"name":"move_x","mode":InteractiveMarkerControl.MOVE_AXIS},
                         {"w":1,"x":0,"y":1,"z":0,"name":"move_y","mode":InteractiveMarkerControl.MOVE_AXIS},
                         {"w":1,"x":0,"y":0,"z":1,"name":"move_Z","mode":InteractiveMarkerControl.MOVE_AXIS},
                         {"w":1,"x":1,"y":0,"z":0,"name":"rotate_x","mode":InteractiveMarkerControl.ROTATE_AXIS},
                         {"w":1,"x":0,"y":1,"z":0,"name":"rotate_y","mode":InteractiveMarkerControl.ROTATE_AXIS},
                         {"w":1,"x":0,"y":0,"z":1,"name":"rotate_z","mode":InteractiveMarkerControl.ROTATE_AXIS}]

        self.refresh_subscriber = rospy.Subscriber(topic+'/refresh', String, self._refresh_all_cb)
        self._visible = True

    def visible(self, state):

        for id in self.markers.keys():
            if state and not self._visible:
                control = InteractiveMarkerControl()
                control.always_visible = True;
                control.markers.append(self.markers[id]["internal_marker"])
                control.name = "internal_marker"
                self.markers[id]["interactive_marker"].controls.append(control)

                if self.markers[id]["movable"]:
                    # Create Move/Rotate Axis Control Handles
                    for control_config in self.controls:
                        # Loop through and create each control, adding to marker
                        control = InteractiveMarkerControl()
                        control.always_visible = True;
                        control.orientation.w = control_config["w"]
                        control.orientation.x = control_config["x"]
                        control.orientation.y = control_config["y"]
                        control.orientation.z = control_config["z"]
                        control.orientation = self.normalizeQuaternion(control.orientation)
                        control.name = control_config["name"]
                        control.interaction_mode = control_config["mode"]
                        self.markers[id]["interactive_marker"].controls.append(control)

                txtControl = InteractiveMarkerControl()
                txtControl.always_visible = True;
                txtControl.markers.append(self.markers[id]["text_marker"])
                txtControl.name = "interactive_text_marker"
                self.markers[id]["interactive_text_marker"].controls.append(txtControl)

                self.refresh(id)
            elif not state and self._visible:
                self.markers[id]["interactive_marker"].controls = []
                self.markers[id]["interactive_text_marker"].controls = []
                self.refresh(id)
        self._visible = state

    def add_marker(self, id, type, callback=None, name=''):
        self.markers[id] = {}
        if callback == None:
            callback = lambda pose: rospy.loginfo(pose)
        else:
            callback = callback

        self.markers[id]["callback"] = callback
        self.markers[id]["name"] = name
        self.markers[id]["interactive_marker"] = InteractiveMarker()
        self.markers[id]["interactive_marker"].header.frame_id = "base_link"
        self.markers[id]["interactive_marker"].name = id
        self.markers[id]["interactive_marker"].description = "Marker"
        self.markers[id]["interactive_marker"].scale = 0.2

        # Save the pose of the marker, this is for resetting the marker on property change
        self.markers[id]["pose"] = self.markers[id]["interactive_marker"].pose

        # Save whether it is movable or not
        self.markers[id]["movable"] = False

        self.markers[id]["internal_marker"] = Marker()
        self.set_type(id, type, True)

        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.markers.append(self.markers[id]["internal_marker"])
        control.name = "internal_marker"
        self.markers[id]["interactive_marker"].controls.append(control)

        self.set_movable(id,True,True)


        self.markers[id]["interactive_text_marker"] = InteractiveMarker()
        self.markers[id]["interactive_text_marker"].header.frame_id = "base_link"
        self.markers[id]["interactive_text_marker"].name = id + TEXT_MARKER_APPEND
        self.markers[id]["interactive_text_marker"].description = "Text Marker"
        self.markers[id]["interactive_text_marker"].scale = 0.2

        self.markers[id]["text_marker"] = Marker()
        self.markers[id]["text_marker"].type = Marker.TEXT_VIEW_FACING
        self.markers[id]["text_marker"].color.r = 1
        self.markers[id]["text_marker"].color.g = 1
        self.markers[id]["text_marker"].color.b = 1
        self.markers[id]["text_marker"].color.a = 0.75
        self.markers[id]["text_marker"].scale.x = 0.15
        self.markers[id]["text_marker"].scale.y = 0.15
        self.markers[id]["text_marker"].scale.z = 0.15
        self.markers[id]["text_marker"].pose.position.z = self.markers[id]["interactive_marker"].scale * 1.25
        self.markers[id]["text_marker"].text = ''

        txtControl = InteractiveMarkerControl()
        txtControl.always_visible = True;
        txtControl.markers.append(self.markers[id]["text_marker"])
        txtControl.name = "interactive_text_marker"
        self.markers[id]["interactive_text_marker"].controls.append(txtControl)

        self.server.insert(self.markers[id]["interactive_marker"], lambda message: self.processFeedback(id,message))
        self.server.insert(self.markers[id]["interactive_text_marker"], lambda x: rospy.loginfo(x))
        self.server.applyChanges()

    def set_name(self, id, name, disableRefresh=False):
        self.markers[id]["name"] = name
        if self.markers[id]['movable']:
            try:
                self.markers[id]["text_marker"].text = name
            except:
                pass
        if not disableRefresh:
            self.refresh(id)

    def set_type(self, id, type, disableRefresh=False):

        if type == Type.ROBOT:
            self.markers[id]["internal_marker"].type = Marker.ARROW
            self.markers[id]["internal_marker"].color.a = 0.5

            self.markers[id]["internal_marker"].pose.orientation.x = 0.0
            self.markers[id]["internal_marker"].pose.orientation.y = -0.7071068
            self.markers[id]["internal_marker"].pose.orientation.z = 0.0
            self.markers[id]["internal_marker"].pose.orientation.w = 0.7071068
        elif type == Type.HUMAN:
            self.markers[id]["internal_marker"].type = Marker.ARROW
            self.markers[id]["internal_marker"].color.a = 0.5

            self.markers[id]["internal_marker"].pose.orientation.x = 0.0
            self.markers[id]["internal_marker"].pose.orientation.y = -0.7071068
            self.markers[id]["internal_marker"].pose.orientation.z = 0.0
            self.markers[id]["internal_marker"].pose.orientation.w = 0.7071068
        elif type == Type.DESTINATION:
            self.markers[id]["internal_marker"].type = Marker.ARROW
            self.markers[id]["internal_marker"].color.a = 0.5

            self.markers[id]["internal_marker"].pose.orientation.x = 0.0
            self.markers[id]["internal_marker"].pose.orientation.y = -0.7071068
            self.markers[id]["internal_marker"].pose.orientation.z = 0.0
            self.markers[id]["internal_marker"].pose.orientation.w = 0.7071068
        elif type == Type.CYLINDER:
            self.markers[id]["internal_marker"].type = Marker.CYLINDER
            self.markers[id]["internal_marker"].color.a = 1
        elif type == Type.SPHERE:
            self.markers[id]["internal_marker"].type = Marker.SPHERE
            self.markers[id]["internal_marker"].color.a = 1
        elif type == Type.CUBE:
            self.markers[id]["internal_marker"].type = Marker.CUBE
            self.markers[id]["internal_marker"].color.a = 1
        elif type == Type.CONTAINER:
            self.markers[id]["internal_marker"].type = Marker.MESH_RESOURCE
            self.markers[id]["internal_marker"].mesh_resource = "package://custom_meshes/container.stl"
            self.markers[id]["internal_marker"].color.a = 1
        elif type == Type.HAND:
            self.markers[id]["internal_marker"].type = Marker.MESH_RESOURCE
            self.markers[id]["internal_marker"].mesh_resource = "package://custom_meshes/hand.dae"
            self.markers[id]["internal_marker"].color.a = 1

        if not disableRefresh:
            self.refresh(id)
        print("set_type")

    def processFeedback(self,id,message):
        print("processFeedback")
        self.markers[id]["interactive_marker"].pose = message.pose

        self.markers[id]["interactive_text_marker"].pose.position = message.pose.position
        self.server.insert(self.markers[id]["interactive_text_marker"], lambda x: rospy.loginfo(x))
        self.server.applyChanges()

        self.markers[id]["callback"](message)

    def set_callback(self, id, callback):
        # Callback always takes in a pose argument
        self.markers[id]["callback"] = callback

    def set_movable(self, id, value=True, disableRefresh=False):
        if value == True and not self.markers[id]["movable"]:
            self.markers[id]["movable"] = True
            try:
                self.markers[id]["text_marker"].text = self.markers[id]['name']
            except:
                pass

            # Create Move/Rotate Axis Control Handles
            for control_config in self.controls:
                # Loop through and create each control, adding to marker
                control = InteractiveMarkerControl()
                control.always_visible = True;
                control.orientation.w = control_config["w"]
                control.orientation.x = control_config["x"]
                control.orientation.y = control_config["y"]
                control.orientation.z = control_config["z"]
                control.orientation = self.normalizeQuaternion(control.orientation)
                control.name = control_config["name"]
                control.interaction_mode = control_config["mode"]
                self.markers[id]["interactive_marker"].controls.append(control)
        elif value == False and self.markers[id]["movable"]:
            self.markers[id]["movable"] = False
            rospy.logwarn(len(self.markers[id]["interactive_marker"].controls))
            self.markers[id]["interactive_marker"].controls = self.markers[id]["interactive_marker"].controls[:1]
            #self.markers[id]["interactive_marker"].controls = []
            rospy.logwarn(len(self.markers[id]["interactive_marker"].controls))
            try:
                self.markers[id]["text_marker"].text = ''
            except:
                pass

        if not disableRefresh:
            self.refresh(id)
        print("set_movable")

    def set_size(self, id, size, disableRefresh=False):
        if size is None:
            size = 0

        #TODO fix size of text

        self.markers[id]["internal_marker"].scale.x = size
        self.markers[id]["internal_marker"].scale.y = size
        self.markers[id]["internal_marker"].scale.z = size

        if size >= 0.2: # Large Object
            self.markers[id]["interactive_marker"].scale = 2.2 * size
            self.markers[id]["text_marker"].pose.position.z = 2.2 * size + 0.2
        elif size >= 0.05: # Medium Object
            self.markers[id]["interactive_marker"].scale = 2.2 * size
            self.markers[id]["text_marker"].pose.position.z = 2.2 * size + 0.15
        else: # Small Object
            self.markers[id]["interactive_marker"].scale = 0.11
            self.markers[id]["text_marker"].pose.position.z = 0.31

        if not disableRefresh:
            self.refresh(id)
        print("set_size")

    def set_color(self, id, r, g, b, disableRefresh=False):
        self.markers[id]["internal_marker"].color.r = r / 255.0
        self.markers[id]["internal_marker"].color.g = g / 255.0
        self.markers[id]["internal_marker"].color.b = b / 255.0
        if not disableRefresh:
            self.refresh(id)
        print("set_color")

    def set_position(self, id, position, disableRefresh=False):
        self.markers[id]["interactive_marker"].pose.position = position
        self.markers[id]["interactive_text_marker"].pose.position = position
        if not disableRefresh:
            self.refresh(id)
        print("set_position")

    def set_orientation(self, id, orientation, disableRefresh=False):
        self.markers[id]["interactive_marker"].pose.orientation = orientation
        if not disableRefresh:
            self.refresh(id)
        print("set_orientation")

    def refresh(self, id):
        self.server.insert(self.markers[id]["interactive_marker"], lambda message: self.processFeedback(id,message))
        self.server.insert(self.markers[id]["interactive_text_marker"], lambda x: rospy.loginfo(x))
        self.server.applyChanges()

    def delete_marker(self, id):
        self.markers.pop(id)
        self.server.erase(id)
        self.server.erase(id + TEXT_MARKER_APPEND)
        self.server.applyChanges()

    def _refresh_all_cb(self, message):
        for id in self.markers.keys():
            self.refresh(id)
            rospy.sleep(0.1)

    @staticmethod
    def normalizeQuaternion( quaternion_msg ):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s
        return quaternion_msg


if __name__ == "__main__":
    rospy.init_node("Interactive_Marker")
    ms = MarkerServer()
    rospy.spin()
