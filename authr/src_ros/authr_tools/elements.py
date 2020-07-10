#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Vector3
import random
from enum import Enum
from authr.msg import EulerPose
import tf

def random_color():
    return {"r":random.randrange(0,156),
            "g":random.randrange(100,256),
            "b":random.randrange(100,256)}

class Type(Enum):
    HUMAN="human"
    HAND='hand'
    ROBOT="robot"
    DESTINATION="destination"
    CUBE="cube"
    SPHERE="sphere"
    CYLINDER="cylinder"
    CONTAINER="container"
    NOTSET="notset"

    @staticmethod
    def fromStr(string):
        s = string.lower()
        if s == Type.HUMAN.value:
            return Type.HUMAN
        elif s == Type.HAND.value:
            return Type.HAND
        elif s == Type.ROBOT.value:
            return Type.ROBOT
        elif s == Type.CUBE.value:
            return Type.CUBE
        elif s == Type.SPHERE.value:
            return Type.SPHERE
        elif s == Type.CYLINDER.value:
            return Type.CYLINDER
        elif s == Type.CONTAINER.value:
            return Type.CONTAINER
        elif s == Type.NOTSET.value:
            return Type.NOTSET
        else:
            raise ValueError('Enum supplied does not exist')

class Element(object):
    """
    Parent Class for Agents, Things, Destinations
    """
    def __init__(self,id):
        self.id = id
        self._name = "Unnamed Element"
        self._type = Type.NOTSET
        self._timestamp = rospy.get_time()
        self._color = random_color()
        self.settable = ["name","type","color"]

    def set(self, settings, settables=None):
        if settables is None:
            settables = self.settable

        # Settings is a dictionary
        for settable in settables:
            if settable in settings.keys():
                self.__setattr__(settable,settings[settable])

    # Read/Write properties
    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        self._type = value

    @property
    def color(self):
        return self._color

    @color.setter
    def color(self, value):
        self._color = value

    # Readonly properties
    @property
    def useIcon(self):
        return True

    @property
    def timestamp(self):
        return self._timestamp
    @property
    def bgColor(self):
        return "rgb({r},{g},{b})".format(**self._color)

    @property
    def txtColor(self):
        if self.color != None:
            weights = {"r":0.2126,"g":0.7152,"b":0.0722}
            lum = 0;
            for channel in ["r","g","b"]:
              c = self.color[channel]/255.0
              if c <= 0.03928:
                  c = c/12.92
              else:
                  c = ((c+0.055)/1.055) ** 2.4
              lum += c * weights[channel]

            if lum > 0.3: # Changed from 0.179
                return 'rgb(0,0,0)'
            else:
                return 'rgb(255,255,255)'
        else:
            return 'null'

    @property
    def element_dict(self):
        return {"id":self.id,
                "name":self.name,
                "type":self.type.value,
                "bgColor":self.bgColor,
                "txtColor":self.txtColor,
                "timestamp":self.timestamp,
                "color":self.color,
                "useIcon":self.useIcon}

class Destination(Element):
    """
    Subclass of Element.
    Represents a destination.
    """
    def __init__(self,id):
        super(Destination,self).__init__(id)
        self._name = "Unnamed Destination"
        self._position = {"x":0,"y":0,"z":0}
        self._orientation = {"x":0,"y":0,"z":0}
        self._reachable = False
        self._movable = True
        self.settable.extend(["pose","position",'orientation','movable','reachable'])

    # Read/Write properties
    @property
    def type(self):
        if self.name != None and self.name != "":
            return self.name
        else:
            return "notset"

    @type.setter
    def type(self, value):
        self._type = value

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = value

    @property
    def reachable(self):
        return self._reachable

    @reachable.setter
    def reachable(self,value):
        self._reachable = value

    @property
    def movable(self):
        return self._movable

    @movable.setter
    def movable(self,value):
        self._movable = value

    @property
    def pose(self):
        return EulerPose(position=Point(**self.position),orientation=Vector3(**self.orientation))

    @pose.setter
    def pose(self, value):
        # Set from a rospy Pose type (Position [x,y,z], Orientation [x,y,z,w])
        position = {"x":value.position.x,
                    "y":value.position.y,
                    "z":value.position.z}
        (r,p,y) = tf.transformations.euler_from_quaternion([value.orientation.x,
                                                            value.orientation.y,
                                                            value.orientation.z,
                                                            value.orientation.w])
        orientation = {"x":r,
                       "y":p,
                       "z":y}
        self.set({"position":position,"orientation":orientation,'elementSetType':"destination"})

    # Readonly properties
    @property
    def useIcon(self):
        if self.name == None or self.name == "":
            return True
        else:
            return False

    @property
    def locked(self):
        return False

    @property
    def destination_dict(self):
        return {"id":self.id,
                "name":self.name,
                "type":self.type,
                "bgColor":self.bgColor,
                "txtColor":self.txtColor,
                "timestamp":self.timestamp,
                "color":self.color,
                "useIcon":self.useIcon,
                "position":self.position,
                "orientation":self.orientation,
                "reachable":self.reachable,
                "movable":self.movable,
                "locked":self.locked}

class Agent(Destination):
    """
    Subclass of Destination.
    Represents an agent.
    """
    def __init__(self, id, type=Type.HUMAN):
        super(Agent, self).__init__(id)
        self._type = type
        self._name = "Unnamed Agent"

    def set(self, settings):
        if "elementSetType" not in settings.keys():
            raise Exception("Must supply elementSetType when setting an Agent")

        settables = None
        if settings["elementSetType"] == "destination":
            settables = ["pose","position","orientation","movable","reachable"]

        print settings, '\n\n', settables

        super(Agent,self).set(settings,settables)

    # Read/Write properties
    @property
    def type(self):
        return self._type.value

    @type.setter
    def type(self, value):
        self._type = Type.fromStr(value)

    # Readonly Properties
    @property
    def useIcon(self):
        return True

    @property
    def locked(self):
        return True

    @property
    def arms(self):
        if self._type == Type.HUMAN:
            return ['human']
        elif self._type == Type.ROBOT:
            return ["manipulator"]
        else:
            return []

    @property
    def agent_dict(self):
        return {"id":self.id,
                "elementSetType": "agent",
                "name":self.name,
                "type":self.type,
                "bgColor":self.bgColor,
                "txtColor":self.txtColor,
                "timestamp":self.timestamp,
                "color":self.color,
                "useIcon":self.useIcon,
                "arms":self.arms}

    @property
    def destination_dict(self):
        return {"id":self.id,
                "elementSetType": "destination",
                "name":"Initial:"+self.name,
                "type":"loc-"+self.type,
                "bgColor":self.bgColor,
                "txtColor":self.txtColor,
                "timestamp":self.timestamp,
                "color":self.color,
                "useIcon":self.useIcon,
                "position":self.position,
                "orientation":self.orientation,
                "locked":self.locked,
                "reachable":self.reachable,
                "movable":self.movable}

class Thing(Destination):
    """
    Subclass of Element/Destination.
    Represents a thing.
    """
    def __init__(self, id, type=Type.CUBE):
        super(Thing, self).__init__(id)
        self._type = type
        self._size = 0.1
        self._name = "Unnamed Thing"
        self._position = {"x":random.uniform(-1, 1),"y":random.uniform(0,1),"z":0}
        self.settable.extend(["size","callback"])

    def set(self, settings):
        if "elementSetType" not in settings.keys():
            raise Exception("Must supply elementSetType when setting a Thing")

        settables = None
        if settings["elementSetType"] == "destination":
            settables = ["pose","position","orientation",'movable','reachable']

        super(Thing,self).set(settings,settables)

    # Read/Write properties
    @property
    def type(self):
        return self._type.value

    @type.setter
    def type(self, value):
        self._type = Type.fromStr(value)

    @property
    def size(self):
        return self._size

    @size.setter
    def size(self, value):
        self._size = value

    @property
    def color(self):
        return self._color

    @color.setter
    def color(self, value):
        self._color = value

    # Readonly properties
    @property
    def useIcon(self):
        return True

    @property
    def locked(self):
        return True

    @property
    def destination_dict(self):
        return {"id":self.id,
                "elementSetType": "destination",
                "name":"Initial:"+self.name,
                "type":"loc-"+self.type,
                "bgColor":self.bgColor,
                "txtColor":self.txtColor,
                "timestamp":self.timestamp,
                "color":self.color,
                "useIcon":self.useIcon,
                "position":self.position,
                "orientation":self.orientation,
                "locked":self.locked,
                "reachable":self.reachable,
                "movable":self.movable}

    @property
    def thing_dict(self):
        return {"id":self.id,
                "elementSetType": "thing",
                "name":self.name,
                "size":self.size,
                "type":self.type,
                "bgColor":self.bgColor,
                "txtColor":self.txtColor,
                "timestamp":self.timestamp,
                "color":self.color,
                "useIcon":self.useIcon,
                "position":self.position,
                "orientation":self.orientation}
