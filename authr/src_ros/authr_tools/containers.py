#! /usr/bin/env python
import rospy
import random

class Task(object):
    """
    Task container object
    """
    def __init__(self, id, name="New Task", therbligs=[], repeat=0, timestamp=None, **kwargs):
        if timestamp == None:
            self.timestamp = rospy.get_time()
        else:
            self.timestamp = timestamp
        self.id = id
        self.name = name
        self.therbligs = therbligs
        self.repeat = repeat
        self.settable = ["name","therbligs","repeat"]

    @property
    def task_dict(self):
        return {"id":self.id,
                "timestamp":self.timestamp,
                "name":self.name,
                "therbligs":self.therbligs,
                "repeat":self.repeat}

    def set(self, settings):
        # Settings is a dictionary
        for settable in self.settable:
            if settable in settings.keys():
                self.__setattr__(settable,settings[settable])

class Macro(object):
    """
    Macro container object
    """
    def __init__(self, id, name="New Macro", therbligs=[], timestamp=None, **kwargs):
        if timestamp == None:
            self.timestamp = rospy.get_time()
        else:
            self.timestamp = timestamp
        self.id = id
        self.name = name
        self.therbligs = therbligs
        self.settable = ["name","therbligs"]

    @property
    def macro_dict(self):
        return {"id":self.id,
                "timestamp":self.timestamp,
                "name":self.name,
                "therbligs":self.therbligs}

    def set(self, settings):
        # Settings is a dictionary
        for settable in self.settable:
            if settable in settings.keys():
                self.__setattr__(settable,settings[settable])
