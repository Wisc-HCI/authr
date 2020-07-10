#! /usr/bin/env python
import random
import rospy
from enum import Enum
from abc import ABCMeta, abstractmethod

class StateSetting(Enum):
    FALSE=0
    TRUE=1
    MATCH=2


class Therblig(object):
    """
    Therblig object
    """
    __metaclass__ = ABCMeta

    def __init__(self, id, data={"type":             "default",
                             "parameter_fields": [],
                             "allowed":          [],
                             "sets":             {},
                             "requires":         {},
                             "tooltip":             'Default Therblig. There is no help information available.',
                             "physical":         False,
                             "cognitive":        False,
                             "settable":         ["parameters","duration","cost"]}):

        self.id = id
        self.data                = data
        self.parameters_field    = {field:None for field in self.data["parameters"]}
        self.parameters_field["agent"] = "OPTIMIZE_DIRECTIVE"

        # Duration and Cost have lookups with each agent as keys, unless universal
        self.universal           = False
        self._duration_lookup    = {}
        self._cost_lookup        = {}

        # Provide a location to store external errors generated through verification
        self.verification_errors = {"agent_destination_set":[],"agent_destination_require":[],
                                    "thing_destination_set":[],"thing_destination_require":[],
                                    "agent_gripping_set":[],"agent_gripping_require":[],
                                    "agent_thing_require":[]}

        self.agent_type_lookup = {}

    def set(self, settings):
        # Settings is a dictionary
        to_set = [settable for settable in self.settable if settable in settings.keys()]
        for settable in to_set:
            rospy.loginfo("Setting "+settable+".")
            if settable in self.parameters_field.keys():
                self.parameters_field[settable] = settings[settable]
            else:
                if settable == "parameters":
                    for value in settings["parameters"].keys():
                        if value in self.settable:
                            self.parameters_field[value] = settings["parameters"][value]
                else:
                    self.__setattr__(settable,settings[settable])


    @property
    def tooltip(self):
        return self.data["tooltip"] + "\nParameters: "    + str(self.data["parameters"]) +\
                                   "\nAllowed: "       + str(self.data["allowed"]) +\
                                   "\nPreconditions: " + str(self.data["preconditions"]) +\
                                   "\nCostraints: "    + str(self.data["constraints"])

    @property
    def duration(self):
        return self._duration_lookup

    @duration.setter
    def duration(self, value):
        if self.universal and value.keys() == ["Any"]:
            self._duration_lookup = value
        elif not self.universal:
            self._duration_lookup = value

    @property
    def cost(self):
        return self._cost_lookup

    @cost.setter
    def cost(self, value):
        self._cost_lookup = value

    @property
    def allowed(self):
        return self.data["allowed"]

    @property
    def sets(self):
        set_dict = {}
        for key, state_setting in self.data["sets"].items():
            if key == "grasping":
                set_dict["grasping"] = bool(state_setting.value)
            elif key == "agent_destination" and state_setting == StateSetting.MATCH:
                set_dict["agent_destination"] = self.parameters_field["destination"]
            elif key == "thing_destination" and state_setting == StateSetting.MATCH:
                set_dict["thing_destination"] = self.parameters_field["destination"]
            elif key == "agent_thing" and state_setting == StateSetting.MATCH:
                set_dict["agent_thing"] = "MATCH"
        return set_dict

    @property
    def requires(self):
        req_dict = {}
        for key, state_setting in self.data["requires"].items():
            if key == "grasping":
                req_dict["grasping"] = bool(state_setting.value)
            elif key == "agent_destination" and state_setting == StateSetting.MATCH:
                req_dict["agent_destination"] = self.parameters_field["destination"]
            elif key == "thing_destination" and state_setting == StateSetting.MATCH:
                req_dict["thing_destination"] = self.parameters_field["destination"]
            elif key == "agent_thing" and state_setting == StateSetting.MATCH:
                req_dict["agent_thing"] = "MATCH"
        return req_dict

    @property
    def constraints(self):
        return self.data["constraints"]

    @property
    def physical(self):
        return self.data["physical"]

    @property
    def cognitive(self):
        return self.data["cognitive"]

    @property
    def settable(self):
        return self.data["settable"]

    @property
    def parameters(self):
        return self.data["parameters"]

    @property
    def type(self):
        return self.data["type"]

    @property
    def showDuration(self):
        return self.parameters_field["agent"] == "OPTIMIZE_DIRECTIVE" or self.parameters_field["agent"] == None or self.agent_type_lookup[self.parameters_field["agent"]] == "human" or self.universal

    @property
    def showCost(self):
        return self.parameters_field["agent"] == "OPTIMIZE_DIRECTIVE"

    def _parameter_check(self):
        error_dict = {}
        for parameter, value in self.parameters_field.items():
            if parameter == "arm":
                if self.parameters_field["agent"] != None and self.parameters_field["agent"] != "OPTIMIZE_DIRECTIVE" and value == None:
                    error_dict[parameter] = ["You must specify the arm of the agent"]
                else:
                    error_dict[parameter] = []
            elif value == None:
                error_dict[parameter] = ["Value not set."]
            else:
                error_dict[parameter] = []
        return error_dict

    def _duration_check(self):
        error = False

        if len(self.duration.keys()) == 0:
            return {"duration":["Must add agents to assign durations."]}
        if self.universal and self.duration["Any"] == None:
            return {"duration":["Explicit duration not assigned."]}
        elif not self.universal and self.parameters_field["agent"] == "OPTIMIZE_DIRECTIVE":
            for agent, value in self._duration_lookup.items():
                if value == None and self.agent_type_lookup[agent] == 'human':
                    return {"duration":["One or more agents do not have durations defined."]}
        elif not self.universal and self.parameters_field["agent"] != None and self.agent_type_lookup[self.parameters_field["agent"]] == "human":
            if self._duration_lookup[self.parameters_field["agent"]] == None:
                return {"duration":["Assigning human requires durations to be defined."]}
        return {"duration":[]}

    def _cost_check(self):
        error = False
        if self.parameters_field["agent"] == "OPTIMIZE_DIRECTIVE":
            if len(self.cost.keys()) == 0:
                return {"cost":["Must add agents to assign costs."]}
            for parameter, value in self._cost_lookup.items():
                if value == None:
                    error = True
            if error:
                return {"cost":["One or more agents do not have costs defined."]}
            else:
                return {"cost":[]}
        else:
            return {"cost":[]}

    @property
    def errors(self):
        error_dict = {}
        error_dict.update(self._parameter_check())
        error_dict.update(self._duration_check())
        error_dict.update(self._cost_check())
        error_dict.update(self.verification_errors)
        count = len([error_dict[key] for key in error_dict.keys() if len(error_dict[key]) > 0])
        if count > 0:
            error_dict["general"] = str(count)
        else:
            error_dict['general'] = None
        return error_dict

    @property
    def therblig_dict(self):
        return {"id":self.id,
            "type":self.type,
            "parameters":self.parameters_field,
            "physical":self.physical,
            "cognitive":self.cognitive,
            "allowed":self.allowed,
            "sets":self.sets,
            "requires":self.requires,
            "duration":self.duration,
            "cost":self.cost,
            "tooltip":self.data["tooltip"],
            "errors":self.errors,
            "showDuration":self.showDuration,
            "showCost":self.showCost,
            "settable":self.settable
        }

    @property
    def primitive_dict(self):
        return {
            "type":self.type,
            "parameters":self.data["parameters"],
            "physical":self.physical,
            "cognitive":self.cognitive,
            "allowed":self.allowed,
            "sets":self.sets,
            "requires":self.requires,
            "tooltip":self.data["tooltip"]
        }

    @classmethod
    def from_dict(cls, therbligChild, data):
        # therblig child must be a child class of this
        # data must be formatted according to the therblig_dict property

        #key reference of data instead of direct passing to produce key-errors here
        therblig = therbligChild(data={"type":             data["type"],
                                       "parameters":       data["parameters"],
                                       "allowed":          data["allowed"],
                                       "sets":             data["sets"],
                                       "requires":         data["requires"],
                                       "tooltip":          data["tooltip"],
                                       "physical":         data["physical"],
                                       "cognitive":        data["cognitive"],
                                       "settable":         data["settable"]})
        therblig.duration = data["duration"]
        therblig.cost = data["cost"]
        return therblig


class TransportEmpty(Therblig):
    """
    Transport an empty gripper or hand to a specified location.
    """
    def __init__(self,id):
        super(TransportEmpty,self).__init__(id,
            {"type":             "transport_empty",
             "parameters": ["agent","destination","arm"],
             "allowed":          ["transport_empty","rest","grasp"],
             "sets":             {"agent_destination":StateSetting.MATCH},
             "requires":         {"grasping":StateSetting.FALSE},
             "tooltip":          "Transport an empty gripper or hand to a specified location.",
             "physical":         True,
             "cognitive":        False,
             "settable":         ["agent","destination","arm","parameters","duration","cost"]}
        )


class TransportLoaded(Therblig):
    """
    Transport a loaded gripper or hand to a specified location.
    """
    def __init__(self,id):
        super(TransportLoaded,self).__init__(id,
            {"type":             "transport_loaded",
             "parameters": ["agent","thing","destination","arm","effort"],
             "allowed":          ["transport_loaded","hold","release_load"],
             "sets":             {"agent_destination":StateSetting.MATCH,"thing_destination":StateSetting.MATCH},
             "requires":         {"grasping":StateSetting.TRUE,"agent_thing":StateSetting.MATCH},
             "tooltip":             "Transport a loaded gripper or hand to a specified location.",
             "physical":         True,
             "cognitive":        False,
             "settable":         ["agent","thing","destination","arm","effort","parameters","duration","cost"]}
        )


class Grasp(Therblig):
    """
    Grasp a thing with a specified effort.
    """
    def __init__(self,id):
        super(Grasp,self).__init__(id,
            {"type":             "grasp",
             "parameters": ["agent","thing","arm","effort"],
             "allowed":          ["release_load","transport_loaded","hold"],
             "sets":             {"grasping":StateSetting.TRUE},
             "requires":         {"grasping":StateSetting.FALSE,"agent_thing":StateSetting.MATCH},
             "tooltip":             "Grasp a thing with a specified effort.",
             "physical":         True,
             "cognitive":        False,
             "settable":         ["agent","thing","arm","effort","parameters","duration","cost"]}
        )


class ReleaseLoad(Therblig):
    """
    Release hold of a gripper or hand.
    """
    def __init__(self,id):
        super(ReleaseLoad,self).__init__(id,
            {"type":             "release_load",
             "parameters": ["agent","thing","arm"],
             "allowed":          ["grasp","transport_empty","rest"],
             "sets":             {"grasping":StateSetting.FALSE},
             "requires":         {"grasping":StateSetting.TRUE,"agent_thing":StateSetting.MATCH},
             "tooltip":             "Release hold of a gripper or hand.",
             "physical":         True,
             "cognitive":        False,
             "settable":         ["agent","thing","arm","parameters","duration","cost"]}
        )


class Hold(Therblig):
    """
    Hold a thing for a specified duration.
    """
    def __init__(self,id):
        super(Hold,self).__init__(id,
            {"type":             "hold",
             "parameters": ["agent","thing","arm","effort"],
             "allowed":          ["hold","transport_loaded","release_load"],
             "sets":             {},
             "requires":         {"grasping":StateSetting.TRUE,"agent_thing":StateSetting.MATCH},
             "tooltip":             "Hold a thing for a specified duration.",
             "physical":         True,
             "cognitive":        False,
             "settable":         ["agent","thing","arm","effort","parameters","duration","cost"]}
        )
        self.universal = True
        self._duration_lookup = {"Any":None}
        rospy.loginfo(self.therblig_dict)


class Rest(Therblig):
    """
    Wait for a specified duration.
    """
    def __init__(self,id):
        super(Rest,self).__init__(id,
            {"type":             "rest",
             "parameters": ["agent","destination","arm"],
             "allowed":          ["rest","transport_empty","grasp"],
             "sets":             {},
             "requires":         {"grasping":StateSetting.FALSE,"agent_destination":StateSetting.MATCH},
             "tooltip":             "Wait for a specified duration.",
             "physical":         True,
             "cognitive":        False,
             "settable":         ["agent","destination","arm","parameters","duration","cost"]}
        )
        self.universal = True
        self._duration_lookup = {"Any":None}
        rospy.loginfo(self.therblig_dict)


class Position(Therblig):
    """
    Position an arm or hand in a specific orientation.
    """
    def __init__(self,id):
        super(Position,self).__init__(id,
            {"type":             "position",
             "parameters": ["agent","thing","arm"],
             "allowed":          [],
             "preconditions":    [],
             "constraints":      {},
             "tooltip":             "Position an arm or hand in a specific orientation.",
             "physical":         True,
             "cognitive":        False,
             "settable":         ["agent","thing","arm","parameters","duration","cost"]}
        )


class PrePosition(Therblig):
    """
    Pre-position an arm or hand.
    """
    def __init__(self,id):
        super(PrePosition,self).__init__(id,
            {"type":             "preposition",
             "parameters": ["agent","thing","arm"],
             "allowed":          [],
             "preconditions":    [],
             "constraints":      {},
             "tooltip":             "Pre-position an arm or hand in a specific orientation.",
             "physical":         True,
             "cognitive":        False,
             "settable":         ["agent","thing","arm","parameters","duration","cost"]}
        )


TherbligLookup = {
    "transport_empty": TransportEmpty,
    "grasp":           Grasp,
    "transport_loaded":TransportLoaded,
    "release_load":    ReleaseLoad,
    "hold":            Hold,
    # "position":        Position,
    "rest":            Rest,
    # "preposition":     PrePosition
    # TODO: Add others later
}
