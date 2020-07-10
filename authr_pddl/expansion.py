#!/usr/bin/env python
import json
import sys
import copy
from enum import Enum
# Removable for full
# import pandas
# import numpy as np
class TherbligType(Enum):
    TRANSPORT_EMPTY=0
    GRASP=1
    TRANSPORT_LOADED=2
    RELEASE_LOAD=3
    HOLD=4
    REST=5
    @staticmethod
    def fromStr(string):
        s = string.lower()
        if s == 'transport_empty':
            return TherbligType.TRANSPORT_EMPTY
        elif s == 'grasp':
            return TherbligType.GRASP
        elif s == 'transport_loaded':
            return TherbligType.TRANSPORT_LOADED
        elif s == 'release_load':
            return TherbligType.RELEASE_LOAD
        elif s == 'hold':
            return TherbligType.HOLD
        elif s == 'position':
            return TherbligType.POSITION
        elif s == 'rest':
            return TherbligType.REST
        elif s == 'preposition':
            return TherbligType.PREPOSITION
        else:
            raise ValueError('Enum supplied ({0}) does not exist'.format(s))
    @staticmethod
    def names():
        return [e.name for e in list(TherbligType)]
def convert(input):
    if isinstance(input, dict):
        return {convert(key): convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input
class Expander(object):
    def __init__(self,plan,trace):
        # print(trace)
        self.plan = plan
        # Get some useful data about the plan
        self.therblig_ids = []
        self.therbligs = {}
        self.agents = self.plan["agents"].keys()
        self.things = self.plan["things"].keys()
        self.destinations = self.plan["destinations"].keys()
        self.agent_count = len(self.agents)
        self.thing_count = len(self.things)
        self.states = []
        self.sorted = {} # a dictionary with agend ids as keys and arrays of dictionaries as values
        # for task_key in self.plan["task_keys"]:
        #     for repeat in range(self.plan["tasks"][task_key]["repeat"] + 1):
        #         for therblig in self.plan["tasks"][task_key]["therbligs"]:
        #             id = therblig+"_{0}".format(repeat)
        #             self.therblig_ids.append(id)
        #             therblig_obj = self.plan["therbligs"][therblig]
        #             therblig_obj["orig_id"] = therblig
        #             therblig_obj['repeat'] = repeat
        #             therblig_obj["is_repeated"] = self.plan["tasks"][task_key]["repeat"] > 0
        #             self.therbligs[id] = therblig_obj
        # self.therblig_count = len(self.therblig_ids)
        therbligs = copy.deepcopy(trace.therbligs())
        previous_therblig = None # remove grasp release pairs
        to_remove = []
        for therblig in therbligs:
            if therblig["type"] == "grasp" and previous_therblig is not None and previous_therblig["type"] == "release_load":
                    to_remove.append(previous_therblig)
                    to_remove.append(therblig)
                    previous_therblig = None
            elif therblig["type"] == "release_load" and previous_therblig is not None and previous_therblig["type"] == "grasp":
                    to_remove.append(previous_therblig)
                    to_remove.append(therblig)
                    previous_therblig = None
            else:
                previous_therblig = therblig
        for therblig in to_remove:
            therbligs.remove(therblig)

        for therblig in therbligs:
            self.therblig_ids.append(therblig["eid"])
            therblig_obj = self.plan["therbligs"][therblig["id"]]
            self.therbligs[therblig["eid"]] = therblig
        self.therblig_count = len(self.therblig_ids)
    def calculate(self):
        # print("calculate")
        # see if the therblig is valid from the bottom up based on state times (the state's therblig start time + duration)
        # put the therblig as early as possible without overlapping an invalid state time
        # stop moving up if an already added therblig will become invalid
        for agent_key in self.agents:
            self.sorted[agent_key] = []
        agent_destinations = {}
        thing_destinations = {}
        for agent_key in self.agents:
            agent_destinations[agent_key] = agent_key
        for thing_key in self.things:
            thing_destinations[thing_key] = thing_key
        self.states.append(State(0, agent_destinations, thing_destinations))
        numthers = 1
        therblig_queue = []
        for therblig_key in self.therblig_ids:
            valid_queued_therblig_index = None
            for j, queued_therblig in enumerate(therblig_queue):
                queued_therblig_valid_states = []
                for i in range(len(self.states)-1, -1, -1): # loop through states backwards
                    if self.states[i].isValid(queued_therblig, self.states, self.sorted) and self.states[i].isValidReverse(queued_therblig, self.states, self.plan):
                        queued_therblig_valid_states.append(self.states[i])
                queued_therblig_valid_states.reverse()

                duration = queued_therblig["duration"]
                state = copy.deepcopy(getBestState(self.states, queued_therblig_valid_states, queued_therblig, duration))
                if state is None: # if the therblig cannot be placed
                    pass
                else:
                    valid_queued_therblig_index = j
                    break
            if valid_queued_therblig_index is not None:
                valid_queued_therblig = therblig_queue.pop(valid_queued_therblig_index)
                self.sorted[valid_queued_therblig["parameters"]["agent"]].append({"therblig": valid_queued_therblig, "start_time": state.time, "duration": valid_queued_therblig["duration"]})
                state.update(valid_queued_therblig, self.sorted, self.plan) # Todo don't make new state and update existing state if same time
                self.states.append(state)

            # print("therblig {}".format(numthers))
            numthers += 1
            therblig = self.therbligs[therblig_key]
            # print("\nTHERBLIG:")
            # print(therblig["type"])
            # print(therblig["parameters"]["agent"])
            # try:
            #     print(therblig["parameters"]["destination"])
            # except:
            #     pass
            if self.therblig_ids[0] == therblig_key: # add the first therblig at t = 0
                state = copy.deepcopy(self.states[0])
                self.sorted[therblig["parameters"]["agent"]].append({"therblig": therblig, "start_time": 0, "duration": therblig["duration"]})
                state.update(therblig, self.sorted, self.plan)
                self.states.append(state)
            else: # for all therbligs other than the first
                validStates = [] # priority will be at beginning
                for i in range(len(self.states)-1, -1, -1): # loop through states backwards
                    # print("isValid")
                    # print(self.states[i].time)
                    # print(self.states[i].isValid(therblig, self.states, self.sorted))
                    # print("isValidReverse")
                    # print(self.states[i].time)
                    # print(self.states[i].isValidReverse(therblig, self.states, self.plan))
                    if self.states[i].isValid(therblig, self.states, self.sorted) and self.states[i].isValidReverse(therblig, self.states, self.plan):
                        validStates.append(self.states[i])
                validStates.reverse()
                # print("VALID STATES: ")
                # for validState in validStates:
                #     print(validState.time)
                duration = therblig["duration"]
                state = copy.deepcopy(getBestState(self.states, validStates, therblig, duration))
                if state is None: # if the therblig cannot be placed
                    therblig_queue.append(therblig)
                    continue
                # print("BEST STATE: ")
                # print(state.time)
                self.sorted[therblig["parameters"]["agent"]].append({"therblig": therblig, "start_time": state.time, "duration": therblig["duration"]})
                state.update(therblig, self.sorted, self.plan) # Todo don't make new state and update existing state if same time
                self.states.append(state)
        return self.sorted
def getBestState(states, validStates, therblig, duration):
    # print(len(validStates))
    bestState = None
    for validState in validStates:
        for checkState in states:
            # print("CHECKSTATE TIME:")
            # print(checkState.time)
            if checkState.time > validState.time and checkState.time < validState.time+duration:
                if not checkState in validStates:
                    break
        if bestState == None or bestState.time > validState.time:
            bestState = validState
    return bestState

class State(object): # there will be a state for the end of each therblig and t = 0. States with same time will be combined(not currently happening)
    def __init__(self, time, agent_destinations, thing_destinations):
        self._time = time
        self._agent_destinations = agent_destinations
        self._thing_destinations = thing_destinations
    def isValid(self, therblig, states, sorted): # true if the given therblig is allowed to cross this state, false otherwise
    # check that therblig is allowed based on set therbligs
        index = states.index(self)
        cantChange = [] # all things and agents that this therblig is not allowed to change the destinations of. If someting is
                        # changed that is in this list, the therbilg is invalid at this state
        for data in sorted[therblig["parameters"]["agent"]]:
            if self.time < data["start_time"] + data["duration"]:
                return False
        if "agent_destination" in therblig["requires"]:
            if not therblig["parameters"]["destination"] == states[index].agent_destinations[therblig["parameters"]["agent"]]:
                return False
        if "thing_destination" in therblig["requires"]:
            if not therblig["parameters"]["destination"] == states[index].thing_destinations[therblig["parameters"]["thing"]]:
                return False
        if not index == 0: # this stuff won't work if index is 0, becasue there is no states[index-1]
            for key in states[index].agent_destinations:
                if not states[index].agent_destinations[key] == states[index-1].agent_destinations[key]:
                    cantChange.append(key)
            for key in states[index].thing_destinations:
                if not states[index].thing_destinations[key] == states[index-1].thing_destinations[key]:
                    cantChange.append(key)
            if "agent_destination" in therblig["sets"]:
                if not therblig["parameters"]["agent"] in states[index].agent_destinations:
                    return False
            if "thing_destination" in therblig["sets"]:
                if not therblig["parameters"]["thing"] in states[index].thing_destinations:
                    return False
        return True
    def isValidReverse(self, therblig, states, plan): # true if adding the therblig at or above this state won't mess up set therbligs
    # check that putting therblig here won't mess up set therbligs
        index = states.index(self)
        duration = therblig["duration"]
        earliestAfter = None # the earliest state after the therblig if starting at self. could be immediately after
        for state in states:
            if state.time >= self.time + duration and (earliestAfter == None or state.time < earliestAfter.time):
                earliestAfter = state
        if earliestAfter is not None:
            if "agent_destination" in therblig["sets"]:
                for state in states:
                    if state.time <= earliestAfter.time and state.time >= self.time: # loop through all states that overlap therblig at self
                        for agent in plan["agents"]:
                            if "destination" in therblig["parameters"] and therblig["parameters"]["destination"] == state.agent_destinations[agent]:
                                return False
        else:
            if "agent_destination" in therblig["sets"]:
                for state in states:
                    if state.time >= self.time:
                        for agent in plan["agents"]:
                            if "destination" in therblig["parameters"] and therblig["parameters"]["destination"] == state.agent_destinations[agent]:
                                return False
        return True
    @property
    def time(self):
        return self._time
    @property
    def agent_destinations(self):
        return self._agent_destinations
    @property
    def thing_destinations(self):
        return self._thing_destinations
    def update(self, therblig, sorted, plan):
        if "agent_destination" in therblig["sets"]:
            self._agent_destinations[therblig["parameters"]["agent"]] = therblig["sets"]["agent_destination"]
        if "thing_destination" in therblig["sets"]:
            self._thing_destinations[therblig["parameters"]["thing"]] = therblig["sets"]["thing_destination"]
        self._time += therblig["duration"]
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No JSON File Provided!")
    else:
        file = sys.argv[1]
        with open(file, "r") as read_file:
            data = convert(json.load(read_file))
        print("######### Calculating Duration #########")
        e = Expander(data)
        expanded = e.calculate()
        # print(expanded)
