#!/usr/bin/env python
import z3
import json
import sys
from enum import Enum
from copy import deepcopy

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

class Sequence(object):
    def __init__(self, context, agents, therbligs, things, destinations):
        self.agents = agents
        self.therbligs = ["INITIAL"]+therbligs+["FINAL"]
        self.things = things
        self.destinations = destinations

        # Create the Therblig/Agent Action Space
        self.action_data = {}
        for i in range(len(self.therbligs)):
            for j in range(len(TherbligType)):
                for k in range(len(self.agents)):
                    self.action_data["{0}_{1}_{2}".format(
                                     self.therbligs[i],
                                     [tt.name for tt in TherbligType][j],
                                     self.agents[k])] = z3.Bool("{0}_{1}_{2}".format(i,j,k),ctx=context)

        # Create the Thing/Location State Space
        self.thing_destination_data = {}
        for i in range(len(self.therbligs)):
            for j in range(len(self.destinations)):
                for k in range(len(self.things)):
                    self.thing_destination_data["tds_{0}_{1}_{2}".format(
                                                self.therbligs[i],
                                                self.destinations[j],
                                                self.things[k])] = z3.Bool("tds_{0}_{1}_{2}".format(i,j,k),ctx=context)

        self.agent_destination_data = {}
        for i in range(len(self.therbligs)):
            for j in range(len(self.destinations)):
                for k in range(len(self.agents)):
                    self.agent_destination_data["ads_{0}_{1}_{2}".format(
                                                self.therbligs[i],
                                                self.destinations[j],
                                                self.agents[k])] = z3.Bool("ads_{0}_{1}_{2}".format(i,j,k),ctx=context)

        self.agent_gripping_data = {}
        for i in range(len(self.therbligs)):
            for j in range(len(self.agents)):
                    self.agent_gripping_data["ags_{0}_{1}".format(
                                             self.therbligs[i],
                                             self.agents[j])] = z3.Bool("ags_{0}_{1}".format(i,j),ctx=context)


    def actions(self, indices=[], therbligs=[], agents=[]):
        responses = []
        for index in indices:
            for therblig in therbligs:
                for agent in agents:
                    responses.append(self.action_data["{0}_{1}_{2}".format(index,therblig,agent)])
        return responses

    def thing_destination(self, indices=[], things=[], destinations=[]):
        responses = []
        for index in indices:
            for thing in things:
                for destination in destinations:
                    responses.append(self.thing_destination_data["tds_{0}_{1}_{2}".format(index,destination,thing)])
        return responses

    def agent_destination(self, indices=[], agents=[], destinations=[]):
        responses = []
        for index in indices:
            for agent in agents:
                for destination in destinations:
                    responses.append(self.agent_destination_data["ads_{0}_{1}_{2}".format(index,destination,agent)])
        return responses

    def agent_thing(self, indices=[], agents=[], things=[], destinations=[]):
        agent_settings = self.agent_destination(indices=indices,agents=agents,destinations=destinations)
        thing_settings = self.thing_destination(indices=indices,things=things,destinations=destinations)
        responses = []
        for agent_setting in agent_settings:
            for thing_setting in thing_settings:
                responses.append(z3.And(agent_setting == z3.BoolVal(True,ctx=context),
                                        thing_setting == z3.BoolVal(True,ctx=context)))
        return

    def agent_gripping(self, indices=[], agents=[]):
        responses = []
        for index in indices:
            for agent in agents:
                responses.append(self.agent_gripping_data["ags_{0}_{1}".format(index,agent)])
        return responses

    def evaluated(self,model,plan):
        evaluated_actions = {}
        cols = ["INITIAL"]
        for i,t in enumerate(self.therbligs):
            if t != "INITIAL" and t != "FINAL":
                name = i
                cols.append(name)
            else:
                name = t
            data = {}
            for tt in TherbligType.names():
                allocations = []
                for a in self.agents:
                    v = model.evaluate(self.actions(indices=[t],therbligs=[tt],agents=[a])[0])
                    if v == True:
                        allocations.append(plan["agents"][a]["name"])
                data[tt]=",".join(allocations)
            evaluated_actions[name]=data
        cols.append("FINAL")
        ##print(evaluated_actions)
        #return pandas.DataFrame(evaluated_actions,index=TherbligType.names(),columns=cols)
        evaluated_actions = pandas.DataFrame(evaluated_actions,index=TherbligType.names(),columns=cols)

        evaluated_agent_destinations = {}
        for i,t in enumerate(self.therbligs):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
            else:
                name = t
            for a in self.agents:
                locations = []
                for d in self.destinations:
                    v = model.evaluate(self.agent_destination(indices=[t],agents=[a],destinations=[d])[0])
                    if v == True:
                        locations.append(plan["destinations"][d]["name"])
                if len(locations) == 0:
                    locations.append("NONE")
                data[plan["agents"][a]["name"]] = ",".join(locations)
            evaluated_agent_destinations[name] = data
        evaluated_agent_destinations = pandas.DataFrame(evaluated_agent_destinations,columns=cols)

        evaluated_thing_destinations = {}
        for i,t in enumerate(self.therbligs):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
            else:
                name = t
            for a in self.things:
                locations = []
                for d in self.destinations:
                    v = model.evaluate(self.thing_destination(indices=[t],things=[a],destinations=[d])[0])
                    if v == True:
                        locations.append(plan["destinations"][d]["name"])
                if len(locations) == 0:
                    locations.append("NONE")
                data[plan["things"][a]["name"]] = ",".join(locations)
            evaluated_thing_destinations[name] = data
        evaluated_thing_destinations = pandas.DataFrame(evaluated_thing_destinations,columns=cols)

        evaluated_grasping = {}
        for i,t in enumerate(self.therbligs):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
            else:
                name = t
            for a in self.agents:
                data[plan["agents"][a]["name"]] = model.evaluate(self.agent_gripping(indices=[t],agents=[a])[0])
                ##print(model.evaluate(self.actions(indices=[t],agents=[a])[0]))
            evaluated_grasping[name] = data
        evaluated_grasping = pandas.DataFrame(evaluated_grasping,columns=cols)

        return evaluated_actions, evaluated_agent_destinations, evaluated_thing_destinations, evaluated_grasping


class Verifier(object):
    def __init__(self,plan):
        self.plan = plan
        # Get some useful data about the plan
        self.therblig_ids = []
        self.therbligs = {}
        self.agents = self.plan["agents"].keys()
        self.things = self.plan["things"].keys()
        self.destinations = self.plan["destinations"].keys()
        self.agent_count = len(self.agents)
        self.thing_count = len(self.things)

        for task_key in self.plan["task_keys"]:
            for repeat in range(self.plan["tasks"][task_key]["repeat"] + 1):
                for therblig in self.plan["tasks"][task_key]["therbligs"]:
                    repeat_val = repeat + 1
                    eid = therblig+"_{0}".format(repeat)
                    self.therblig_ids.append(eid)
                    therblig_obj = deepcopy(self.plan["therbligs"][therblig])
                    if "id" not in therblig_obj.keys():
                        therblig_obj["id"] = therblig
                    therblig_obj["eid"] = eid
                    therblig_obj['repeat'] = repeat_val
                    therblig_obj["is_repeated"] = self.plan["tasks"][task_key]["repeat"] > 0
                    self.therbligs[eid] = therblig_obj
        self.therblig_count = len(self.therblig_ids)

    def check(self):
        """
        Check can be called to verify if there are any errors with the plan.
        Returns error count and the errors themselves, indexed by therblig id.
        """
        z3.set_option(unsat_core=True)
        #z3.set_option(timeout=500)
        errors = {self.therbligs[therblig]["id"]:{"agent_destination_set":[],"agent_destination_require":[],
                          "thing_destination_set":[],"thing_destination_require":[],
                          "agent_gripping_set":[],"agent_gripping_require":[],
                          "agent_thing_require":[]} for therblig in self.therblig_ids}
        error_count = 0
        context = z3.Context()
        solver = z3.Solver(ctx=context)

        # Define the sequence, which is a therblig x type x agent grid
        seq = Sequence(context,self.agents,self.therblig_ids,self.things,self.destinations)

        # Define Basic Constraints
        solver.push()

        # Constrain all values in relevant spaces to be 0 or 1.
        # for setting in seq.actions(indices=self.therblig_ids,therbligs=TherbligType.names(),agents=self.agents):
        #     solver.add(setting >= z3.BoolVal(False,ctx=context))
        #     solver.add(setting <= z3.BoolVal(True,ctx=context))
        # for setting in seq.thing_destination(indices=self.therblig_ids,things=self.things,destinations=self.destinations):
        #     solver.add(setting >= z3.BoolVal(False,ctx=context))
        #     solver.add(setting <= z3.BoolVal(True,ctx=context))
        # for setting in seq.agent_destination(indices=self.therblig_ids,agents=self.agents,destinations=self.destinations):
        #     solver.add(setting >= z3.BoolVal(False,ctx=context))
        #     solver.add(setting <= z3.BoolVal(True,ctx=context))

        # Constrain agents and things to only appear in a single location at a given time
        for therblig in ["INITIAL"]+self.therblig_ids:
            for agent in self.agents:
                solver.add(z3.PbEq([(o,1) for o in seq.agent_destination(indices=[therblig],agents=[agent],destinations=self.destinations)],1,ctx=context))
            for thing in self.things:
                solver.add(z3.PbEq([(o,1) for o in seq.thing_destination(indices=[therblig],things=[thing],destinations=self.destinations)],1,ctx=context))

        # Prevent multiple things, agents from occupying the same locations
        for therblig in ["INITIAL"]+self.therblig_ids:
            for destination in self.destinations:
                if len(self.agents) > 0:
                    solver.add(z3.PbLe([(o,1) for o in seq.agent_destination(indices=[therblig],agents=self.agents,destinations=[destination])],1))
                if len(self.things) > 0:
                    solver.add(z3.PbLe([(o,1) for o in seq.thing_destination(indices=[therblig],things=self.things,destinations=[destination])],1))

        # Set that therbligs occur at the given index
        # for therblig in self.therblig_ids:
        #     solver.add(z3.PbEq([(o,1) for o in seq.actions(indices=[therblig],
        #                                                    therbligs=[TherbligType.fromStr(self.therbligs[therblig]["type"]).name],
        #                                                    agents=self.agents)],1,ctx=context))

        # Constrain initial positions for things and agents
        for thing in self.things:
            solver.add(seq.thing_destination(indices=["INITIAL"],
                                             things=[thing],
                                             destinations=[thing])[0] == z3.BoolVal(True,ctx=context))
            if len(self.therblig_ids) > 0:
                solver.add(seq.thing_destination(indices=[self.therblig_ids[0]],
                                                 things=[thing],
                                                 destinations=[thing])[0] == z3.BoolVal(True,ctx=context))

        for agent in self.agents:
            solver.add(seq.agent_destination(indices=["INITIAL"],
                                             agents=[agent],
                                             destinations=[agent])[0] == z3.BoolVal(True,ctx=context))
            if len(self.therblig_ids) > 0:
                solver.add(seq.agent_destination(indices=[self.therblig_ids[0]],
                                                 agents=[agent],
                                                 destinations=[agent])[0] == z3.BoolVal(True,ctx=context))

        # Constrain initial grasping for agents to be false
        for agent in self.agents:
            solver.add(seq.agent_gripping(indices=["INITIAL"],
                                          agents=[agent])[0]==z3.BoolVal(False,ctx=context))
            if len(self.therblig_ids) > 0:
                solver.add(seq.agent_gripping(indices=[self.therblig_ids[0]],
                                              agents=[agent])[0]==z3.BoolVal(False,ctx=context))

        #print("BASIC CONSTRAINTS ADDED")
        #print(solver.check())

        # Set state settings (constraints) for each therblig
        for i, therblig in enumerate(self.therblig_ids):

            try:
                next_therblig = self.therblig_ids[i+1]
            except:
                next_therblig = "FINAL"

            sets = self.therbligs[therblig]["sets"]
            requires = self.therbligs[therblig]["requires"]
            params = self.therbligs[therblig]["parameters"]
            therblig_type = TherbligType.fromStr(self.therbligs[therblig]["type"])
            #print("## Requirements/Settings for {0}".format(therblig_type))

            # Constrain Agents Destinations
            if "agent_destination" in sets.keys() and (params['agent'] == "OPTIMIZE_DIRECTIVE" or params["agent"] == None):
                # TODO: handle case where agent is an optimize directive
                pass
            elif "agent_destination" in sets.keys():
                #print("Agent Destination Set Defined")
                for agent in self.agents:
                    #print("Considering Agent {0}".format(self.plan["agents"][agent]["name"]))
                    if agent == params["agent"] and params["destination"] != None:
                        #print("Agent moves to {0}".format(self.plan["destinations"][params["destination"]]["name"]))
                        # The agent moves to the given lcoation
                        next_agent_dest = seq.agent_destination(indices=[next_therblig],
                                                                agents=[agent],
                                                                destinations=[params["destination"]])[0]
                        solver.push()
                        solver.add(next_agent_dest == z3.BoolVal(True,ctx=context))
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            #print("error found, agent_destination, branch 1")
                            if self.therbligs[therblig]["is_repeated"]:
                                errors[self.therbligs[therblig]["id"]]["agent_destination_set"].append("On Loop {1}, Agent '{0}' cannot move to the given destination.".format(self.plan["agents"][agent]["name"],self.therbligs[therblig]["repeat"]))
                            else:
                                errors[self.therbligs[therblig]["id"]]["agent_destination_set"].append("Agent '{0}' cannot move to the given destination.".format(self.plan["agents"][agent]["name"]))

                    elif params["destination"] != None:
                        #print("Agent {0} stays in the same location.".format(self.plan["agents"][agent]["name"]))
                        # The agent stays in the same location
                        for destination in self.destinations:
                            this_agent_dest = seq.agent_destination(indices=[therblig],
                                                                    agents=[agent],
                                                                    destinations=[destination])[0]
                            next_agent_dest = seq.agent_destination(indices=[next_therblig],
                                                                    agents=[agent],
                                                                    destinations=[destination])[0]
                            solver.push()
                            solver.add(this_agent_dest == next_agent_dest)
                            if solver.check() != z3.sat:
                                solver.pop()
                                error_count += 1
                                if self.therbligs[therblig]["is_repeated"]:
                                    errors[self.therbligs[therblig]["id"]]["agent_destination_set"].append("On Loop {1}, Agent '{0}' cannot stay in the given destination.".format(self.plan["agents"][agent]["name"],self.therbligs[therblig]["repeat"]))
                                else:
                                    errors[self.therbligs[therblig]["id"]]["agent_destination_set"].append("Agent '{0}' cannot stay in the given destination".format(self.plan["agents"][agent]["name"]))
            else:
                #print("Agent Destination Set NOT Defined")
                for agent in self.agents:
                    for destination in self.destinations:
                        # The agent stays in the same location
                        #print("Passing along settings for agent '{0}', destination '{1}'".format(self.plan["agents"][agent]["name"],self.plan["destinations"][destination]["name"]))
                        this_agent_dest = seq.agent_destination(indices=[therblig],
                                                                agents=[agent],
                                                                destinations=[destination])[0]
                        next_agent_dest = seq.agent_destination(indices=[next_therblig],
                                                                agents=[agent],
                                                                destinations=[destination])[0]
                        solver.push()
                        solver.add(this_agent_dest == next_agent_dest)
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            #print("error found, agent_destination, branch 3")
                            if self.therbligs[therblig]["is_repeated"]:
                                errors[self.therbligs[therblig]["id"]]["agent_destination_set"].append("On Loop {1}, Agent '{0}' destination conflicts in next therblig.".format(self.plan["agents"][agent]["name"],self.therbligs[therblig]["repeat"]))
                            else:
                                errors[self.therbligs[therblig]["id"]]["agent_destination_set"].append("Agent '{0}' destination conflicts in next therblig.".format(self.plan["agents"][agent]["name"]))

            if "agent_destination" in requires.keys() and (params['agent'] == "OPTIMIZE_DIRECTIVE" or params["agent"] == None):
                # TODO: handle case where agent is an optimize directive
                pass
            elif "agent_destination" in requires.keys():
                #print("Agent Destination Requirement Defined")
                for agent in self.agents:
                    #print("Considering Agent {0}".format(self.plan["agents"][agent]["name"]))
                    if agent == params["agent"] and params["destination"] != None:
                        #print("Agent already must be at {0}".format(self.plan["destinations"][params["destination"]]["name"]))
                        # The agent moves to the given lcoation
                        agent_dest = seq.agent_destination(indices=[therblig],
                                                           agents=[agent],
                                                           destinations=[params["destination"]])[0]
                        solver.push()
                        solver.add(agent_dest == z3.BoolVal(True,ctx=context))
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            #print("error found, agent_destination, branch 4")
                            if self.therbligs[therblig]["is_repeated"]:
                                errors[self.therbligs[therblig]["id"]]["agent_destination_require"].append("On Loop {1}, Agent '{0}' is currently in a different destination.".format(self.plan["agents"][agent]["name"],self.therbligs[therblig]["repeat"]))
                            else:
                                errors[self.therbligs[therblig]["id"]]["agent_destination_require"].append("Agent '{0}' is currently in a different destination.".format(self.plan["agents"][agent]["name"]))

            # Constrain Things Destinations
            if "thing_destination" in sets.keys() and "thing" in params.keys() and params["thing"] == None:
                # TODO: handle case where agent is an optimize directive
                pass
            elif "thing_destination" in sets.keys():
                for thing in self.things:
                    #print("Considering Thing {0}".format(self.plan["things"][thing]["name"]))
                    if thing == params["thing"] and params["destination"] != None:
                        # The thing moves to the given lcoation
                        #print("Thing moves to {0}".format(self.plan["destinations"][params["destination"]]["name"]))
                        next_thing_dest = seq.thing_destination(indices=[next_therblig],
                                                                things=[thing],
                                                                destinations=[params["destination"]])[0]
                        solver.push()
                        solver.add(next_thing_dest == z3.BoolVal(True,ctx=context))
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            if self.therbligs[therblig]["is_repeated"]:
                                errors[self.therbligs[therblig]["id"]]["thing_destination_set"].append("On Loop {1}, Thing '{0}' destination conflicts in next therblig.".format(self.plan["things"][thing]["name"],self.therbligs[therblig]["repeat"]))
                            else:
                                errors[self.therbligs[therblig]["id"]]["thing_destination_set"].append("Thing '{0}' destination conflicts in next therblig.".format(self.plan["things"][thing]["name"]))

                    elif params["destination"] != None:
                        #print("Different thing than the one acted on, maintaining prior destination.")
                        # The thing stays in the same location
                        for destination in self.destinations:
                            this_thing_dest = seq.thing_destination(indices=[therblig],
                                                                    things=[thing],
                                                                    destinations=[destination])[0]
                            next_thing_dest = seq.thing_destination(indices=[next_therblig],
                                                                    things=[thing],
                                                                    destinations=[destination])[0]
                            solver.push()
                            solver.add(this_thing_dest == next_thing_dest)
                            if solver.check() != z3.sat:
                                solver.pop()
                                error_count += 1
                                if self.therbligs[therblig]["is_repeated"]:
                                    errors[self.therbligs[therblig]["id"]]["thing_destination_set"].append("On Loop {1}, Thing '{0}' cannot stay in the given destination.".format(self.plan["things"][thing]["name"],self.therbligs[therblig]["repeat"]))
                                else:
                                    errors[self.therbligs[therblig]["id"]]["thing_destination_set"].append("Thing '{0}' cannot stay in the given destination.".format(self.plan["things"][thing]["name"]))

            else:
                #print("Thing Destination Set NOT Defined")
                for thing in self.things:
                    for destination in self.destinations:
                        #print("Passing along settings for thing '{0}', destination '{1}'".format(self.plan["things"][thing]["name"],self.plan["destinations"][destination]["name"]))
                        # The thing stays in the same location
                        this_thing_dest = seq.thing_destination(indices=[therblig],
                                                                things=[thing],
                                                                destinations=[destination])[0]
                        next_thing_dest = seq.thing_destination(indices=[next_therblig],
                                                                things=[thing],
                                                                destinations=[destination])[0]
                        solver.push()
                        solver.add(this_thing_dest == next_thing_dest)
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            if self.therbligs[therblig]["is_repeated"]:
                                errors[self.therbligs[therblig]["id"]]["thing_destination_set"].append("On Loop {1}, Thing '{0}' destination conflicts in next therblig.".format(self.plan["things"][thing]["name"],self.therbligs[therblig]["repeat"]))
                            else:
                                errors[self.therbligs[therblig]["id"]]["thing_destination_set"].append("Thing '{0}' destination conflicts in next therblig.".format(self.plan["things"][thing]["name"]))


            if "thing_destination" in requires.keys() and "thing" in params.keys() and params["thing"] == None:
                # TODO: handle case where thing is an optimize directive
                pass
            elif "thing_destination" in requires.keys():
                #print("Thing Destination Requirement Defined")
                for thing in self.things:
                    #print("Considering Thing {0}".format(self.plan["things"][thing]["name"]))
                    if thing == params["thing"] and params["destination"] != None:
                        #print("Thing already must already be at {0}".format(self.plan["destinations"][params["destination"]]["name"]))
                        # The thing moves to the given lcoation
                        thing_dest = seq.thing_destination(indices=[therblig],
                                                           things=[thing],
                                                           destinations=[params["destination"]])[0]
                        solver.push()
                        solver.add(thing_dest == z3.BoolVal(True,ctx=context))
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            if self.therbligs[therblig]["is_repeated"]:
                                errors[self.therbligs[therblig]["id"]]["thing_destination_require"].append("On Loop {1}, Thing '{0}' is currently in a different destination.".format(self.plan["things"][thing]["name"],self.therbligs[therblig]["repeat"]))
                            else:
                                errors[self.therbligs[therblig]["id"]]["thing_destination_require"].append("Thing '{0}' is currently in a different destination.".format(self.plan["things"][thing]["name"]))


            # Constrain Grasping State
            if "grasping" in sets.keys() and (params['agent'] == "OPTIMIZE_DIRECTIVE" or params["agent"] == None):
                # TODO: handle case where agent is an optimize directive
                pass
            elif "grasping" in sets.keys():
                #print("Agent Grasping Set Defined")
                for agent in self.agents:
                    #print("Considering Agent {0}".format(self.plan["agents"][agent]["name"]))
                    if agent == params["agent"]:
                        #print("Agent's grasping set to {0}".format(sets["grasping"]))
                        # The agent moves to the given lcoation
                        next_agent_grasp = seq.agent_gripping(indices=[next_therblig],
                                                                 agents=[agent])[0]
                        solver.push()
                        solver.add(next_agent_grasp == z3.BoolVal(int(sets["grasping"]),ctx=context))
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            if sets["grasping"]:
                                if self.therbligs[therblig]["is_repeated"]:
                                    errors[self.therbligs[therblig]["id"]]["agent_gripping_set"].append("On Loop {1}, Agent '{0}' cannot grasp for next therblig.".format(self.plan["things"][thing]["name"],self.therbligs[therblig]["repeat"]))
                                else:
                                    errors[self.therbligs[therblig]["id"]]["agent_gripping_set"].append("Agent '{0}' cannot grasp for next therblig.".format(self.plan["things"][thing]["name"]))
                            else:
                                if self.therbligs[therblig]["is_repeated"]:
                                    errors[self.therbligs[therblig]["id"]]["agent_gripping_set"].append("On Loop {1}, Agent '{0}' cannot release grasp for next therblig.".format(self.plan["things"][thing]["name"],self.therbligs[therblig]["repeat"]))
                                else:
                                    errors[self.therbligs[therblig]["id"]]["agent_gripping_set"].append("Agent '{0}' cannot release grasp for next therblig.".format(self.plan["things"][thing]["name"]))
                    else:
                        #print("Different agent than the one assigned, maintaining values")
                        # The agent stays in the same location
                        this_agent_grasp = seq.agent_gripping(indices=[therblig],
                                                              agents=[agent])[0]
                        next_agent_grasp = seq.agent_gripping(indices=[next_therblig],
                                                              agents=[agent])[0]
                        solver.push()
                        solver.add(this_agent_grasp == next_agent_grasp)
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            if self.therbligs[therblig]["is_repeated"]:
                                errors[self.therbligs[therblig]["id"]]["agent_gripping_set"].append("On Loop {1}, Agent '{0}' cannot maintain gripping status for next therblig.".format(self.plan["things"][thing]["name"],self.therbligs[therblig]["repeat"]))
                            else:
                                errors[self.therbligs[therblig]["id"]]["agent_gripping_set"].append("Agent '{0}' cannot maintain gripping status for next therblig.".format(self.plan["things"][thing]["name"]))

            else:
                #print("Agent Grasping Set NOT Defined")
                for agent in self.agents:
                    # The agent stays in the same location

                    this_agent_grasp = seq.agent_gripping(indices=[therblig],
                                                          agents=[agent])[0]
                    next_agent_grasp = seq.agent_gripping(indices=[next_therblig],
                                                          agents=[agent])[0]
                    #print("Passing along settings for agent '{0}' gripping status".format(self.plan["agents"][agent]["name"]))
                    solver.push()
                    solver.add(this_agent_grasp == next_agent_grasp)
                    if solver.check() != z3.sat:
                        solver.pop()
                        error_count += 1
                        if self.therbligs[therblig]["is_repeated"]:
                            errors[self.therbligs[therblig]["id"]]["agent_gripping_set"].append("On Loop {1}, Agent '{0}' cannot maintain gripping status for next therblig.".format(self.plan["things"][thing]["name"],self.therbligs[therblig]["repeat"]))
                        else:
                            errors[self.therbligs[therblig]["id"]]["agent_gripping_set"].append("Agent '{0}' cannot maintain gripping status for next therblig.".format(self.plan["things"][thing]["name"]))

            if "grasping" in requires.keys() and (params['agent'] == "OPTIMIZE_DIRECTIVE" or params["agent"] == None):
                # TODO: handle case where agent is an optimize directive
                pass
            elif "grasping" in requires.keys():
                #print("Agent Grasping Requirement Defined")
                for agent in self.agents:
                    #print("Considering Agent {0}".format(self.plan["agents"][agent]["name"]))
                    if agent == params["agent"]:
                        #print("Agent's grasping is required to be {0}".format(requires["grasping"]))
                        # The agent moves to the given lcoation
                        agent_grasp = seq.agent_gripping(indices=[therblig],
                                                         agents=[agent])[0]
                        solver.push()
                        solver.add(agent_grasp == z3.BoolVal(int(requires["grasping"]),ctx=context))
                        if solver.check() != z3.sat:
                            solver.pop()
                            error_count += 1
                            if requires["grasping"]:
                                if self.therbligs[therblig]["is_repeated"]:
                                    errors[self.therbligs[therblig]["id"]]["agent_gripping_require"].append("On Loop {1}, Agent '{0}' cannot perform this therblig because it isn't gripping.".format(self.plan["agents"][agent]["name"],self.therbligs[therblig]["repeat"]))
                                else:
                                    errors[self.therbligs[therblig]["id"]]["agent_gripping_require"].append("Agent '{0}' cannot perform this therblig because it isn't gripping.".format(self.plan["agents"][agent]["name"]))
                            else:
                                if self.therbligs[therblig]["is_repeated"]:
                                    errors[self.therbligs[therblig]["id"]]["agent_gripping_require"].append("On Loop {1}, Agent '{0}' cannot perform this therblig because it is gripping.".format(self.plan["agents"][agent]["name"],self.therbligs[therblig]["repeat"]))
                                else:
                                    errors[self.therbligs[therblig]["id"]]["agent_gripping_require"].append("Agent '{0}' cannot perform this therblig because it is gripping.".format(self.plan["agents"][agent]["name"]))

            # Constrain Agent/Thing Matching
            if "agent_thing" in requires.keys():
                #print("Agent Grasping Requirement Defined")
                for agent in self.agents:
                    #print("Considering Agent {0}".format(self.plan["agents"][agent]["name"]))
                    if agent == params["agent"]:
                        for thing in self.things:
                            if thing == params["thing"]:
                                for destination in self.destinations:
                                    agent_destination = seq.agent_destination(indices=[therblig],
                                                                              agents=[agent],
                                                                              destinations=[destination])[0]
                                    thing_destination = seq.thing_destination(indices=[therblig],
                                                                              things=[thing],
                                                                              destinations=[destination])[0]
                                    solver.push()
                                    solver.add(agent_destination == thing_destination)
                                    if solver.check() != z3.sat:
                                        solver.pop()
                                        error_count += 1
                                        if self.therbligs[therblig]["is_repeated"]:
                                            errors[self.therbligs[therblig]["id"]]["agent_thing_require"].append("On Loop {1}, Agent '{0}' cannot act on Thing {2} because they are in different locations.".format(self.plan["agents"][agent]["name"],
                                                                                                                                                                                                                    self.therbligs[therblig]["repeat"],
                                                                                                                                                                                                                    self.plan["things"][thing]["name"]))
                                        else:
                                            errors[self.therbligs[therblig]["id"]]["agent_thing_require"].append("Agent '{0}' cannot act on Thing {1} because they are in different locations.".format(self.plan["agents"][agent]["name"],
                                                                                                                                                                                                       self.plan["things"][thing]["name"]))

            #print(error_count)
            # if error_count:
            #     self.show(solver, seq)

        return error_count, errors

    def show(self, solver, seq):
        if solver.check() == z3.sat:
            s1,s2,s3,s4 = seq.evaluated(solver.model(),self.plan)
            print(s1)
            print(s2)
            print(s3)
            print(s4)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No JSON File Provided!")
    else:
        file = sys.argv[1]
        with open(file, "r") as read_file:
            data = convert(json.load(read_file))
        print("######### Checking Plan #########")
        v = Verifier(data)
        ec, e = v.check()
        print("Error Count: {0}".format(ec))
        print(e)
