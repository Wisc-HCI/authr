#!/usr/bin/env python
import z3
import json
import sys
from enum import Enum
from copy import deepcopy
from itertools import product, tee, izip
import time

# Removable for full
# import pandas
# import numpy as np

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)

def convert(input):
    if isinstance(input, dict):
        return {convert(key): convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

class StateSpace(object):
    def __init__(self, context, agents, therbligs, things, destinations):
        self.agents = agents
        self.therbligs = ["INITIAL"]+therbligs
        self.things = things
        self.destinations = destinations

        # Create the Therblig/Agent Allocation Space
        self.agent_allocation_data = {}
        for i in range(len(self.therbligs)):
            for j in range(len(self.agents)):
                self.agent_allocation_data["aas_{0}_{1}".format(
                                     self.therbligs[i],
                                     self.agents[j])] = z3.Bool("aas_{0}_{1}".format(i,j),ctx=context)

        # Create the Therblig/Thing Allocation Space
        self.thing_allocation_data = {}
        for i in range(len(self.therbligs)):
            for j in range(len(self.things)):
                self.thing_allocation_data["tas_{0}_{1}".format(
                                     self.therbligs[i],
                                     self.things[j])] = z3.Bool("tas_{0}_{1}".format(i,j),ctx=context)

        # Create the Therblig/Thing Allocation Space
        self.destination_allocation_data = {}
        for i in range(len(self.therbligs)):
            for j in range(len(self.destinations)):
                self.destination_allocation_data["das_{0}_{1}".format(
                                     self.therbligs[i],
                                     self.destinations[j])] = z3.Bool("das_{0}_{1}".format(i,j),ctx=context)

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

    def agent_allocation(self, therbligs=[], agents=[]):
        responses = []
        for index in therbligs:
            for agent in agents:
                responses.append(self.agent_allocation_data["aas_{0}_{1}".format(index,agent)])
        return responses

    def thing_allocation(self, therbligs=[], things=[]):
        responses = []
        for index in therbligs:
            for thing in things:
                responses.append(self.thing_allocation_data["tas_{0}_{1}".format(index,thing)])
        return responses

    def destination_allocation(self, therbligs=[], destinations=[]):
        responses = []
        for index in therbligs:
            for destination in destinations:
                responses.append(self.destination_allocation_data["das_{0}_{1}".format(index,destination)])
        return responses

    def thing_destination(self, therbligs=[], things=[], destinations=[]):
        responses = []
        for index in therbligs:
            for thing in things:
                for destination in destinations:
                    responses.append(self.thing_destination_data["tds_{0}_{1}_{2}".format(index,destination,thing)])
        return responses

    def agent_destination(self, therbligs=[], agents=[], destinations=[]):
        responses = []
        for index in therbligs:
            for agent in agents:
                for destination in destinations:
                    responses.append(self.agent_destination_data["ads_{0}_{1}_{2}".format(index,destination,agent)])
        return responses

    def agent_thing(self, therblig, agent, thing):
        responses = []
        for destination in self.destinations:
            agent_setting = self.agent_destination(therbligs=[therblig],agents=[agent],destinations=[destination])[0]
            thing_setting = self.thing_destination(therbligs=[therblig],things=[thing],destinations=[destination])[0]
            responses.append(agent_setting == thing_setting)
        return responses

    def agent_gripping(self, therbligs=[], agents=[]):
        responses = []
        for index in therbligs:
            for agent in agents:
                responses.append(self.agent_gripping_data["ags_{0}_{1}".format(index,agent)])
        return responses

    def evaluated(self,model,plan,slice=slice(0,-1)):
        cols=["INITIAL"]
        evaluated_actions = {}
        for i,t in enumerate(self.therbligs[slice]):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
                cols.append(name)
            else:
                name = t
            for a in self.agents:
                data[plan["agents"][a]["name"]] = model.evaluate(self.agent_allocation(therbligs=[t],agents=[a])[0])
                ##print(model.evaluate(self.actions(therbligs=[t],agents=[a])[0]))
            evaluated_actions[name] = data
        evaluated_actions = pandas.DataFrame(evaluated_actions,columns=cols)

        evaluated_things = {}
        for i,t in enumerate(self.therbligs[slice]):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
            else:
                name = t
            for h in self.things:
                data[plan["things"][h]["name"]] = model.evaluate(self.thing_allocation(therbligs=[t],things=[h])[0])
                ##print(model.evaluate(self.actions(therbligs=[t],agents=[a])[0]))
            evaluated_things[name] = data
        evaluated_things = pandas.DataFrame(evaluated_things,columns=cols)

        evaluated_destinations = {}
        for i,t in enumerate(self.therbligs[slice]):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
            else:
                name = t
            for d in self.destinations:
                data[plan["destinations"][d]["name"]] = model.evaluate(self.destination_allocation(therbligs=[t],destinations=[d])[0])
                ##print(model.evaluate(self.actions(therbligs=[t],agents=[a])[0]))
            evaluated_destinations[name] = data
        evaluated_destinations = pandas.DataFrame(evaluated_destinations,columns=cols)

        evaluated_agent_destinations = {}
        for i,t in enumerate(self.therbligs[slice]):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
            else:
                name = t
            for a in self.agents:
                locations = []
                for d in self.destinations:
                    v = model.evaluate(self.agent_destination(therbligs=[t],agents=[a],destinations=[d])[0])
                    if v == True:
                        locations.append(plan["destinations"][d]["name"])
                if len(locations) == 0:
                    locations.append("NONE")
                data[plan["agents"][a]["name"]] = ",".join(locations)
            evaluated_agent_destinations[name] = data
        evaluated_agent_destinations = pandas.DataFrame(evaluated_agent_destinations,columns=cols)

        evaluated_thing_destinations = {}
        for i,t in enumerate(self.therbligs[slice]):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
            else:
                name = t
            for a in self.things:
                locations = []
                for d in self.destinations:
                    v = model.evaluate(self.thing_destination(therbligs=[t],things=[a],destinations=[d])[0])
                    if v == True:
                        locations.append(plan["destinations"][d]["name"])
                if len(locations) == 0:
                    locations.append("NONE")
                data[plan["things"][a]["name"]] = ",".join(locations)
            evaluated_thing_destinations[name] = data
        evaluated_thing_destinations = pandas.DataFrame(evaluated_thing_destinations,columns=cols)

        evaluated_grasping = {}
        for i,t in enumerate(self.therbligs[slice]):
            data = {}
            if t != "INITIAL" and t != "FINAL":
                name = i
            else:
                name = t
            for a in self.agents:
                data[plan["agents"][a]["name"]] = model.evaluate(self.agent_gripping(therbligs=[t],agents=[a])[0])
                ##print(model.evaluate(self.actions(therbligs=[t],agents=[a])[0]))
            evaluated_grasping[name] = data
        evaluated_grasping = pandas.DataFrame(evaluated_grasping,columns=cols)

        return evaluated_actions, evaluated_things, evaluated_destinations, evaluated_agent_destinations, evaluated_thing_destinations, evaluated_grasping


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

    def infer_goal(self):
        thing_locs = {thing:thing for thing in self.things}
        for therblig_id in self.therblig_ids:
            if self.therbligs[therblig_id]["type"] == "transport_loaded":
                params = self.therbligs[therblig_id]["parameters"]
                if params["thing"] != None and params["destination"] != None:
                    thing_locs[params["thing"]] = params["destination"]
        return thing_locs

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

        # Define the states, which is a therblig x type x agent grid
        states = StateSpace(context,self.agents,self.therblig_ids,self.things,self.destinations)

        # Define Basic Constraints
        basic_constraints = self.basic_constraints(states,context)
        solver.push()
        for basic_constraint in basic_constraints:
            solver.add(basic_constraint)

        solver.push()

        therblig_constraint_sets = self.therblig_constraints(states,context)

        # Since one check is enough to see whether the whole thing is valid,
        # First check that it is valid, and then only look for specific errors
        # if not valid.
        for therblig_constraints in therblig_constraint_sets:
            for (therblig, field, message, constraint) in therblig_constraints:
                solver.add(constraint)

        if solver.check() != z3.sat:
            solver.pop()
            for therblig_constraints in therblig_constraint_sets:
                solver.push()
                for (therblig, field, message, constraint) in therblig_constraints:
                    solver.add(constraint)
                if solver.check() != z3.sat:
                    solver.pop()
                    for (therblig, field, message, constraint) in therblig_constraints:
                        solver.push()
                        solver.add(constraint)
                        if solver.check() != z3.sat:
                            solver.pop()
                            errors[therblig][field].append(message)
                            error_count += 1

        return error_count, errors

    def show(self, solver, states, slice=slice(0,-1)):
        if solver.check() == z3.sat:
            s1,s2,s3,s4,s5,s6 = states.evaluated(solver.model(),self.plan,slice=slice)
            print("\nAllocation")
            print(s1)
            print("\nThings")
            print(s2)
            print("\nDestinations")
            print(s3)
            print("\nAgent Destinations")
            print(s4)
            print("\nThing Destinations")
            print(s5)
            print("Agent Gripping")
            print(s6)

    def basic_constraints(self,states,context):
        constraints = []
        constraints.append(z3.PbEq([(o,1) for o in states.agent_allocation(therbligs=["INITIAL"],agents=self.agents)],0,ctx=context))
        constraints.append(z3.PbEq([(o,1) for o in states.thing_allocation(therbligs=["INITIAL"],things=self.things)],0,ctx=context))
        constraints.append(z3.PbEq([(o,1) for o in states.destination_allocation(therbligs=["INITIAL"],destinations=self.destinations)],0,ctx=context))
        for therblig in self.therblig_ids:
            constraints.append(z3.PbEq([(o,1) for o in states.agent_allocation(therbligs=[therblig],agents=self.agents)],1,ctx=context))
            if "thing" in self.therbligs[therblig]["parameters"].keys():
                constraints.append(z3.PbEq([(o,1) for o in states.thing_allocation(therbligs=[therblig],things=self.things)],1,ctx=context))
            else:
                constraints.append(z3.PbEq([(o,1) for o in states.thing_allocation(therbligs=[therblig],things=self.things)],0,ctx=context))
            if therblig != "INITIAL" and "destination" in self.therbligs[therblig]["parameters"].keys():
                constraints.append(z3.PbEq([(o,1) for o in states.destination_allocation(therbligs=[therblig],destinations=self.destinations)],1,ctx=context))
            else:
                constraints.append(z3.PbEq([(o,1) for o in states.destination_allocation(therbligs=[therblig],destinations=self.destinations)],0,ctx=context))

        # Constrain agents and things to only appear in a single location at a given time
        for therblig in ["INITIAL"]+self.therblig_ids:
            for agent in self.agents:
                constraints.append(z3.PbEq([(o,1) for o in states.agent_destination(therbligs=[therblig],agents=[agent],destinations=self.destinations)],1,ctx=context))
            for thing in self.things:
                constraints.append(z3.PbEq([(o,1) for o in states.thing_destination(therbligs=[therblig],things=[thing],destinations=self.destinations)],1,ctx=context))

        # Prevent multiple things, agents from occupying the same locations
        for therblig in ["INITIAL"]+self.therblig_ids:
            for destination in self.destinations:
                if len(self.agents) > 0:
                    constraints.append(z3.PbLe([(o,1) for o in states.agent_destination(therbligs=[therblig],agents=self.agents,destinations=[destination])],1))
                if len(self.things) > 0:
                    constraints.append(z3.PbLe([(o,1) for o in states.thing_destination(therbligs=[therblig],things=self.things,destinations=[destination])],1))


        # Constrain initial positions for things and agents
        for thing in self.things:
            constraints.append(states.thing_destination(therbligs=["INITIAL"],
                                             things=[thing],
                                             destinations=[thing])[0] == z3.BoolVal(True,ctx=context))
        for agent in self.agents:
            constraints.append(states.agent_destination(therbligs=["INITIAL"],
                                             agents=[agent],
                                             destinations=[agent])[0] == z3.BoolVal(True,ctx=context))

        # Constrain the goal state based on inference.
        thing_goal_destinations = self.infer_goal()
        for thing,destination in thing_goal_destinations.items():
            constraints.append(states.thing_destination(therbligs=[self.therblig_ids[-1]],
                                                things=[thing],
                                                destinations=[destination])[0] == z3.BoolVal(True,ctx=context))

        # Constrain initial grasping for agents to be false
        for agent in self.agents:
            constraints.append(states.agent_gripping(therbligs=["INITIAL"],
                                          agents=[agent])[0]==z3.BoolVal(False,ctx=context))

        # Attempt to perform allocation by constraining the allocation variables.
        for this in self.therblig_ids:
            for setting in self.therbligs[this]["parameters"].keys():
                add = False
                if setting == "agent" and self.therbligs[this]["parameters"]["agent"] != "OPTIMIZE_DIRECTIVE":
                    add = True
                    constraint = states.agent_allocation(therbligs=[this],agents=[self.therbligs[this]["parameters"]["agent"]])[0] == z3.BoolVal(True,ctx=context)
                elif setting == "thing" and self.therbligs[this]["parameters"]["thing"] != None:
                    add = True
                    constraint = states.thing_allocation(therbligs=[this],things=[self.therbligs[this]["parameters"]["thing"]])[0] == z3.BoolVal(True,ctx=context)
                elif setting == "destination" and self.therbligs[this]["parameters"]["destination"] != None:
                    add = True
                    constraint = states.destination_allocation(therbligs=[this],destinations=[self.therbligs[this]["parameters"]["destination"]])[0] == z3.BoolVal(True,ctx=context)
                if add:
                    constraints.append(constraint)
        return constraints

    def therblig_constraints(self,states,context):
        constraints = []
        for (past,this) in pairwise(["INITIAL"]+self.therblig_ids):
            constraints.append(self.therblig_to_constraints(past,this,states,context))
        return constraints

    def agent_destination_pre_constraints(self, past, therblig, states, context):
        agent_clauses = []
        # Conceptually represent as an if statement on agent allocation variable
        for (agent,destination) in product(self.agents,self.destinations):
            agent_name = self.plan["agents"][agent]["name"]
            destination_name = self.plan["destinations"][destination]["name"]
            agent_agnostic = therblig['parameters']['agent'] == "OPTIMIZE_DIRECTIVE"

            # If both match, enforce that the agent be at the specified destination.
            if agent_agnostic:
                message = 'No agents are at destination "{0}"'.format(destination_name)
            else:
                message = "Agent '{0}' is not at destination '{1}'".format(agent_name,destination_name)
            true_true = z3.And(states.agent_allocation(therbligs=[therblig["eid"]],agents=[agent])[0],
                               states.destination_allocation(therbligs=[therblig["eid"]],destinations=[destination])[0],context)
            clause = z3.Implies(true_true,states.agent_destination(therbligs=[past],agents=[agent],destinations=[destination])[0] == z3.BoolVal(True,ctx=context))
            agent_clauses.append((message,clause))

            # If neither match, carry over.
            false_false = z3.And(z3.Not(states.agent_allocation(therbligs=[therblig["eid"]],agents=[agent])[0],context),
                                 z3.Not(states.destination_allocation(therbligs=[therblig["eid"]],destinations=[destination])[0],context),context)
            clause = z3.Implies(false_false,states.agent_destination(therbligs=[past],agents=[agent],destinations=[destination])[0] == z3.BoolVal(False,ctx=context))
            agent_clauses.append((message,clause))

            # If one matches, ignore. This should get sorted out by other constraints.

        return [("agent_destination_require",ac[0],ac[1]) for ac in agent_clauses]

    def thing_destination_pre_constraints(self, past, therblig, states, context):
        thing_clauses = []
        # Conceptually represent as an if statement on thing allocation variable
        for (thing,destination) in product(self.things,self.destinations):
            thing_name = self.plan["things"][thing]["name"]
            destination_name = self.plan["destinations"][destination]["name"]

            # If both match, enforce that the thing be at the specified destination.
            message = "Thing '{0}' is not at destination '{1}'".format(thing_name,destination_name)
            true_true = z3.And(states.thing_allocation(therbligs=[therblig["eid"]],things=[thing])[0],
                               states.destination_allocation(therbligs=[therblig["eid"]],destinations=[destination])[0],context)
            clause = z3.Implies(true_true,states.thing_destination(therbligs=[past],things=[thing],destinations=[destination])[0] == z3.BoolVal(True,ctx=context))
            thing_clauses.append((message,clause))

            # If neither match, carry over.
            message = "Thing '{0}' is not at destination '{1}'".format(thing_name,destination_name)
            false_false = z3.And(z3.Not(states.thing_allocation(therbligs=[therblig["eid"]],things=[thing])[0],context),
                                 z3.Not(states.destination_allocation(therbligs=[therblig["eid"]],destinations=[destination])[0],context),context)
            clause = z3.Implies(false_false,states.thing_destination(therbligs=[past],things=[thing],destinations=[destination])[0] == z3.BoolVal(False,ctx=context))
            thing_clauses.append((message,clause))

            # If one matches, ignore. This should get sorted out by other constraints.

        return [("thing_destination_require",tc[0],tc[1]) for tc in thing_clauses]

    def agent_thing_pre_constraints(self, past, therblig, states, context):
        agent_thing_clauses = []
        for (agent,thing) in product(self.agents,self.things):
            agent_name = self.plan["agents"][agent]["name"]
            thing_name = self.plan["things"][thing]["name"]
            agent_agnostic = therblig['parameters']['agent'] == "OPTIMIZE_DIRECTIVE"

            # If allocated, require a match to gripping state
            if agent_agnostic:
                message = "Some agent and thing '{1}' must be at the same destination".format(agent_name,thing_name)
            else:
                message = "Agent '{0}' and thing '{1}' must be at the same destination".format(agent_name,thing_name)
            true_true = z3.And(states.agent_allocation(therbligs=[therblig["eid"]],agents=[agent])[0],
                               states.thing_allocation(therbligs=[therblig["eid"]],things=[thing])[0],context)
            clause = z3.Implies(true_true,z3.And(*(states.agent_thing(therblig=past,agent=agent,thing=thing)+[context])))
            agent_thing_clauses.append((message,clause))

        return [("agent_thing_require",atc[0],atc[1]) for atc in agent_thing_clauses]

    def agent_gripping_pre_constraints(self, past, therblig, states, context):
        agent_clauses = []
        for agent in self.agents:
            agent_name = self.plan["agents"][agent]["name"]
            value = therblig["requires"]["grasping"]
            agent_agnostic = therblig['parameters']['agent'] == "OPTIMIZE_DIRECTIVE"

            # If both match, move the agent must set gripping.
            if agent_agnostic:
                if value:
                    message = "Agent must be gripping"
                else:
                    message = "Agent must not be gripping"
            else:
                if value:
                    message = "Agent '{0}' must be gripping".format(agent_name)
                else:
                    message = "Agent '{0}' must not be gripping".format(agent_name)
            clause = z3.Implies(states.agent_allocation(therbligs=[therblig["eid"]],agents=[agent])[0],
                                states.agent_gripping(therbligs=[past],agents=[agent])[0] == z3.BoolVal(therblig["requires"]["grasping"],ctx=context))
            agent_clauses.append((message,clause))

        return [("agent_gripping_require",ac[0],ac[1]) for ac in agent_clauses]

    def agent_destination_post_constraints(self, past, therblig, states, context):
        # Conceptually represent as an if statement on allocation variables thing and destination
        agent_clauses = []
        for (agent,destination) in product(self.agents,self.destinations):
            agent_name = self.plan["agents"][agent]["name"]
            destination_name = self.plan["destinations"][destination]["name"]
            agent_agnostic = therblig['parameters']['agent'] == "OPTIMIZE_DIRECTIVE"

            # If both match, move the agent to the destination.
            if agent_agnostic:
                message = "Some agent must be able to travel to destination '{0}'".format(destination_name)
            else:
                message = "Agent '{0}' must be able to travel to destination '{1}'".format(agent_name,destination_name)
            true_true = z3.And(states.agent_allocation(therbligs=[therblig["eid"]],agents=[agent])[0],
                               states.destination_allocation(therbligs=[therblig["eid"]],destinations=[destination])[0],context)
            clause = z3.Implies(true_true,states.agent_destination(therbligs=[therblig["eid"]],agents=[agent],destinations=[destination])[0] == z3.BoolVal(True,ctx=context))
            agent_clauses.append((message,clause))

            # If neither match, carry over.
            if agent_agnostic:
                message = "Some agent must be able to stay at destination '{0}'".format(destination_name)
            else:
                message = "Agent '{0}' must be able to stay at destination '{1}'".format(agent_name,destination_name)
            false_false = z3.And(z3.Not(states.agent_allocation(therbligs=[therblig["eid"]],agents=[agent])[0],context),
                                 z3.Not(states.destination_allocation(therbligs=[therblig["eid"]],destinations=[destination])[0],context),context)
            clause = z3.Implies(false_false,states.agent_destination(therbligs=[therblig["eid"]],agents=[agent],destinations=[destination])[0] == states.agent_destination(therbligs=[past],agents=[agent],destinations=[destination])[0])
            agent_clauses.append((message,clause))

            # If one matches, ignore. This should get sorted out by other constraints.

        return [("agent_destination_set",ac[0],ac[1]) for ac in agent_clauses]

    def agent_destination_consistency(self, past, therblig, states, context):
        agent_clauses = []
        for (agent,destination) in product(self.agents,self.destinations):
            agent_name = self.plan["agents"][agent]["name"]
            destination_name = self.plan["destinations"][destination]["name"]
            agent_agnostic = therblig['parameters']['agent'] == "OPTIMIZE_DIRECTIVE"

            if agent_agnostic:
                message = "Some agent must be able to stay at destination '{0}'".format(destination_name)
            else:
                message = "Agent '{0}' must be able to stay at destination '{1}'".format(agent_name,destination_name)
            clause = states.agent_destination(therbligs=[therblig["eid"]],agents=[agent],destinations=[destination])[0] == states.agent_destination(therbligs=[past],agents=[agent],destinations=[destination])[0]
            agent_clauses.append((message,clause))
        return [("agent_destination_set",ac[0],ac[1]) for ac in agent_clauses]

    def thing_destination_post_constraints(self, past, therblig, states, context):
        # Conceptually represent as an if statement on allocation variables thing and destination
        thing_clauses = []
        for (thing,destination) in product(self.things,self.destinations):
            thing_name = self.plan["things"][thing]["name"]
            destination_name = self.plan["destinations"][destination]["name"]

            # If both match, move the thing to the destination.
            message = "Thing '{0}' must be able to travel to destination '{1}'".format(thing_name,destination_name)
            true_true = z3.And(states.thing_allocation(therbligs=[therblig["eid"]],things=[thing])[0],
                               states.destination_allocation(therbligs=[therblig["eid"]],destinations=[destination])[0],context)
            clause = z3.Implies(true_true,states.thing_destination(therbligs=[therblig["eid"]],things=[thing],destinations=[destination])[0] == z3.BoolVal(True,ctx=context))
            thing_clauses.append((message,clause))

            # If neither match, carry over.
            message = "Thing '{0}' must be able to stay at destination '{1}'".format(thing_name,destination_name)
            false_false = z3.And(z3.Not(states.thing_allocation(therbligs=[therblig["eid"]],things=[thing])[0],context),
                                 z3.Not(states.destination_allocation(therbligs=[therblig["eid"]],destinations=[destination])[0],context),context)
            clause = z3.Implies(false_false,states.thing_destination(therbligs=[therblig["eid"]],things=[thing],destinations=[destination])[0] == states.thing_destination(therbligs=[past],things=[thing],destinations=[destination])[0])
            thing_clauses.append((message,clause))

            # If one matches, ignore. This should get sorted out by other constraints.

        return [("thing_destination_set",tc[0],tc[1]) for tc in thing_clauses]

    def thing_destination_consistency(self, past, therblig, states, context):
        thing_clauses = []
        for (thing,destination) in product(self.things,self.destinations):
            thing_name = self.plan["things"][thing]["name"]
            destination_name = self.plan["destinations"][destination]["name"]
            message = "Thing '{0}' must be able to stay at destination '{1}'".format(thing_name,destination_name)
            clause = states.thing_destination(therbligs=[therblig["eid"]],things=[thing],destinations=[destination])[0] == states.thing_destination(therbligs=[past],things=[thing],destinations=[destination])[0]
            thing_clauses.append((message,clause))
        return [("thing_destination_set",tc[0],tc[1]) for tc in thing_clauses]

    def agent_gripping_post_constraints(self, past, therblig, states, context):
        agent_clauses = []
        for agent in self.agents:
            agent_name = self.plan["agents"][agent]["name"]
            value = therblig["sets"]["grasping"]
            agent_agnostic = therblig['parameters']['agent'] == "OPTIMIZE_DIRECTIVE"
            # If allocated, change the grip
            if agent_agnostic:
                if value:
                    message = "Some agent must be able to grip".format(agent_name)
                else:
                    message = "Some agent must be able to release grip".format(agent_name)
            else:
                if value:
                    message = "Agent '{0}' must be able to grip".format(agent_name)
                else:
                    message = "Agent '{0}' must be able to release grip".format(agent_name)
            clause = z3.Implies(states.agent_allocation(therbligs=[therblig["eid"]],agents=[agent])[0],
                                states.agent_gripping(therbligs=[therblig["eid"]],agents=[agent])[0] == z3.BoolVal(therblig["sets"]["grasping"],ctx=context))
            agent_clauses.append((message,clause))
            # If not allocated, stay the same
            if agent_agnostic:
                message = "This agent must maintain gripping state"
            else:
                message = "Agent '{0}' must maintain gripping state".format(agent_name)
            clause = z3.Implies(z3.Not(states.agent_allocation(therbligs=[therblig["eid"]],agents=[agent])[0],context),
                                states.agent_gripping(therbligs=[therblig["eid"]],agents=[agent])[0] == states.agent_gripping(therbligs=[past],agents=[agent])[0])
            agent_clauses.append((message,clause))
        return [("agent_gripping_set",ac[0],ac[1]) for ac in agent_clauses]

    def agent_gripping_consistency(self, past, therblig, states, context):
        agent_clauses = []
        for agent in self.agents:
            agent_name = self.plan["agents"][agent]["name"]
            agent_agnostic = therblig['parameters']['agent'] == "OPTIMIZE_DIRECTIVE"
            if agent_agnostic:
                message = "Agent must maintain gripping state"
            else:
                message = "Agent '{0}' must maintain gripping state".format(agent_name)
            clause = states.agent_gripping(therbligs=[therblig["eid"]],agents=[agent])[0] == states.agent_gripping(therbligs=[past],agents=[agent])[0]
            agent_clauses.append((message, clause))
        return [("agent_gripping_set",ac[0],ac[1]) for ac in agent_clauses]

    def therblig_to_constraints(self,past,this,states,context):
        constraints = []
        therblig_data = self.therbligs[this]
        therblig_id = therblig_data["id"]
        for preconstraint in self.therbligs[this]["requires"].keys():
            if preconstraint == "agent_destination":
                constraints.extend(self.agent_destination_pre_constraints(past,therblig_data,states,context))
            elif preconstraint == "thing_destination":
                constraints.extend(self.thing_destination_pre_constraints(past,therblig_data,states,context))
            elif preconstraint == "agent_thing":
                constraints.extend(self.agent_thing_pre_constraints(past,therblig_data,states,context))
            elif preconstraint == "grasping":
                constraints.extend(self.agent_gripping_pre_constraints(past,therblig_data,states,context))
            else:
                print("Pre-constraint '{0}' not known!".format(preconstraint))

        if "agent_destination" in self.therbligs[this]["sets"].keys():
            constraints.extend(self.agent_destination_post_constraints(past,therblig_data,states,context))
        else:
            constraints.extend(self.agent_destination_consistency(past,therblig_data,states,context))

        if "thing_destination" in self.therbligs[this]["sets"].keys():
            constraints.extend(self.thing_destination_post_constraints(past,therblig_data,states,context))
        else:
            constraints.extend(self.thing_destination_consistency(past,therblig_data,states,context))


        if "grasping" in self.therbligs[this]["sets"].keys():
            constraints.extend(self.agent_gripping_post_constraints(past,therblig_data,states,context))
        else:
            constraints.extend(self.agent_gripping_consistency(past,therblig_data,states,context))


        return [(therblig_id,field,message,constraint) for field,message,constraint in constraints]



if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No JSON File Provided!")
    else:
        file = sys.argv[1]
        with open(file, "r") as read_file:
            data = convert(json.load(read_file))
        print("######### Checking Plan #########")
        v = Verifier(data)
        start = time.time()
        ec, e = v.check()
        print("Time: {0}".format(time.time()-start))
        print("Error Count: {0}".format(ec))
        print(e)
