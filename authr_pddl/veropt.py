#!/usr/bin/env python
import json
import sys
from enum import Enum
import itertools
from copy import deepcopy
import pprint
import math
import time
try:
    from authr_tools.expansion import Expander
except:
    from expansion import Expander

def convert(input):
    if isinstance(input, dict):
        return {convert(key): convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def powerset(iterable):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    s = list(iterable)
    return itertools.chain.from_iterable(itertools.combinations(s, r) for r in range(len(s)+1))

class Trace(object):
    """
    A Class representing the execution steps performed
    """
    def __init__(self):
        self.actions = []
        self.successful = False
        self.time = 0
        self.cost = 0

    def add(self,action,time,cost):
        self.actions.append(action)
        self.time += time
        self.cost += cost

    def therbligs(self):
        return self.actions

    def __repr__(self):
        return " -> \n".join(["\t{0}({1})".format(t["type"],t["parameters"]["agent"]) for t in self.actions]) + "\n\t(time: {0}, cost: {1})".format(self.time,self.cost)

class State(object):
    """
    A Class representing a single state of the plan.
    """
    def __init__(self,attributes,is_initial=False):
        self.attributes = attributes
        self.is_initial = is_initial

    def __repr__(self):
        return pprint.pformat(self.attributes)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.attributes == other.attributes
        else:
            return False

    def transition(self,changes,trace,therblig,agent,tofs,agents):
        time = 0
        cost = 0

        attr = deepcopy(self.attributes)
        if therblig["type"] == "transport_empty":
            if attr["locations"][agent] == therblig["parameters"]["destination"]:
                return (State(attr,False),trace)
        if therblig["type"] == "transport_loaded":
            if attr["locations"][agent] == therblig["parameters"]["destination"] and attr["locations"][therblig["parameters"]["thing"]] == therblig["parameters"]["destination"]:
                return (State(attr,False),trace)
        new_trace = deepcopy(trace)
        local_therblig = deepcopy(therblig)
        local_therblig["parameters"]["agent"] = agent
        if list(local_therblig["duration"].keys()) == ["Any"]:
            time += local_therblig["duration"]["Any"]
        elif local_therblig["duration"][agent] != None:
            time += local_therblig["duration"][agent]
        else:
            if "agent_destination" in local_therblig["sets"].keys():
                time += tofs[self.dest(local_therblig["parameters"]["agent"])+"_"+local_therblig["sets"]["agent_destination"]]
            elif "thing_destination" in local_therblig["sets"].keys():
                time += tofs[self.dest(local_therblig["parameters"]["thing"])+"_"+local_therblig["sets"]["thing_destination"]]

        if "grasping" in local_therblig["sets"].keys():
            time += math.fabs(self.grasp(agent)-changes["gripping"][agent])
        if therblig["cost"][agent]:
            cost += therblig["cost"][agent]
        new_trace.add(local_therblig,time,cost)
        local_therblig["duration"] = time
        local_therblig["cost"] = cost

        if agents[agent]["type"] == "robot":
            if "agent_destination" in local_therblig["sets"].keys():
                local_therblig["trajectory_id"] = self.dest(local_therblig["parameters"]["agent"])+"_"+local_therblig["sets"]["agent_destination"]
            elif "thing_destination" in local_therblig["sets"].keys():
                local_therblig["trajectory_id"] = self.dest(local_therblig["parameters"]["thing"])+"_"+local_therblig["sets"]["thing_destination"]

        attr["locations"].update(changes["locations"])
        attr["gripping"].update(changes["gripping"])
        return (State(attr,False),new_trace)

    def dest(self,element):
        if element in self.attributes["locations"].keys():
            return self.attributes["locations"][element]
        else:
            return None

    def grasp(self,element):
        if element in self.attributes["gripping"].keys():
            return self.attributes["gripping"][element]
        else:
            return None

    def is_valid(self,agents,things):
        #return True # Currently Disabled for parity with standard verification
        dest_count = {}
        valid = True
        for element,destination in self.attributes["locations"].items():
            if destination not in dest_count.keys():
                dest_count[destination] = {"agents":0,"things":0}
            if element in agents:
                dest_count[destination]["agents"] += 1
                if dest_count[destination]["agents"] > 2:
                    valid = False
            elif element in things:
                dest_count[destination]["things"] += 1
                if dest_count[destination]["things"] > 2:
                    valid = False
        return valid

def sets_to_changes(therblig,agent=None):
    has_agent = True
    has_thing = True
    has_dest = True
    if (therblig["parameters"]["agent"] == "OPTIMIZE_DIRECTIVE" or therblig["parameters"]["agent"] == None) and agent != None:
        pass
    elif therblig["parameters"]["agent"] != None:
        agent = therblig["parameters"]["agent"]
    else:
        has_agent = False

    if "thing" in therblig["parameters"].keys() and therblig["parameters"]["thing"] !=  None:
        thing = therblig["parameters"]["thing"]
    else:
        has_thing = False

    if "destination" in therblig["parameters"].keys() and therblig["parameters"]["destination"] !=  None:
        dest = therblig["parameters"]["destination"]
    else:
        has_dest = False

    changes = {"locations":{},"gripping":{}}
    for sets,value in therblig['sets'].items():
        if sets == "agent_destination" and has_agent and has_dest:
            changes["locations"][agent] = dest
        if sets == "thing_destination" and has_thing and has_dest:
            changes["locations"][thing] = dest
        if sets == "grasping" and has_agent:
            if value == True and therblig['parameters']["effort"] != None:
                changes["gripping"][agent] = therblig['parameters']["effort"]
            elif value == True:
                changes['gripping'][agent] = 1.0
            else:
                changes["gripping"][agent] = 0.0
    return changes

def check_state(state,therblig,agent=None):
    if therblig["parameters"]["agent"] != None and agent == None:
        agent = therblig["parameters"]["agent"]
    valid = True
    errors = {"agent_destination_set":None,"agent_destination_require":None,
              "thing_destination_set":None,"thing_destination_require":None,
              "agent_gripping_set":None,"agent_gripping_require":None,
              "agent_thing_require":None}
    for req,value in therblig["requires"].items():
        if req == "agent_destination" and state.dest(agent) != therblig["parameters"]["destination"]:
            valid = False
            #print(state.dest(agent),therblig["parameters"]["destination"])
            errors["agent_destination_require"] = "Agent must be at the given destination."
        if req == "thing_destination" and state.dest(therblig["parameters"]["thing"]) != therblig["parameters"]["destination"]:
            valid = False
            errors["thing_destination_require"] = "Thing must be at the given destination."
        if req == "agent_thing" and state.dest(agent) != state.dest(therblig["parameters"]["thing"]):
            valid = False
            errors["agent_thing_require"] = "Locations of Agent and Thing must match"
        if req == "grasping" and value == True and "effort" in therblig["parameters"].keys() and therblig["parameters"]["effort"] != None:
            if state.grasp(agent) != therblig["parameters"]["effort"]:
                valid = False
                errors["agent_gripping_require"] = "Agent must be gripping with the same force."
        elif req == "grasping" and value == True and "effort" in therblig["parameters"].keys():
            # Default to Effort 1.0 if not defined
            if state.grasp(agent) != 1.0:
                valid = False
                errors["agent_gripping_require"] = "Agent must be gripping."
        elif req == 'grasping' and value == False and state.grasp(agent) != 0.0:
            valid = False
            errors["agent_gripping_require"] = "Agent must not be gripping."

    return valid, errors



class StateMachine(object):
    """
    A Class representing the state space tree
    """
    def __init__(self,agents,things,destinations):

        self.agents = agents
        self.things = things
        self.destinations = destinations
        # self.valid_states = []

        # Create initial state:
        attributes = {"locations":{},"gripping":{}}
        attributes["locations"].update({agent:agent for agent in self.agents})
        attributes["locations"].update({thing:thing for thing in self.things})
        attributes['gripping'].update({agent:0.0 for agent in self.agents})
        self.initial_state = State(attributes,True)

    def check(self,plan):
        active_states = [(self.initial_state,Trace())]
        all_errors = {}
        for therblig_id in plan.therblig_ids:
            all_errors[therblig_id] = {}
            print(len(active_states))
            new_active_states = []
            therblig = plan.therbligs[therblig_id]
            if therblig["parameters"]["agent"] == "OPTIMIZE_DIRECTIVE" or therblig["parameters"]["agent"] == None:
                agents = self.agents
            else:
                agents = [therblig["parameters"]["agent"]]
            for agent in agents:
                changes = sets_to_changes(therblig,agent)
                for (state,trace) in active_states:
                    has_error, errors = check_state(state,therblig,agent)
                    if all_errors[therblig_id] == {}:
                        all_errors[therblig_id] = errors
                    else:
                        for key in errors.keys():
                            if all_errors[therblig_id][key] is None:
                                all_errors[therblig_id][key] == errors[key]
                    if has_error:
                        (new_state,new_trace) = state.transition(changes,trace,therblig,agent,plan.tofs,plan.agents)
                        #print("state requirements matched")
                        if new_state.is_valid(self.agents,self.things):
                            new_active_states.append((new_state,new_trace))
                    #     else:
                    #         print("resulting state not valid")
                    # else:
                    #     print("state requirements not matched")
                    #     print(errors)

            active_states = new_active_states
        return len(active_states) >= 1, all_errors, [s[1] for s in active_states]

class Plan(object):
    """
    A Class representing the plan
    """
    def __init__(self,data):
        self.data = data
        self.therblig_ids = []
        self.therbligs = {}
        self.tofs = data["tofs"]
        self.agents = data["agents"]
        #Expand Therbligs
        for task_key in self.data["task_keys"]:
            for repeat in range(self.data["tasks"][task_key]["repeat"] + 1):
                for therblig in self.data["tasks"][task_key]["therbligs"]:
                    eid = therblig+"_{0}".format(repeat)
                    self.therblig_ids.append(eid)
                    therblig_obj = deepcopy(self.data["therbligs"][therblig])
                    therblig_obj["eid"] = eid
                    therblig_obj['repeat'] = repeat
                    therblig_obj["is_repeated"] = self.data["tasks"][task_key]["repeat"] > 0
                    self.therbligs[eid] = therblig_obj
        self.therblig_count = len(self.therblig_ids)

def choose_trace(traces, time_weight, cost_weight, plan):
    best_score = None
    best_trace = None
    best_expansion = None
    best_time = 0
    best_cost = 0
    for trace in traces:
        e = Expander(plan, trace)
        expanded = e.calculate()
        duration = 0
        for agent in expanded:
            for data in expanded[agent]:
                if data["start_time"] + data["duration"] > duration:
                    duration = data["start_time"] + data["duration"]
        score = duration * time_weight + trace.cost * cost_weight
        if best_score == None or score < best_score:
            best_score = score
            best_trace = trace
            best_expansion = expanded
            best_time = duration
            best_cost = trace.cost
    return best_trace, best_expansion, best_time, best_cost, best_score

class VerOpt(object):
    def __init__(self, agents,things,destinations):
        self.state_machine = StateMachine(agents,destinations,destinations)

    def plan(self, data):
        time, cost, score = (0,0,0)
        pl = Plan(data)
        valid, errors, traces = self.state_machine.check(pl)
        #print(valid,errors,len(traces))
        if valid:
            trace, expansion, time, cost, score = choose_trace(traces, data["time_weight"], data["cost_weight"], data)
        else:
            trace = None
            expansion = {agent:[] for agent in data["agents"].keys()}
        return valid, errors, trace, expansion, time, cost, score


def param_check(data,enforce_agents=False):
    """Checks whether a plan is sufficiently parametrized.

    Parameters
    ----------
    data : dict
        A dictionary/serialized version of a plan.
    enforce_agents : bool
        Whether to enforce that agents are parametrized.
        Non-parametrized agents are allowed in optimization.

    Returns
    -------
    int
        Number of unique errors.
    dict
        Issues indexed by type/key.
    """
    errors = 0
    info = {"general":{}}

    # Check general properties
    if len(data["tasks"]) == 0:
        info["general"]["tasks"] = "No tasks defined"
        errors += 1
    else:
        therblig_count = 0
        for task_key in data["task_keys"]:
            therblig_count += len(data["tasks"][task_key]["therbligs"])
        if therblig_count == 0:
            info["general"]["therbligs"] = "No therbligs defined"
            errors += 1
    if len(data["agents"]) == 0:
        info["general"]["agents"] = "No agents defined"
        errors += 1
    if len(data["things"]) == 0:
        info["general"]["things"] = "No things defined"
        errors += 1
    if len(data["destinations"]) == 0:
        info["general"]["destinations"] = "No destinations defined"
        errors += 1
    if len(data["tofs"]) == 0:
        info["general"]["tofs"] = "No tofs defined"
        errors += 1
    else:
        for tof_key,tof_value in data["tofs"].items():
            if tof_value in ["error", "pending", "invalid"]:
                if "tofs" not in info.keys():
                    info["tofs"] = {}
                info["tofs"][tof_key] = "Invalid value for TOF"
                errors += 1


    for task_key in data["task_keys"]:
        for therblig_key in data["tasks"][task_key]["therbligs"]:
            info[therblig_key] = {}
            therblig = data["therbligs"][therblig_key]
            # Check Parameters
            for parameter, value in therblig["parameters"].items():
                if parameter == "agent" and value == None and enforce_agents:
                    info[therblig_key]["agent"] = "Agent not specified"
                    errors += 1
                elif parameter == "arm" and value == None and enforce_agents and therblig["parameters"]["agent"] != None:
                    info[therblig_key]["arm"] = "Arm not specified"
                    errors += 1
                elif parameter == "thing" and value == None:
                    info[therblig_key]["thing"] = "Thing not specified"
                    errors += 1
                elif parameter == "destination" and value == None:
                    info[therblig_key]["destination"] = "Destination not specified"
                    errors += 1
                elif parameter == "effort" and value == None:
                    info[therblig_key]["effort"] = "Effort not specified"
                    errors += 1

            # Check Durations
            if list(therblig["duration"].keys()) == ["Any"]:
                # Universal therblig (rest/hold)
                if therblig["duration"]["Any"] == None:
                    info[therblig_key]["duration"] = "Explicit duration not assigned."
                    errors += 1
            elif therblig["parameters"]["agent"] != None and therblig["parameters"]["agent"] != "OPTIMIZE_DIRECTIVE" and data["agents"][therblig["parameters"]["agent"]]["type"] == "human":
                # User has explicitly has set a human to perform the therblig
                if therblig["duration"][therblig["parameters"]["agent"]] == None:
                    info[therblig_key]["duration"] = "Human is assigned, but duration is not specified."
                    errors += 1
            elif therblig["parameters"]["agent"] != None and therblig["parameters"]["agent"] != "OPTIMIZE_DIRECTIVE" and data["agents"][therblig["parameters"]["agent"]]["type"] == "robot":
                # User has explicitly has set a robot to perform the therblig
                pass # Duration is obtained by looking up in the TOF Table.
            else:
                # Therblig is optimized. Ensure that all values are defined when human.
                for agent, duration in therblig["duration"].items():
                    if agent == "Any":
                        pass
                    elif data["agents"][agent]["type"] == "human" and duration == None:
                        info[therblig_key]["duration"] = "Optimization requires durations for human agents."
                        errors += 1

            # Check Costs
            pass # Actually don't need to set. None is converted to 0 by default.


    return errors, info

def print_expanded(expanded,full):
    import pandas
    import numpy as np
    agents = expanded.keys()
    empty = False
    indices = []
    for agent in agents:
        indices.extend([th["start_time"] for th in expanded[agent]])
        indices.extend([th["start_time"]+th["duration"] for th in expanded[agent]])
    indices = sorted(list(set(indices)))
    plan = pandas.DataFrame(index=indices,columns=[full["agents"][agent]["name"] for agent in agents])
    for agent in agents:
        agent_name = full["agents"][agent]["name"]
        # Add in completions first, so they get overwritten if a new therblig starts
        for therblig in expanded[agent]:
            end_time = therblig["start_time"] + therblig["duration"]
            plan.loc[end_time,agent_name] = "-"

        # Add in starts
        for therblig in expanded[agent]:
            start_time = therblig["start_time"]
            therblig_type = therblig["therblig"]["type"]
            params = []
            if therblig_type in ["transport_loaded","grasp","release_load"]:
                thing_id = therblig["therblig"]["parameters"]["thing"]
                thing_name = full["things"][thing_id]["name"]
                params.append(thing_name)
            if therblig_type in ["transport_loaded","transport_empty"]:
                destination_id = therblig["therblig"]["parameters"]["destination"]
                destination_name = full["destinations"][destination_id]["name"]
                params.append(destination_name)
            plan.loc[start_time,agent_name] = therblig_type+"({0})".format(", ".join(params))

        # Fill in missing spots
        running = False
        for i,row in plan.iterrows():
            if pandas.isna(row[agent_name]) and running == False:
                plan.loc[i,agent_name] = " "
            if pandas.notna(row[agent_name]) and running == False and row[agent_name] not in ["|"," ","-"]:
                running = True
            if pandas.isna(row[agent_name]) and running == True:
                plan.loc[i,agent_name] = "|"
            if row[agent_name] == "-":
                running = False
    print(plan)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No JSON File Provided!")
    else:
        file = sys.argv[1]
        with open(file, "r") as read_file:
            data = convert(json.load(read_file))
        print("######### Checking/Optimizing Plan #########")
        print(param_check(data))
        start = time.time()
        v = VerOpt(data["agents"].keys(),data["things"].keys(),data["destinations"].keys())
        valid, errors, trace, expansion, plantime, plancost, planscore = v.plan(data)
        end = time.time()
        #print("Valid: {0}".format(valid))
        #print("Errors: {0}".format(errors))
        #print("Trace: {0}".format(trace))
        print_expanded(expansion,data)
        print("Time: {0}".format(plantime))
        print("Cost: {0}".format(plancost))
        print("Score: {0}".format(planscore))
        print("TTC: {0}".format(end-start))
