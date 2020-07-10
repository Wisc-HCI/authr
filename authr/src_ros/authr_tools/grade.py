#!/usr/bin/env python
import json
import sys
import time
try:
    from authr_tools.veropt import VerOpt
except:
    from veropt import VerOpt

def convert(input):
    if isinstance(input, dict):
        return {convert(key): convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def get_expected_costs(expansion, plan):
    agent_data = {}
    for agent_id in expansion.keys():
        agent_data[agent_id] = {}
        agent_data[agent_id]["type"] = plan["agents"][agent_id]["type"]
        agent_data[agent_id]["cost"] = 0
    for agent_id in expansion.keys():
        for data in expansion[agent_id]:
            if agent_data[agent_id]["type"] == "robot":
                if data["therblig"]["type"] == "transport_empty":
                    agent_data[agent_id]["cost"] += .05
                elif data["therblig"]["type"] == "transport_loaded":
                    agent_data[agent_id]["cost"] += .05
                elif data["therblig"]["type"] == "grasp":
                    agent_data[agent_id]["cost"] += .01
                elif data["therblig"]["type"] == "release_load":
                    agent_data[agent_id]["cost"] += .01
                elif data["therblig"]["type"] == "rest":
                    agent_data[agent_id]["cost"] += .01
                elif data["therblig"]["type"] == "hold":
                    agent_data[agent_id]["cost"] += .05
            elif agent_data[agent_id]["type"] == "human":
                if data["therblig"]["type"] == "transport_empty":
                    agent_data[agent_id]["cost"] += .1
                elif data["therblig"]["type"] == "transport_loaded":
                    agent_data[agent_id]["cost"] += .1 + object_cost(data["therblig"]["id"], plan)
                elif data["therblig"]["type"] == "grasp":
                    agent_data[agent_id]["cost"] += .05
                elif data["therblig"]["type"] == "release_load":
                    agent_data[agent_id]["cost"] += .05
                elif data["therblig"]["type"] == "hold":
                    agent_data[agent_id]["cost"] += object_cost(data["therblig"]["id"], plan)
    return agent_data

def object_cost(therblig_id, plan):
    thing_id = plan["therbligs"][therblig_id]["parameters"]["thing"]
    if plan["things"][thing_id]["type"] == "cube":
        return .15
    elif plan["things"][thing_id]["type"] == "cylinder":
        return .2
    else:
        return .1


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No JSON File Provided!")
    else:
        file = sys.argv[1]
        with open(file, "r") as read_file:
            data = convert(json.load(read_file))
        print("######### Grading Plan #########")
        start = time.time()
        v = VerOpt(data["agents"].keys(),data["things"].keys(),data["destinations"].keys())
        valid, errors, trace, expansion, plantime, cost, score = v.plan(data)
        end = time.time()
        print("Time: {0}".format(plantime))
        print("Cost: {0}".format(cost))
        print("Score: {0}".format(score))
        print("TTC: {0}".format(end-start))
        expected_costs = get_expected_costs(expansion, data)
        for agent_id in expected_costs.keys():
            print("Expected cost for {0} ({1}): {2}".format(agent_id, expected_costs[agent_id]["type"], expected_costs[agent_id]["cost"]))
