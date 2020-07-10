import json
import os
import sys
import re
from pprint import pprint
from authrpddl import unclean_id

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
            if therblig_type in ["transport_loaded","grasp","release_load","release"]:
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

def convert(input):
    if isinstance(input, dict):
        return {convert(key): convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

#translates raw output of MODAE-YAHSP planner to JSON and renames best plan to plan.JSON
def select_plan(file,full):
    best_score = float("inf")
    best_time = float("inf")
    best_cost = float("inf")
    best_plan = None
    best_number = None

    with open(file) as domain:
        domain_data = json.load(domain)
        print("Cost Weight: {0}".format(domain_data["cost_weight"]))
        print("Time Weight: {0}".format(domain_data["time_weight"]))

    plan_counter = 0
    for filename in os.listdir('.'):
    	#looks for planner output files
        if filename.startswith("plan.soln."):
            plan_counter +=1
    	#translates output to JSON
            with open(filename, 'r') as f:
                lines = f.readlines()
                data = {}
                data['plan'] = {}
                counter = 0
                for line in lines:
                    if counter == 2:
                        data['cost'] = float(line.split(' ')[2])
                    if not line.startswith(';'):
                        tokens = line.split(' ')
                        time = float(tokens[0].replace(":",""))
                        therblig_type = tokens[1].replace("(","").replace("-","_")
                        therblig_id = unclean_id(tokens[2],"therblig").split("_")[0]
                        print(therblig_type,therblig_id)
                        params = full["therbligs"][therblig_id]["parameters"]
                        agent = unclean_id(tokens[3],"agent")
                        duration = float(tokens[-2].strip("[]"))
                        cost = float(tokens[-1].strip("[]\n"))
                        therblig_data = {"start_time":time,"duration":duration,"therblig":{"type":therblig_type,"parameters":params}}
                        # print(tokens)
                        if agent not in data["plan"].keys():
                            data["plan"][agent] = []
                        data["plan"][agent].append(therblig_data)
                        # data["plan"][agent].append({})
                        # data['plan'][tokens[1][1:-5]] = {'starttime': re.sub('[\:]', '', tokens[0]),
                        #                                  'agent': tokens[3][0],
                        #                                  'duration': tokens[len(tokens) - 2].strip('[]\n'),
    					# 	     'cost' : tokens[len(tokens)-1].strip('[]\n')}
                        # # print(line)
                        if counter == len(lines) - 2:
                            data['time'] = float(re.sub('[\:]', '', tokens[0])) + float(tokens[len(tokens) - 2].strip('[]\n'))
                    counter += 1
                #print(data["plan"])
                #print_expanded(data["plan"])

    	#saves output
                with open('candidate' + str(plan_counter), 'w') as outfile:
                    json.dump(data, outfile, sort_keys=True, indent=4, separators=(',', ': '))
    	#checks generated JSON against current best plan
            with open('candidate' + str(plan_counter)) as plan:
                plan_data = convert(json.load(plan))
                cost = float(plan_data['cost'])/100.0
                duration = float(plan_data['time'])
                plan_score = (cost * domain_data['cost_weight']) + (duration * domain_data['time_weight'])
                if plan_score < best_score:
                    best_cost = cost# * domain_data['cost_weight']
                    best_time = duration# * domain_data['time_weight']
                    best_score = plan_score
                    best_number = plan_counter
                    best_plan = plan_data

    compute_time = 0
    for filename in os.listdir('.'):
        if filename.startswith("plan.soln"):
            if filename[-1:] == str(best_number):
                continue
            os.remove(filename)
        if filename.startswith("candidate"):
            os.remove(filename)

    os.rename('./plan.soln.'+str(best_number), './plan.soln')
    print_expanded(best_plan["plan"],full)

    return(best_score, best_time, best_cost)
