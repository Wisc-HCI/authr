import csv
from veropt import VerOpt
import json
import sys
from authrpddl import plan_to_pddl
import subprocess
import plan_selector
import os
import time

def convert(input):
    if isinstance(input, dict):
        return {convert(key): convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def pddlopt(file,max_time):
    start = time.time()
    with open(file, "r") as read_file:
        data = convert(json.load(read_file))
    domain,instance= plan_to_pddl(data,goaltype="state")
    out_root = os.path.dirname(os.path.realpath(__file__))+"/out"
    domainfile = out_root+".domain.pddl"
    instancefile = out_root+".instance.pddl"
    with open(domainfile,"w+") as write_file:
        write_file.write(domain)
    with open(instancefile,"w+") as write_file:
        write_file.write(instance)
    proc = time.time()
    build_plans = os.path.dirname(os.path.realpath(__file__))+"/descarwin/build/dae/dae --domain={0} --instance={1} -i={2} --verbose=progress".format(domainfile,instancefile,90-(proc-start))

    process = subprocess.Popen(build_plans.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

    best_score, best_time, best_cost = plan_selector.select_plan(file,data)
    end = time.time()
    #os.remove(domainfile)
    #os.remove(instancefile)

    return best_time, best_cost, best_score, (end-start)

def veropt(file):
    with open(file, "r") as read_file:
        data = convert(json.load(read_file))
    print("######### Grading Plan #########")
    start = time.time()
    v = VerOpt(data["agents"].keys(),data["things"].keys(),data["destinations"].keys())
    valid, errors, trace, expansion, plantime, cost, score = v.plan(data)
    end = time.time()
    return plantime, cost, score, (end-start)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Provide the number of pddlopt iterations')
    elif len(sys.argv) < 3:
        print('Provide one or more plans to compute')
    pddl_iterations = int(sys.argv[1])

    with open('plan.csv', mode='w') as plan_file:
        plan = csv.writer(plan_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        plan.writerow(['planner', 'plan', 'time', 'cost', 'score', 'compute time'])

        for i in range(len(sys.argv) - 2):
            v_time, v_cost, v_score, v_compute_time = veropt(sys.argv[2+i])
            plan.writerow(['veropt', sys.argv[2+i], v_time, v_cost, v_score, v_compute_time])

            for j in range(pddl_iterations):
                p_time, p_cost, p_score, p_compute_time = pddlopt(sys.argv[2+i],v_time)
                plan.writerow(['pddlopt', sys.argv[2+i], p_time, p_cost, p_score, p_compute_time])
