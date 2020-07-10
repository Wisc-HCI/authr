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


if __name__ == "__main__":
    start = time.time()
    if len(sys.argv) < 2:
        print("No JSON File Provided!")
    else:
        file = sys.argv[1]
        with open(file, "r") as read_file:
            data = convert(json.load(read_file))
        domain,instance= plan_to_pddl(data,goaltype="state")
        try:
            out_root = sys.argv[2]
        except:
            out_root = os.path.dirname(os.path.realpath(__file__))+"/out"
        domainfile = out_root+".domain.pddl"
        instancefile = out_root+".instance.pddl"
        with open(domainfile,"w+") as write_file:
            write_file.write(domain)
        with open(instancefile,"w+") as write_file:
            write_file.write(instance)

        build_plans = os.path.dirname(os.path.realpath(__file__))+"/descarwin/build/dae/dae --domain={0} --instance={1} -r=5 --verbose=progress".format(domainfile,instancefile)

        process = subprocess.Popen(build_plans.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()

        best_score, best_time, best_cost = plan_selector.select_plan(sys.argv[1],data)
        end = time.time()
        #os.remove(domainfile)
        #os.remove(instancefile)

        print("Time: {0}".format(best_time))
        print("Cost: {0}".format(best_cost))
        print("Score: {0}".format(best_score))
        print("TTC: {0}".format(end-start))
