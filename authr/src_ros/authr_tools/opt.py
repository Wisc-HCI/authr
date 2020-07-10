import json
import sys
from copy import deepcopy
import pprint
import itertools

def convert(input):
    if isinstance(input, dict):
        return {convert(key): convert(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [convert(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

def clean_id(id,type):
    return type.upper()+id.replace(".","-DOT-")

def unclean_id(id,type):
    return id.replace("-DOT-",".").replace(type.upper(),"")

def plan_to_agents(plan):
    return " ".join(plan["agents"].keys())

def plan_to_predicates(plan,therblig_ids,therbligs):

    therblig_identities = ['        (therblig_is_{0} ?t - therblig)\n'.format(clean_id(therbligs[id]['eid'],"therblig")) for id in therblig_ids]
    agent_identites = ['        (agent_is_{0} ?a - agent)\n'.format(clean_id(agent,"agent")) for agent in plan["agents"].keys()]

    return """
    (:predicates
        (therblig_ran ?th - therblig)\n{therblig_identities}\n{agent_identites}
        (isRobot ?a - agent)
        (gripping ?a - agent)
        (at ?a - agent ?d - destination)
        (at ?t - thing ?d - destination)
    )
    """.format(therblig_identities="".join(therblig_identities),
               agent_identites="".join(agent_identites))

def plan_to_constants(plan):
    return """
    (:constants
        {destination_constants}
    )
    """.format(destination_constants='')

def plan_to_actions(plan,therblig_ids,therbligs):
    action_string = "  "+"\n\n  ".join([therblig_to_action(therbligs[therblig_id],plan) for therblig_id in therblig_ids])+"\n"
    action_string += "  "+"\n\n  ".join([tof_to_action(pair[0],pair[1],plan["tofs"]) for pair in itertools.permutations(plan["destinations"].keys(),2)])+"\n"
    return action_string

def therblig_to_effects(therblig,destinations):
    effects = []
    for effect,value in therblig["sets"].items():
        if effect == "agent_destination":
            effects.append("(at ?a {dest})".format(dest=clean_id(therblig["parameters"]["destination"],"dest")))
            for dest in destinations:
                if dest != therblig["parameters"]["destination"]:
                    effects.append(unset_dest_clause("?a",clean_id(dest,"dest")))
        elif effect == "thing_destination":
            effects.append("(at {thing} {dest})".format(thing=clean_id(therblig["parameters"]["thing"],"thing"),
                                                        dest=clean_id(therblig["parameters"]["destination"],"dest")))
            for dest in destinations:
                if dest != therblig["parameters"]["destination"]:
                    effects.append(unset_dest_clause(clean_id(therblig["parameters"]["thing"],"thing"),clean_id(dest,"dest")))
        elif effect == "grasping" and value == True:
            effects.append("(gripping ?a)")
        elif effect == "grasping" and value == False:
            effects.append("(not (gripping ?a))")

    # therblig tracking
    effects.append("(therblig_ran ?thr)")

    return " ".join(effects)

def therblig_to_conditions(therblig,destinations,agent):
    conds = []
    for cond,value in therblig["requires"].items():
        if cond == "agent_destination":
            conds.append("(at ?a {dest})".format(dest=clean_id(therblig["parameters"]["destination"],"dest")))
        elif cond == "thing_destination":
            conds.append("(at {thing} {dest})".format(thing=clean_id(therblig["parameters"]["thing"],"thing"),
                                                      dest=clean_id(therblig["parameters"]["destination"],"dest")))
        elif cond == "grasping":
            conds.append("(gripping ?a)")
        elif cond == "agent_thing":
            for dest in destinations:
                conds.append(agent_thing_clause(dest))

    # therblig tracking
    conds.append("(therblig_is_{0} ?thr)".format(clean_id(therblig['eid'],"therblig")))
    conds.append("(not (therblig_ran ?thr))")

    # agent identity
    conds.append("(agent_is_{0} ?a)".format(clean_id(agent,"agent")))

    return " ".join(conds)

def therblig_to_params(therblig):

    params = []
    params.append("?thr - therblig ")
    params.append("?a - agent ")

    return " ".join(params)

def therblig_to_duration(therblig,agent):
    if ['any'] == therblig['duration'].keys():
        duration = therblig['duration']['any']
    else:
        duration = therblig['duration'][agent]

    return '{0}'.format(duration)

def therblig_to_cost(therblig,agent):
    if therblig['cost'][agent]:
        return '{0}'.format(therblig['cost'][agent])
    else:
        return '0'

def agent_thing_clause(dest):
    return "(= (at ?a {dest}) (at ?t {dest}))".format(dest=clean_id(dest,"dest"))

def unset_dest_clause(obj, dest):
    return "(not (at {obj} {dest}))".format(obj=obj,dest=dest)

def therblig_to_action(therblig,plan):

    actions = []

    if therblig["parameters"]["agent"] == None or therblig["parameters"]["agent"] == "OPTIMIZE_DIRECTIVE":
        agents = list(plan["agents"].keys())
    else:
        agents = [therblig["parameters"]["agent"]]

    for agent in agents:
        actions.append("""(:durative-action {therblig_id}
            :parameters ({parameters})
            :duration (= ?duration {duration})
            :precondition (and {condition})
            :effect (and {effects} (increase (total-cost) {cost} ) )\n  )\n""".format(therblig_id=clean_id(therblig['eid'],"therblig")+"_AGENT_"+clean_id(agent,"agent"),
                      parameters=therblig_to_params(therblig),
                      duration=therblig_to_duration(therblig,agent),
                      condition=therblig_to_conditions(therblig,plan["destinations"].keys(),agent),
                      effects=therblig_to_effects(therblig,plan["destinations"].keys()),
                      cost=therblig_to_cost(therblig,agent)))

    return "".join(actions)

def tof_to_action(dest1,dest2,tof_table):
    return """(:durative-action {tof_id}
        :duration (= ?duration {duration})\n  )""".format(tof_id=clean_id(dest1+"_"+dest2,"TOF"),
                duration=tof_table[dest1+"_"+dest2])

def plan_to_objects(plan,therblig_ids,therbligs):
    objects = []
    objects.append("        ({agents} - agent)".format(agents=" ".join([clean_id(agent,"agent") for agent in plan["agents"].keys()])))
    objects.append("        ({things} - thing)".format(things=" ".join([clean_id(thing,"thing") for thing in plan["things"].keys()])))
    objects.append("        ({destinations} - destination)".format(destinations=" ".join([clean_id(destination,"dest") for destination in plan["destinations"].keys()])))
    objects.append("        ({therbligs} - therblig)".format(therbligs=" ".join([clean_id(therbligs[id]['eid'],"therblig") for id in therblig_ids])))
    return "\n".join(objects)

def plan_to_init(plan,therblig_ids,therbligs):
    states = []

    for agent in plan["agents"].keys():
        aid = clean_id(agent,"agent")
        states.append("        (at {id1} {id2})".format(id1=aid,id2=clean_id(agent,"dest")))
        states.append("        (gripping {id})".format(id=aid))
        states.append("        (agent_is_{id1} {id2})".format(id1=aid,id2=aid))
        if plan["agents"][agent]["type"] == "robot":
            states.append("        (isRobot {id})".format(id=aid))

    for thing in plan["things"].keys():
        states.append("        (at {id1} {id2})".format(id1=clean_id(thing,"thing"),id2=clean_id(thing,"dest")))

    for id in therblig_ids:
        eid = clean_id(therbligs[id]['eid'],'therblig')
        states.append("        (therblig_is_{id1} {id2})".format(id1=eid,id2=eid))

    return "\n".join(states)

def plan_to_goal(therblig_ids,therbligs):
    states = []

    for id in therblig_ids:
        states.append("(therblig_ran {0})".format(clean_id(therbligs[id]['eid'],'therblig')))

    return " ".join(states)

def plan_to_pddl(plan):

    # Unroll the plan
    therblig_ids = []
    therbligs = {}
    for task_key in plan["task_keys"]:
        for repeat in range(plan["tasks"][task_key]["repeat"] + 1):
            for therblig in plan["tasks"][task_key]["therbligs"]:
                eid = therblig+"_{0}".format(repeat)
                therblig_ids.append(eid)
                therblig_obj = deepcopy(plan["therbligs"][therblig])
                therblig_obj["eid"] = eid
                therblig_obj['repeat'] = repeat
                therblig_obj["is_repeated"] = plan["tasks"][task_key]["repeat"] > 0
                therbligs[eid] = therblig_obj

    domain = """(define (domain {name})
        (:requirements :durative-actions :action-costs :typing :equality)
        (:types agent thing destination therblig)
        {predicates}
        \n{actions}\n)""".format(name=plan["name"].replace(" ",""),
                                 predicates=plan_to_predicates(plan,therblig_ids,therbligs),
                                 actions=plan_to_actions(plan,therblig_ids,therbligs))

    instance = """(define (problem assignment)
      (:domain {name})
      (:objects\n{objects}
      )
      (:init\n{init}
      )
      (:goal (and {goal}))

      (:metric (and (minimize (total-time))(minimize(total-cost))))\n)
      """.format(name=plan["name"].replace(" ",""),
                 objects=plan_to_objects(plan,therblig_ids,therbligs),
                 init=plan_to_init(plan,therblig_ids,therbligs),
                 goal=plan_to_goal(therblig_ids,therbligs))

    return domain,instance

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No JSON File Provided!")
    else:
        file = sys.argv[1]
        with open(file, "r") as read_file:
            data = convert(json.load(read_file))
        d,i = plan_to_pddl(data)
        print(d)
        print
        print(i)
