from copy import deepcopy
import pprint
import itertools
from dest_whitelist import in_transport_wl, in_release_wl

def clean_id(id,type):
    return type.upper()+id.replace(".","-DOT-")

def unclean_id(id,type):
    return id.replace("-DOT-",".").replace("-dot-",".").replace(type.upper(),"").replace(type+"0","0")

def plan_to_objects(plan,therblig_ids,therbligs):
    objects = []
    objects.append("        {agents} - agent".format(agents=" ".join([clean_id(agent,"agent") for agent in plan["agents"].keys()])))
    objects.append("        {things} - thing".format(things=" ".join([clean_id(thing,"thing") for thing in plan["things"].keys()])))
    objects.append("        {destinations} - destination".format(destinations=" ".join([clean_id(destination,"dest") for destination in plan["destinations"].keys()])))
    sorted_therbligs = {"transport_empty":[],"transport_loaded":[],"grasp":[],"release_load":[]}
    for id in therblig_ids:
        sorted_therbligs[therbligs[id]["type"]].append(id)
    objects.append("        {therbligs} - te".format(therbligs=" ".join([clean_id(therbligs[id]['eid'],"therblig") for id in sorted_therbligs["transport_empty"]])))
    objects.append("        {therbligs} - tl".format(therbligs=" ".join([clean_id(therbligs[id]['eid'],"therblig") for id in sorted_therbligs["transport_loaded"]])))
    objects.append("        {therbligs} - g ".format(therbligs=" ".join([clean_id(therbligs[id]['eid'],"therblig") for id in sorted_therbligs["grasp"]])))
    objects.append("        {therbligs} - rl".format(therbligs=" ".join([clean_id(therbligs[id]['eid'],"therblig") for id in sorted_therbligs["release_load"]])))
    return "\n".join(objects)

def plan_to_init(plan,therblig_ids,therbligs):
    states = []

    for agent in plan["agents"].keys():
        aid = clean_id(agent,"agent")
        states.append("        (at {id1} {id2})".format(id1=aid,id2=clean_id(agent,"dest")))
        states.append("        (not (gripping {id}))".format(id=aid))
        states.append("        (not-gripping {id})".format(id=aid))

    for thing in plan["things"].keys():
        states.append("        (at {id1} {id2})".format(id1=clean_id(thing,"thing"),id2=clean_id(thing,"dest")))

    for dest in plan["destinations"].keys():
        did = clean_id(dest,"dest")
        if dest in plan["agents"].keys():
            states.append("        (not (agent-space {id}))".format(id=did))
        else:
            states.append("        (agent-space {id})".format(id=did))
        if dest in plan["things"].keys():
            states.append("        (not (thing-space {id}))".format(id=did))
        else:
            states.append("        (thing-space {id})".format(id=did))

    for id in therblig_ids:
        states.append("        (not-executed {id})".format(id=clean_id(therbligs[id]['eid'],'therblig')))

    for therblig, destination, agent in itertools.product(therblig_ids,plan["destinations"].keys(),plan["agents"].keys()):
        therblig_obj = plan["therbligs"][therblig[:-2]]
        if therblig_obj["type"] in ["transport_empty","transport_loaded"]:
            if in_transport_wl(destination,therblig_obj["parameters"]["destination"]):
                if plan["agents"][agent]["type"] == "robot":
                    states.append("        (= (therblig-time {} {} {} {}) {})".format(clean_id(therblig, 'therblig'), clean_id(agent, 'agent'), clean_id(destination, 'dest'), clean_id(therblig_obj["parameters"]["destination"], 'dest'), plan["tofs"][destination + "_" + therblig_obj["parameters"]["destination"]]))
                else:
                    states.append("        (= (therblig-time {} {} {} {}) {})".format(clean_id(therblig, 'therblig'), clean_id(agent, 'agent'), clean_id(destination, 'dest'), clean_id(therblig_obj["parameters"]["destination"], 'dest'), therblig_obj["duration"][agent]))
        elif therblig_obj["type"] == "release_load":
            if in_release_wl(destination):
                states.append("        (= (therblig-time {} {} {} {}) {})".format(clean_id(therblig, 'therblig'), clean_id(agent, 'agent'), clean_id(destination, 'dest'), clean_id(destination, 'dest'), .5))
        else:
            states.append("        (= (therblig-time {} {} {} {}) {})".format(clean_id(therblig, 'therblig'), clean_id(agent, 'agent'), clean_id(destination, 'dest'), clean_id(destination, 'dest'), .5))

    for therblig, agent in itertools.product(therblig_ids,plan["agents"].keys()):
        states.append("        (= (therblig-cost {} {}) {})".format(clean_id(therblig, 'therblig'), clean_id(agent, 'agent'), plan["therbligs"][therblig[:-2]]["cost"][agent] * 100))

    for therblig in therblig_ids:
        if plan["therbligs"][therblig[:-2]]["parameters"]["agent"] == "OPTIMIZE_DIRECTIVE":
            for agent in plan["agents"].keys():
                states.append("        (agent-allocation {} {})".format(clean_id(therblig, 'therblig'), clean_id(agent, 'agent')))

    for therblig in therblig_ids:
        if "thing" in plan["therbligs"][therblig[:-2]]["parameters"]:
            states.append("        (thing-allocation {} {})".format(clean_id(therblig, 'therblig'), clean_id(plan["therbligs"][therblig[:-2]]["parameters"]["thing"], 'thing')))

    for therblig in therblig_ids:
        if "destination" in plan["therbligs"][therblig[:-2]]["parameters"]:
            states.append("        (destination-allocation {} {})".format(clean_id(therblig, 'therblig'), clean_id(plan["therbligs"][therblig[:-2]]["parameters"]["destination"], 'dest')))

    for agent in plan["agents"]:
        states.append("        (free {})".format(clean_id(agent, 'agent')))

    states.append("        (= (total-cost) 0)")
    return "\n".join(states)

def plan_to_therblig_goal(therblig_ids,therbligs):
    states = []

    for id in therblig_ids:
        states.append("(not (not-executed {0}))".format(clean_id(therbligs[id]['eid'],'therblig')))

    return " ".join(states)

def plan_to_state_goal(plan,therblig_ids,therbligs):
    thing_ats = {thing_id: thing_id for thing_id in plan["things"]}
    grasping = 0
    for therblig in therblig_ids:
        if therbligs[therblig]["type"] == "transport_loaded":
            thing_ats[therbligs[therblig]["parameters"]["thing"]] = therbligs[therblig]["parameters"]["destination"]
        elif therbligs[therblig]["type"] == "grasp":
            grasping += 1
        elif therbligs[therblig]["type"] == "release_load":
            grasping -= 1
    thing_goal_vec = []

    goal = ""

    for thing_id in thing_ats:
        goal += "(at {} {})".format(clean_id(thing_id, "thing"), clean_id(thing_ats[thing_id], "dest"))

    if grasping == 0:
        for agent in plan["agents"]:
            goal += "(not-gripping {0})".format(clean_id(agent,"agent"))

    return goal

def plan_to_pddl(plan,goaltype="therblig"):

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

    if goaltype == "therblig":
        goal = plan_to_therblig_goal(therblig_ids,therbligs)
    else:
        goal = plan_to_state_goal(plan,therblig_ids,therbligs)


    domain = """(define (domain authr)
            (:requirements :durative-actions :action-costs :fluents :typing :equality)
            (:types element - object
                    agent thing destination - element
                    therblig - object
                    te tl g rl r h - therblig
            )

        (:predicates
            (free ?a - agent)
            (gripping ?a - agent)
            (not-gripping ?a - agent)
            (agent-space ?destination)
            (thing-space ?destination)
            (at ?e - element ?d - destination)
            (not-executed ?th - therblig)
            (agent-allocation ?th - therblig ?a - agent)
            (thing-allocation ?th - therblig ?t - thing)
            (destination-allocation ?th - therblig ?d - destination)
        )

        (:functions
              (total-cost)
              (agent-allocation ?a - agent ?th - therblig)
              (thing-allocation ?t - thing ?th - therblig)
              (destination-allocation ?d - destination ?th - therblig)
              (therblig-cost ?th - therblig ?a - agent)
              (therblig-time ?th - therblig ?a - agent ?dcurrent - destination ?dnew - destination)
        )

        (:durative-action transport-empty
            :parameters (?th - te ?a - agent ?dcurrent ?dnew - destination)
            :duration (= ?duration (therblig-time ?th ?a ?dcurrent ?dnew))
            :condition (and (at start (free ?a))
                            (at start (not-executed ?th))
                            (at start (agent-allocation ?th ?a))
                            (at start (destination-allocation ?th ?dnew))
                            (at start (at ?a ?dcurrent))
                            (at start (not-gripping ?a))
                            (at end (agent-space ?dnew))
                       )
            :effect (and (at start (not (free ?a)))
                         (at start (not (not-executed ?th)))
                         (at start (not (at ?a ?dcurrent)))
                         (at start (agent-space ?dcurrent))
                         (at end (at ?a ?dnew))
                         (at end (free ?a))
                         (at end (not (agent-space ?dnew)))
                         (at end (increase (total-cost) (therblig-cost ?th ?a)))
                     )
        )

        (:durative-action transport-loaded
            :parameters (?th - tl ?a - agent ?t - thing ?dcurrent ?dnew - destination)
            :duration (= ?duration (therblig-time ?th ?a ?dcurrent ?dnew))
            :condition (and (at start (free ?a))
                            (at start (not-executed ?th))
                            (at start (agent-allocation ?th ?a))
                            (at start (thing-allocation ?th ?t))
                            (at start (destination-allocation ?th ?dnew))
                            (at start (at ?a ?dcurrent))
                            (at start (at ?t ?dcurrent))
                            (at start (gripping ?a))
                            (at end (agent-space ?dnew))
                            (at end (thing-space ?dnew))
                       )
            :effect (and (at start (not (free ?a)))
                         (at start (not (not-executed ?th)))
                         (at start (not (at ?a ?dcurrent)))
                         (at start (not (at ?t ?dcurrent)))
                         (at start (agent-space ?dcurrent))
                         (at start (thing-space ?dcurrent))
                         (at end (at ?a ?dnew))
                         (at end (at ?t ?dnew))
                         (at end (not (agent-space ?dnew)))
                         (at end (not (thing-space ?dnew)))
                         (at end (free ?a))
                         (at end (increase (total-cost) (therblig-cost ?th ?a)))
                     )
        )

        (:durative-action grasp
            :parameters (?th - g ?a - agent ?t - thing ?dcurrent ?dnew - destination)
            :duration (= ?duration (therblig-time ?th ?a ?dcurrent ?dnew))
            :condition (and (at start (free ?a))
                            (= ?dcurrent ?dnew)
                            (at start (not-executed ?th))
                            (at start (agent-allocation ?th ?a))
                            (at start (thing-allocation ?th ?t))
                            (at start (not-gripping ?a))
                            (at start (at ?a ?dcurrent))
                            (at start (at ?t ?dcurrent))
                       )
            :effect (and (at start (not (free ?a)))
                         (at start (not (not-executed ?th)))
                         (at end (gripping ?a))
                         (at end (not (not-gripping ?a)))
                         (at end (free ?a))
                         (at end (increase (total-cost) (therblig-cost ?th ?a)))
                     )
        )

        (:durative-action release
            :parameters (?th - rl ?a - agent ?t - thing ?dcurrent ?dnew - destination)
            :duration (= ?duration (therblig-time ?th ?a ?dcurrent ?dnew))
            :condition (and (at start (free ?a))
                            (= ?dcurrent ?dnew)
                            (at start (not-executed ?th))
                            (at start (agent-allocation ?th ?a))
                            (at start (thing-allocation ?th ?t))
                            (at start (gripping ?a))
                            (at start (at ?a ?dcurrent))
                            (at start (at ?t ?dcurrent))
                       )
            :effect (and (at start (not (free ?a)))
                         (at start (not (not-executed ?th)))
                         (at end (not (gripping ?a)))
                         (at end (not-gripping ?a))
                         (at end (free ?a))
                         (at end (increase (total-cost) (therblig-cost ?th ?a)))
                     )
        )
    )"""

    instance = """(define (problem allocate)
      (:domain authr)
      (:objects\n{objects}
      )
      (:init\n{init}
      )
      (:goal (and {goal}))

      (:metric (and (minimize (total-time))(minimize(total-cost))))\n)
      """.format(name=plan["name"].replace(" ",""),
                 objects=plan_to_objects(plan,therblig_ids,therbligs),
                 init=plan_to_init(plan,therblig_ids,therbligs),
                 goal=goal)

    return domain,instance
