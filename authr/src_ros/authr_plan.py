#!/usr/bin/env python

import tf
import sys
import time
import json
import uuid
import rospy
import threading
import traceback
import moveit_commander

from copy import deepcopy
from authr_tools.msg_conversion import *
from authr_tools.verification_orig import Verifier as OrigVerifier
from authr_tools.verification_advanced import Verifier as AdvancedVerifier
from authr_tools.veropt import VerOpt, param_check
from authr_tools.expansion import Expander
from authr_tools.containers import Task, Macro
from authr_tools.therbligs import TherbligLookup
from authr_tools.elements import Thing, Destination, Agent, Type
from authr_tools.marker import MarkerServer
from authr_tools.simple_marker import SimpleMarkerServer

from std_msgs.msg import String, Header
from authr.msg import MarkerPose
from authr.msg import PlanUpdate, JobResult
from geometry_msgs.msg import Point, Quaternion, Pose, Vector3, PoseStamped
from authr.msg import JobSubmissionRequest, JobSubmissionResponse
from authr.srv import GetRequest,    GetRequestResponse
from authr.srv import SetRequest,    SetRequestResponse
from authr.srv import GetEE, GetEERequest, GetEEResponse
from authr.srv import CreateRequest, CreateRequestResponse
from authr.srv import DeleteRequest, DeleteRequestResponse
from authr.srv import CreateAgent, DeleteAgent, UpdateAgent


moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


class AuthrPlan(object):

    def __init__(self):
        rospy.init_node("authr_plan")

        self.name = "New Plan"
        self.version = '0.3'
        self.max_robots = 1
        self.max_humans = 1
        self.robot = rospy.get_param("/authr/robot")
        self.verifier_type = rospy.get_param("/authr/verifier")
        self.fixed_frame = "base_link"
        # TODO: Add focused attribute and ros parameter/service

        self._CONTAINER_COLLISION_MESH_FILE = rospy.get_param("~collider_mesh_path")

        # Containers for different Elements
        self.task_order =   []
        self.tasks =        {}
        self.therbligs =    {}
        self.therblig_types = ["transport_empty","transport_loaded","grasp","release_load","hold","rest"]
        self.primitives =   {type:TherbligLookup[type](type).primitive_dict for type in self.therblig_types}
        self.macros =       {}
        self.tofs =         {}
        # Things, Agents are referenced in both
        # destination and thing lookups
        self.things =       {}
        self.agents =       {}
        self.destinations = {}

        # Used by Optimizer
        self.time_weight = 0.5
        self.cost_weight = 0.5
        self.verify_thread = threading.Thread(target=self._verify, kwargs={"async":False,"log":False})
        self.needs_verification = False
        self.verify_thread_lock = threading.Lock()

        # Expanded plan cache
        self.expanded_plan_cache = {}

        # Create the first task
        self._create_task("My First Task")

        # Services to request information
        self.get_element_srv = rospy.Service("authr_plan/get_element", GetRequest, self.get_element)
        self.set_element_srv = rospy.Service("authr_plan/set_element", SetRequest, self.set_element)
        self.create_element_srv = rospy.Service("authr_plan/create_element", CreateRequest, self.create_element)
        self.delete_element_srv = rospy.Service("authr_plan/delete_element", DeleteRequest, self.delete_element)

        # Services requestable
        self.create_agent_sim = rospy.ServiceProxy("authr_sim/create_agent", CreateAgent)
        self.delete_agent_sim = rospy.ServiceProxy("authr_sim/delete_agent", DeleteAgent)
        self.update_agent_sim = rospy.ServiceProxy("authr_sim/update_agent", UpdateAgent)
        self.get_ee = rospy.ServiceProxy("authr_sim/get_ee", GetEE)
        self.clear_trajectory_cache_entry = rospy.ServiceProxy("authr_sim/delete_joint_trajectory", DeleteRequest)

        # Topic for debugging plan
        self.get_cache_topic = rospy.Publisher('authr_plan/get_cache', String, queue_size=10)
        self.request_cache_refresh_topic = rospy.Subscriber('authr_plan/request_cache_refresh', String, self.request_cache_refresh)

        # Topic for async update notice of plan
        self.update_topic = rospy.Publisher('authr_plan/update', PlanUpdate, queue_size=10)

        # Topics for authr sim jobs
        self.job_submission_request = rospy.Publisher('authr_sim/job_submission/request', JobSubmissionRequest, queue_size=30)
        self.job_submission_response = rospy.Subscriber("authr_sim/job_submission/response", JobSubmissionResponse, self._job_submission_response)
        self.job_results = rospy.Subscriber("authr_sim/job_result", JobResult, self._job_result)

        # Make a marker server to display things
        self.marker_server = MarkerServer("/interactive_marker/elements")
        self.visual_marker_server = SimpleMarkerServer("/visualization_marker/elements/")
        self.visual_marker_pose_topic = rospy.Subscriber("visualization_marker/set_pose", MarkerPose, self._visual_marker_set_pose)
        self.marker_visibility = rospy.Subscriber("authr_plan/marker_visibility", String, self._set_visible_marker_set)

    def get_robot_agent_id(self):
        #TODO note this assumes only one robot, when this assumption breaks
        # this function should be rewritten (probably part of larger refactor)
        id = None
        for key in self.agents.keys():
            if self.agents[key].type == 'robot':
                id = key
                break
        return id

    @property
    def counts(self):
        humans = 0
        robots = 0
        for agent in self.agents.keys():
            if self.agents[agent].type == "robot":
                robots += 1
            else:
                humans += 1

        return {"agents":len(self.agents),
                "humans":humans,
                "robots":robots,
                "things":len(self.things),
                "macros":len(self.macros),
                "tasks":len(self.tasks),
                "destinations":len(self.destinations),
                "therbligs":len(self.therbligs)}

    @property
    def plan_dict(self):
        return {"name":self.name,
                "version":self.version,
                "time_weight":self.time_weight,
                "cost_weight":self.cost_weight,
                "task_keys":self.task_order,
                "tasks":{task_id:self.tasks[task_id].task_dict for task_id in self.task_order},
                "macros":{macro_id:self.macros[macro_id].macro_dict for macro_id in self.macros.keys()},
                "therbligs":{therblig_id:self.therbligs[therblig_id].therblig_dict for therblig_id in self.therbligs.keys()},
                "agents":{agent_id:self.agents[agent_id].agent_dict for agent_id in self.agents.keys()},
                "things":{thing_id:self.things[thing_id].thing_dict for thing_id in self.things.keys()},
                "destinations":{destination_id:self.destinations[destination_id].destination_dict for destination_id in self.destinations.keys()},
                "tofs":self.tofs,
                "tof_table":self.tof_table}

    @property
    def tof_table(self):
        table = []
        keys = sorted(self.destinations.keys(), key=lambda id: self.destinations[id].timestamp)
        for start_destination_id in keys:
            entry = {"index":start_destination_id}
            for end_destination_id in keys:
                try:
                    entry[end_destination_id] = self.tofs[start_destination_id+"_"+end_destination_id]
                except KeyError, ke:
                    rospy.logwarn(str(ke))
                    entry[end_destination_id] = 'error'
            table.append(entry)
        return table

    @property
    def cache(self):
        return {
            "plan": self.plan_dict,
            "counts": self.counts,
            "fixed_frame": self.fixed_frame,
            "robot": self.robot,
            "primitives": self.primitives,
            "primitive_keys": self.therblig_types
        }

    def clear_plan(self):
        for agent_key in self.agents.keys():
            self.delete_agent_sim(agent_key)

        for dest_key in self.destinations.keys():
            try:
                self.marker_server.delete_marker(dest_key)
                self.visual_marker_server.delete_marker(dest_key)
            except: pass

            if dest_key in self.things and self.things[dest_key].type == Type.CONTAINER.value:
                scene.remove_world_object(dest_key)

        self.name = "New Plan"
        self.version = '0.3'
        self.max_robots = 1
        self.max_humans = 1
        self.robot = rospy.get_param("/authr/robot")
        self.fixed_frame = "base_link"
        # TODO: Add focused attribute and ros parameter/service

        # Containers for different Elements
        self.task_order =   []
        self.tasks =        {}
        self.therbligs =    {}
        self.therblig_types = ["transport_empty","transport_loaded","grasp","release_load","hold","rest"]
        self.primitives =   {type:TherbligLookup[type](type).primitive_dict for type in self.therblig_types}
        self.macros =       {}

        for did in self.destinations.keys():
            self._refresh_tofs(did, mode='delete')
        self.tofs =         {}
        # Things, Agents are referenced in both
        # destination and thing lookups
        self.things =       {}
        self.agents =       {}
        self.destinations = {}

        # Used by Optimizer
        self.time_weight = 0.5
        self.cost_weight = 0.5

        # Expanded plan must be generated first
        self.expanded_plan_cache = {}

    def request_cache_refresh(self, message):
        self.publish_cache_to_topic()

    def set_plan_from_dict(self, settings):
        self.clear_plan()
        if "version" in settings.keys() and settings["version"] in ["0.2","0.3"]:
            self.version = settings["version"]
            if "name" in settings.keys():
                self.name = settings["name"]
            if "cost_weight" in settings.keys():
                self.cost_weight = settings["cost_weight"]
            if "time_weight" in settings.keys():
                self.time_weight = settings["time_weight"]
            if "task_keys" in settings.keys():
                self.task_order = settings["task_keys"]
            if "tasks" in settings.keys():
                self.tasks = {task_key:Task(**settings["tasks"][task_key]) for task_key in settings["task_keys"]}
            if "therbligs" in settings.keys():
                for therblig_key in settings["therbligs"].keys():
                    therblig = TherbligLookup[settings["therbligs"][therblig_key]["type"]](therblig_key)
                    therblig.set(settings["therbligs"][therblig_key])
                    self.therbligs[therblig_key] = therblig
            if "agents" in settings.keys():
                for agent_info in sorted(settings["agents"].items(), key=lambda kv: kv[1]["name"]):
                    agent_key = agent_info[0]
                    agent = Agent(agent_key)
                    agent.set(settings["agents"][agent_key])
                    agent.set(settings["destinations"][agent_key])
                    self._create_agent_marker(agent_key,agent)
                    #TODO make this more straightforward
                    self.create_agent_sim(agent_key,agent.arms[0])
                    self.agents[agent_key] = agent
                    self.destinations[agent_key] = agent
                    for therblig_id in self.therbligs.keys():
                        self.therbligs[therblig_id].agent_type_lookup[agent_key] = agent.type
                    self._refresh_tofs(agent_key, mode='update')
            if "things" in settings.keys():
                for thing_info in sorted(settings["things"].items(), key=lambda kv: kv[1]["name"]):
                    thing_key = thing_info[0]
                    thing = Thing(thing_key)
                    thing.set(settings["things"][thing_key])
                    thing.set(settings["destinations"][thing_key])
                    self._create_thing_marker(thing_key,thing)

                    if thing.type == Type.CONTAINER.value:

                        (x,y,z,w) = tf.transformations.quaternion_from_euler(
                            thing.orientation['x'],
                            thing.orientation['y'],
                            thing.orientation['z'],
                            'sxyz')

                        scene.remove_world_object(thing_key)
                        scene.add_mesh(
                            name=thing_key,
                            pose=PoseStamped(
                                header=Header(frame_id=robot.get_planning_frame()),
                                pose=Pose(
                                    position=Vector3(thing.position['x'],thing.position['y'],thing.position['z']),
                                    orientation=Quaternion(x,y,z,w))),
                            filename=self._CONTAINER_COLLISION_MESH_FILE ,
                            size=[thing.size]*3)

                    self.things[thing_key] = thing
                    self.destinations[thing_key] = thing
                    self._refresh_tofs(thing_key, mode='update')
            if "destinations" in settings.keys():
                for destination_info in sorted(settings["destinations"].items(), key=lambda kv: kv[1]["name"]):
                    destination_key = destination_info[0]
                    if destination_key not in settings["agents"].keys() and destination_key not in settings["things"].keys():
                        rospy.loginfo(destination_key)
                        destination = Destination(destination_key)
                        destination.set(settings["destinations"][destination_key])
                        self._create_destination_marker(destination_key,destination)
                        self.destinations[destination_key] = destination
                        self._refresh_tofs(destination_key, mode='update', retries=3)
            rospy.loginfo(self.agents.keys())
            rospy.loginfo(self.things.keys())
            rospy.loginfo(self.destinations.keys())
        self.publish_cache_to_topic()

    def publish_cache_to_topic(self):
        self.get_cache_topic.publish(data=json.dumps(self.cache))

    def get_element(self, message):
        response = GetRequestResponse('{}')

        if message.id == "":
            if message.type == "name":
                name_string = self.name
                response = GetRequestResponse(name_string)
            elif message.type == "robot":
                robot_string = self.robot
                response = GetRequestResponse(robot_string)
            elif message.type == "fixed_frame":
                fixed_frame_string = self.fixed_frame
                response = GetRequestResponse(fixed_frame_string)
            elif message.type == "time_weight":
                time_weight_string = json.dumps(self.time_weight)
                response = GetRequestResponse(time_weight_string)
            elif message.type == "cost_weight":
                cost_weight_string = json.dumps(self.cost_weight)
                response = GetRequestResponse(cost_weight_string)
            elif message.type == "counts":
                counts_string = json.dumps(self.counts)
                response = GetRequestResponse(counts_string)
            elif message.type == "agent":
                agent_string = json.dumps({agent_id:self.agents[agent_id].agent_dict for agent_id in self.agents.keys()})
                response = GetRequestResponse(agent_string)
            elif message.type == "agent_keys":
                agent_string = json.dumps(sorted(self.agents.keys(), key=lambda id: self.agents[id].timestamp))
                response = GetRequestResponse(agent_string)
            elif message.type == "thing":
                thing_string = json.dumps({thing_id:self.things[thing_id].thing_dict for thing_id in self.things.keys()})
                response = GetRequestResponse(thing_string)
            elif message.type == "thing_keys":
                thing_string = json.dumps(sorted(self.things.keys(), key=lambda id: self.things[id].timestamp))
                response = GetRequestResponse(thing_string)
            elif message.type == "destination":
                destination_string = json.dumps({destination_id:self.destinations[destination_id].destination_dict for destination_id in self.destinations.keys()})
                response = GetRequestResponse(destination_string)
            elif message.type == "destination_keys":
                destination_string = json.dumps(sorted(self.destinations.keys(), key=lambda id: self.destinations[id].timestamp))
                response = GetRequestResponse(destination_string)
            elif message.type == "task":
                task_string = json.dumps({task_id:self.tasks[task_id].task_dict for task_id in self.task_order})
                response = GetRequestResponse(task_string)
            elif message.type == "task_keys":
                task_string = json.dumps(self.task_order)
                response = GetRequestResponse(task_string)
            elif message.type == "macro":
                macro_string = json.dumps({macro_id:self.macros[macro_id].macro_dict for macro_id in self.macros.keys()})
                response = GetRequestResponse(macro_string)
            elif message.type == "macro_keys":
                macro_string = json.dumps(sorted(self.macros.keys(), key=lambda id: self.macros[id].timestamp))
                response = GetRequestResponse(macro_string)
            elif message.type == "therblig":
                therblig_string = json.dumps({therblig_id:self.therbligs[therblig_id].therblig_dict for therblig_id in self.therbligs.keys()})
                response = GetRequestResponse(therblig_string)
            elif message.type == "therblig_keys":
                therblig_string = json.dumps(self.therbligs.keys())
                response = GetRequestResponse(therblig_string)
            elif message.type == "therblig_primitive":
                primitive_string = json.dumps(self.primitives)
                response = GetRequestResponse(primitive_string)
            elif message.type == "therblig_primitive_keys":
                primitive_string = json.dumps(self.therblig_types)
                response = GetRequestResponse(primitive_string)
            elif message.type == "tof_table":
                table = []
                keys = sorted(self.destinations.keys(), key=lambda id: self.destinations[id].timestamp)
                for start_destination_id in keys:
                    entry = {"index":start_destination_id}
                    for end_destination_id in keys:
                        entry[end_destination_id] = self.tofs[start_destination_id+"_"+end_destination_id]
                    table.append(entry)
                tof_string = json.dumps(table)
                response = GetRequestResponse(tof_string)
            elif message.type == "tof":
                tof_string = json.dumps(self.tofs)
                response = GetRequestResponse(tof_string)
            elif message.type == "plan":
                plan_string = json.dumps(self.plan_dict)
                response = GetRequestResponse(plan_string)
            elif message.type == "cache":
                cache_string = json.dumps(self.cache)
                response = GetRequestResponse(cache_string)
            elif message.type == "expanded_plan":
                self.expanded_plan_cache = self._expand()
                plan_string = json.dumps(self.expanded_plan_cache)
                response = GetRequestResponse(plan_string)
            elif message.type == "expanded_plan_cached":
                plan_string = json.dumps(self.expanded_plan_cache)
                response = GetRequestResponse(plan_string)
            else:
                response = GetRequestResponse('{}')
        else:
            if message.type == "agent" and message.id in self.agents.keys():
                agent_string = json.dumps(self.agents[message.id].agent_dict)
                response = GetRequestResponse(agent_string)
            elif message.type == "thing" and message.id in self.things.keys():
                thing_string = json.dumps(self.things[message.id].thing_dict)
                response = GetRequestResponse(thing_string)
            elif message.type == "destination" and message.id in self.destinations.keys():
                destination_string = json.dumps(self.destinations[message.id].destination_dict)
                response = GetRequestResponse(destination_string)
            elif message.type == "task" and message.id in self.tasks.keys():
                task_string = json.dumps(self.tasks[message.id].task_dict)
                response = GetRequestResponse(task_string)
            elif message.type == "macro" and message.id in self.macros.keys():
                macro_string = json.dumps(self.macros[message.id].macro_dict)
                response = GetRequestResponse(macro_string)
            elif message.type == "therblig" and message.id in self.therbligs.keys():
                therblig_string = json.dumps(self.therbligs[message.id].therblig_dict)
                response = GetRequestResponse(therblig_string)
            elif message.type == "tof" and message.id in self.tofs.keys():
                tof_string = json.dumps(self.tofs[message.id])
                response = GetRequestResponse(tof_string)
            else:
                response = GetRequestResponse("{}")

        self.publish_cache_to_topic()
        return response

    def set_element(self, message):
        success = True
        if message.type == "agent" and message.id in self.agents.keys():
            settings = json.loads(message.settings)
            self.agents[message.id].set(settings)
            if "type" in settings.keys():
                for therblig_id in self.therbligs.keys():
                    self.therbligs[therblig_id].agent_type_lookup[message.id] = self.agents[message.id].type
            #TODO clean up the end-effector linkage between agents, plan, sim, and behavior_planner
            self.update_agent_sim(message.id,self.agents[message.id].arms[0])
            self._update_marker_setting(settings, message.id, allowed=('type','color','name','movable'))
            if len(set(settings.keys()).intersection(("position","orientation","type"))) > 0:

                agent = self.get_robot_agent_id()
                if agent != None:
                    self.job_submission_request.publish(JobSubmissionRequest(message.id,agent,'validate_pose',json.dumps({
                        'pose': self.agents[message.id].destination_dict
                    })))

                self._refresh_tofs(message.id, mode='update')

        elif message.type == "thing" and message.id in self.things.keys():
            settings = json.loads(message.settings)

            thing = self.things[message.id]

            prevType = thing.type

            thing.set(settings)
            self._update_marker_setting(settings, message.id, allowed=('type','color','size','name','movable'))

            updatedPosition = len(set(settings.keys()).intersection(("position","orientation"))) > 0

            print '\n\n\n Thing set looking at container setting'
            print prevType, thing.type, Type.CONTAINER

            if prevType == Type.CONTAINER.value and thing.type != Type.CONTAINER.value:
                scene.remove_world_object(message.id)
            elif thing.type == Type.CONTAINER.value and (prevType != Type.CONTAINER.value or updatedPosition):

                (x,y,z,w) = tf.transformations.quaternion_from_euler(
                    thing.orientation['x'],
                    thing.orientation['y'],
                    thing.orientation['z'],
                    'sxyz')

                scene.remove_world_object(message.id)
                scene.add_mesh(
                    name=message.id,
                    pose=PoseStamped(
                        header=Header(frame_id=robot.get_planning_frame()),
                        pose=Pose(
                            position=Vector3(thing.position['x'],thing.position['y'],thing.position['z']),
                            orientation=Quaternion(x,y,z,w))),
                    filename=self._CONTAINER_COLLISION_MESH_FILE ,
                    size=[thing.size]*3)

            if updatedPosition:
                agent = self.get_robot_agent_id()
                if agent != None:
                    self.job_submission_request.publish(JobSubmissionRequest(message.id,agent,'validate_pose',json.dumps({
                        'pose': thing.destination_dict
                    })))

                self._refresh_tofs(message.id, mode='update')

        elif message.type == "destination" and  message.id in self.destinations.keys():
            settings = json.loads(message.settings)
            rospy.loginfo("Setting destination {0}".format(settings))
            settings.update({"elementSetType":"destination"})
            self.destinations[message.id].set(settings)
            if message.id in self.things.keys() or message.id in self.agents.keys():
                self._update_marker_setting(settings, message.id, allowed=('position','orientation','movable'))
            else:
                self._update_marker_setting(settings, message.id, allowed=('color','position','orientation','name','movable'))
            if len(set(settings.keys()).intersection(("position","orientation"))) > 0:

                agent = self.get_robot_agent_id()
                if agent != None:

                    self.job_submission_request.publish(JobSubmissionRequest(message.id,agent,'validate_pose',json.dumps({
                        'pose': self.destinations[message.id].destination_dict
                    })))

                self._refresh_tofs(message.id, mode='update')

        elif message.type == "therblig" and message.id in self.therbligs.keys():
            settings = json.loads(message.settings)
            rospy.loginfo("Setting therblig {0}".format(settings))
            # Unpack the parameters
            if "parameters" in settings.keys():
                settings.update(settings["parameters"])
                settings.pop("parameters")
            if "arm" in self.therbligs[message.id].settable:
                # Set the arm if only one option exists for the agent provided
                if "agent" in settings.keys() and "agent" in self.therbligs[message.id].settable:
                    # Check if unsetting or setting agent
                    if (settings["agent"] == None or settings["agent"] == "OPTIMIZE_DIRECTIVE") and "arm" in self.therbligs[message.id].settable:
                        settings['arm'] = None
                    elif "arm" in self.therbligs[message.id].settable and len(self.agents[settings["agent"]].arms) == 1:
                        settings["arm"] = self.agents[settings["agent"]].arms[0]
            self.therbligs[message.id].set(settings)
            if len(set(["parameters","agent","thing","destination","arm"]).union(set(settings.keys()))) > 0:
                self._verify(async=True,log=True)

        elif message.type == "task" and message.id in self.tasks.keys():
            settings = json.loads(message.settings)
            should_verify = False
            if "therbligs" in settings.keys() or "repeat" in settings.keys():
                should_verify = True
            self.tasks[message.id].set(settings)
            if len(set(self.tasks[message.id].therbligs).intersection(self.therblig_types)) > 0:
                while len(set(self.tasks[message.id].therbligs).intersection(self.therblig_types)) > 0:
                    for index, therblig_id in enumerate(self.tasks[message.id].therbligs):
                        if therblig_id in self.therblig_types:
                            rospy.logwarn("Replacing primitive {0} with therblig".format(therblig_id))
                            self.tasks[message.id].therbligs.remove(therblig_id)
                            self._create_therblig(message.id,index,therblig_id)
                            should_verify = True
                            break
            if len(set(self.tasks[message.id].therbligs).intersection(self.macros.keys())) > 0:
                while len(set(self.tasks[message.id].therbligs).intersection(self.macros.keys())) > 0:
                    for index, therblig_id in enumerate(self.tasks[message.id].therbligs):
                        if therblig_id in self.macros.keys():
                            rospy.logwarn("Replacing macro {0} with therbligs".format(therblig_id))
                            self.tasks[message.id].therbligs.remove(therblig_id)
                            self._create_therbligs_from_macro(message.id,index,therblig_id)
                            should_verify = True
                            break
            if should_verify:
                self._verify(async=True,log=True)
        elif message.type == "task_keys":
            rospy.logwarn(json.loads(message.settings))
            self.task_order = json.loads(message.settings)
        elif message.type == "macro" and message.id in self.macros.keys():
            self.macros[message.id].set(json.loads(message.settings))
        elif message.type == "time_weight":
            self.time_weight = json.loads(message.settings)
        elif message.type == "cost_weight":
            self.cost_weight = json.loads(message.settings)
        elif message.type == "name":
            self.name = message.settings
        elif message.type == "fixed_frame":
            self.fixed_frame = message.settings
        elif message.type == "plan":
            self.set_plan_from_dict(json.loads(message.settings))
            self._verify(async=True,log=True)
        elif message.type == "pose" and message.id in self.destinations.keys():
            agent = self.get_robot_agent_id()
            if agent != None:
                resp = self.get_ee(agent,GetEERequest.TYPE_ARM)
                pos = resp.pose.position
                ori = resp.pose.orientation
                self.destinations[message.id].position = {"x":pos.x,"y":pos.y,"z":pos.z}
                self.destinations[message.id].orientation = {"x":ori.x,"y":ori.y,"z":ori.z}

                updateMsg = PlanUpdate()
                updateMsg.type = 'thing'
                updateMsg.id = message.id
                self.update_topic.publish(updateMsg)

                self._refresh_tofs(message.id, mode='update')
        elif message.type == "tof" and message.id in self.tofs.keys():
            self.tofs[message.id] = 'pending'

            settings = json.loads(message.settings)
            self.job_submission_request.publish(JobSubmissionRequest(
                message.id,
                self.get_robot_agent_id(),
                'tof_pose',
                json.dumps({
                    'pose_start':pose_eulerDictFromEulerMsg(self.destinations[settings['start_destination_id']].pose),
                    'pose_end':pose_eulerDictFromEulerMsg(self.destinations[settings['end_destination_id']].pose)})))

        elif message.type == "validate_pose" and message.id in self.destinations.keys():
            rospy.loginfo('validating pose')
            agent = self.get_robot_agent_id()
            destination = self.destinations[message.id]
            if agent != None:
                self.job_submission_request.publish(JobSubmissionRequest(message.id,agent,'validate_pose',json.dumps({
                    'pose': destination.destination_dict
                })))

        self.publish_cache_to_topic()
        return SetRequestResponse(success,"")

    def _update_marker_setting(self, settings, id, allowed=[]):
        if len(set(allowed).intersection(set(settings.keys()))) > 0:
            if 'type' in settings.keys() and 'type' in allowed:
                type_marker = Type.fromStr(settings['type'])
                self.marker_server.set_type(id,type_marker,disableRefresh=True)
                self.visual_marker_server.set_type(id,type_marker,disableRefresh=False)
            if 'color' in settings.keys() and 'color' in allowed:
                self.marker_server.set_color(id,
                    r=settings["color"]["r"],
                    g=settings["color"]["g"],
                    b=settings["color"]["b"],
                    disableRefresh=True)
                self.visual_marker_server.set_color(id,
                    r=settings["color"]["r"],
                    g=settings["color"]["g"],
                    b=settings["color"]["b"],
                    disableRefresh=True)
            if 'size' in settings.keys() and 'size' in allowed:
                self.marker_server.set_size(id,settings["size"], disableRefresh=True)
                self.visual_marker_server.set_size(id,settings["size"], disableRefresh=True)
            if 'position' in settings.keys() and 'position' in allowed:
                position = Point(
                    x=settings["position"]['x'],
                    y=settings["position"]['y'],
                    z=settings["position"]['z'])
                self.marker_server.set_position(id, position, disableRefresh=True)
                self.visual_marker_server.set_position(id, position, disableRefresh=True)
            if 'orientation' in settings.keys() and 'orientation' in allowed:
                eulerOrientation = settings["orientation"]
                (x,y,z,w) = tf.transformations.quaternion_from_euler(
                    eulerOrientation['x'],
                    eulerOrientation['y'],
                    eulerOrientation['z'],
                    'sxyz')
                quaternionOrientation = Quaternion(x=x,y=y,z=z,w=w)
                self.marker_server.set_orientation(id, quaternionOrientation, disableRefresh=True)
                self.visual_marker_server.set_orientation(id, quaternionOrientation, disableRefresh=True)
            if 'name' in settings.keys() and 'name' in allowed:
                self.marker_server.set_name(id,settings['name'], disableRefresh=True)
                self.visual_marker_server.set_name(id,settings['name'], disableRefresh=True)
            if 'movable' in settings.keys() and 'movable' in allowed:
                self.marker_server.set_movable(id,settings['movable'], disableRefresh=True)

            self.marker_server.refresh(id)
            self.visual_marker_server.refresh(id)

    def create_element(self, message):
        parameters = json.loads(message.parameters)
        rospy.loginfo("Received request to create {0}.".format(message.type))
        if message.type == "task":
            (success, message, id) = self._create_task()
        # elif message.type == "therblig":
        #     (success, message, id) = self._create_therblig(**parameters)
        #     self._verify(async=True,log=True)
        elif message.type == "agent":
            (success, message, id) = self._create_agent()
        elif message.type == "thing":
            (success, message, id) = self._create_thing()
        elif message.type == "destination":
            (success, message, id) = self._create_destination()
        elif message.type == "macro":
            (success, message, id) = self._create_macro(**parameters)
        self.publish_cache_to_topic()
        return CreateRequestResponse(success,message, id)

    def delete_element(self, message):
        rospy.loginfo("Received request to delete {0} with id {1}.".format(message.type, message.id))
        should_verify = False
        if message.type == "task":
            (success, message) = self._delete_task(message.id)
            should_verify = True
        elif message.type == "therblig":
            (success, message) = self._delete_therblig(message.id)
            should_verify = True
        elif message.type == "agent":
            (success, message) = self._delete_agent(message.id)
            should_verify = True
        elif message.type == "thing":
            (success, message) = self._delete_thing(message.id)
            should_verify = True
        elif message.type == "destination":
            (success, message) = self._delete_destination(message.id)
            should_verify = True
        elif message.type == "macro":
            (success, message) = self._delete_macro(message.id)
        if should_verify:
            self._verify(async=True,log=True)
        self.publish_cache_to_topic()
        return DeleteRequestResponse(success,message)

    def end_update_thread(self):
        self._tof_update_thread_run = False

    def _create_task(self, name="New Task"):
        id = self._create_id("task")
        task = Task(id,name,therbligs=[])
        self.tasks[id] = task
        self.task_order.append(id)
        rospy.loginfo("Created task {0} with id {1}".format(task.task_dict, id))
        return (True, "", id)

    def _delete_task(self, id):
        try:
            self.task_order.remove(id)
            rospy.loginfo("Deleting task {0} with id {1}".format(self.tasks[id].task_dict, id))
            for therblig_id in self.tasks[id].therbligs:
                self._delete_therblig(therblig_id)
            self.tasks.pop(id)
            return (True, "")
        except:
            return (False, "Error: Could not delete task "+id)

    def _create_macro(self, task_id):
        id = self._create_id("macro")
        agent_lookup = {agent_id:self.agents[agent_id].type for agent_id in self.agents.keys()}
        if task_id in self.tasks.keys():
            therbligs = []
            references = self.tasks[task_id].therbligs
            for reference in references:
                therblig_id = self._create_id("tmpl_therblig")
                therblig_obj = TherbligLookup[self.therbligs[reference].type](therblig_id)
                therblig_obj.set(deepcopy(self.therbligs[reference].therblig_dict))
                therblig_obj.agent_type_lookup = agent_lookup
                self.therbligs[therblig_id] = therblig_obj
                therbligs.append(therblig_id)
            self.macros[id] = Macro(id,self.tasks[task_id].name,therbligs)
            rospy.loginfo("Created macro {0} with id {1}".format(self.macros[id].name, id))
            return (True, "", id)
        return (False, "Error: Could not create macro.","")

    def _delete_macro(self, id):
        try:
            rospy.loginfo("Deleting macro {0} with id {1}".format(self.macros[id].name, id))
            for therblig_id in self.macros[id].therbligs:
                self.therbligs.pop(therblig_id)
            self.macros.pop(id)
            rospy.loginfo("Deleted macro with id "+id)
            return (True, "")
        except:
            return (False, "Error: Could not delete macro "+id)

    def _create_therblig(self,task_id,index,type):
        if type in self.macros.keys():
            return self._create_therbligs_from_macro(task_id,index,type)
        if task_id in self.task_order:
            rospy.loginfo("task_id found")
            therblig_id = self._create_id("therblig")
            therblig_obj = TherbligLookup[type](therblig_id)

            # Define agents, things, destinations, and arms if possible
            settings = {"duration":{},"cost":{}}
            if len(self.agents) == 1 and "agent" in therblig_obj.settable:
                # rospy.loginfo("Inferring agent "+self.agents.keys()[0]+" for therblig")
                # settings["agent"] = self.agents.keys()[0]
                settings["agent"] = "OPTIMIZE_DIRECTIVE"
            if len(self.things) == 1 and "thing" in therblig_obj.settable:
                rospy.loginfo("Inferring thing "+self.things.keys()[0]+" for therblig")
                settings["thing"] = self.things.keys()[0]
            if len(self.destinations) == 1 and "destination" in therblig_obj.settable:
                rospy.loginfo("Inferring destination "+self.destinations.keys()[0]+" for therblig")
                settings["destination"] = self.destinations.keys()[0]
            # if "agent" in settings.keys() and "arm" in therblig_obj.settable and settings["agent"] != None and  len(self.agents[settings["agent"]].arms) == 1:
            #     rospy.loginfo("Inferring arm for "+self.agents.keys()[0]+" for therblig")
            #     settings["arm"] = self.agents[settings["agent"]].arms[0]

            # Add any agents for cost/duration lookup
            for agent_id in self.agents.keys():
                if not therblig_obj.universal:
                    settings["duration"][agent_id] = None
                settings["cost"][agent_id] = None
                therblig_obj.agent_type_lookup[agent_id] = self.agents[agent_id].type

            rospy.loginfo("Settings for therblig: {0}".format(settings))
            therblig_obj.set(settings)

            # Add Therblig to lookups and task
            self.therbligs[therblig_id] = therblig_obj
            self.tasks[task_id].therbligs.insert(index,therblig_id)
            return (True, "", therblig_id)
        else:
            return (False, "Error: Could not create therblig, task "+task_id+" does not exist.", '')

    def _create_therbligs_from_macro(self,task_id,index,macro_id):
        agent_lookup = {agent_id:self.agents[agent_id].type for agent_id in self.agents.keys()}
        if task_id in self.task_order:
            for reference in self.macros[macro_id].therbligs:
                therblig_id = self._create_id("therblig")
                therblig_obj = TherbligLookup[self.therbligs[reference].type](therblig_id)
                therblig_obj.set(deepcopy(self.therbligs[reference].therblig_dict))
                therblig_obj.agent_type_lookup = agent_lookup
                self.therbligs[therblig_id] = therblig_obj
                self.tasks[task_id].therbligs.insert(index,therblig_id)
                index+=1

    def _delete_therblig(self,therblig_id):
        try:
            # Remove references to therbligs
            for task_id in self.task_order:
                while therblig_id in self.tasks[task_id].therbligs:
                    self.tasks[task_id].therbligs.remove(therblig_id)

            # Remove therblig from lookup
            self.therbligs.pop(therblig_id)
            return (True, "")
        except:
            return (False, "Error: Could not delete therblig "+therblig_id)

    def _create_agent_marker(self, agent_id, agent_obj):
        self.marker_server.add_marker(agent_id, Type.fromStr(agent_obj.type), callback=lambda message: self._marker_callback(agent_id,message),name=agent_obj.name)
        self.visual_marker_server.add_marker(agent_id, Type.fromStr(agent_obj.type),Pose(),name=agent_obj.name)
        self.marker_server.set_color(agent_id, r=agent_obj.color["r"],g=agent_obj.color["g"],b=agent_obj.color["b"], disableRefresh=True)
        self.visual_marker_server.set_color(agent_id, r=agent_obj.color["r"],g=agent_obj.color["g"],b=agent_obj.color["b"], disableRefresh=True)
        self.marker_server.set_size(agent_id, 0.1, disableRefresh=True)
        self.visual_marker_server.set_size(agent_id, 0.1, disableRefresh=True)
        self.marker_server.set_movable(agent_id, agent_obj.movable, disableRefresh=True)


        position = Point(
            x=agent_obj.position['x'],
            y=agent_obj.position['y'],
            z=agent_obj.position['z'])
        self.marker_server.set_position(agent_id, position, disableRefresh=True)
        self.visual_marker_server.set_position(agent_id, position, disableRefresh=True)

        eulerOrientation = agent_obj.orientation
        (x,y,z,w) = tf.transformations.quaternion_from_euler(
            eulerOrientation['x'],
            eulerOrientation['y'],
            eulerOrientation['z'],
            'sxyz')
        quaternionOrientation = Quaternion(x=x,y=y,z=z,w=w)
        self.marker_server.set_orientation(agent_id, quaternionOrientation, disableRefresh=True)
        self.visual_marker_server.set_orientation(agent_id, quaternionOrientation, disableRefresh=True)
        self.marker_server.refresh(agent_id)
        self.visual_marker_server.refresh(agent_id)

    def _create_agent(self):
        try:
            agent_id = self._create_id("agent")
            counts = self.counts
            agent_type = Type.HUMAN
            if counts["humans"] >= self.max_humans and counts["robots"] >= self.max_robots:
                return (False, "Error: Could not create agent. Max number of humans and robots reached.", '')
            elif counts["humans"] >= self.max_humans:
                agent_type = Type.ROBOT

            agent_obj = Agent(agent_id,agent_type)
            self._create_agent_marker(agent_id,agent_obj)

            for therblig_id, therblig_obj in self.therbligs.items():
                if not therblig_obj.universal:
                    therblig_obj.duration[agent_id] = None
                therblig_obj.cost[agent_id] = None
                therblig_obj.agent_type_lookup[agent_id] = agent_obj.type

            #TODO make this more straightforward
            self.create_agent_sim(agent_id,agent_obj.arms[0])

            self.agents[agent_id] = agent_obj
            self.destinations[agent_id] = agent_obj
            rospy.loginfo(self.agents.keys())
            rospy.loginfo(self.destinations.keys())
            self._refresh_tofs(agent_id, mode='update')
            rospy.loginfo(self.tofs)
            return (True, "", agent_id)
        except Exception, e:
            print e
            traceback.print_exc()
            return (False, "Error: Could not create agent.", '')

    def _delete_agent(self,id):
        try:
            self.marker_server.delete_marker(id)
            self.visual_marker_server.delete_marker(id)

            for therblig_id, therblig_obj in self.therbligs.items():
                rospy.loginfo("Checking "+therblig_id+" for agent "+id)
                if "agent" in therblig_obj.parameters and therblig_obj.parameters_field["agent"] == id:
                    therblig_obj.set({'agent':"OPTIMIZE_DIRECTIVE",'arm':None})
                if "destination" in therblig_obj.parameters and therblig_obj.parameters_field["destination"] == id:
                    therblig_obj.set({"destination":None})
                if id in therblig_obj.duration.keys():
                    therblig_obj.duration.pop(id)
                therblig_obj.cost.pop(id)
                therblig_obj.agent_type_lookup.pop(id)

            rospy.loginfo("refreshing tofs")
            self._refresh_tofs(id, mode='delete')
            rospy.loginfo(self.tofs)
            self.agents.pop(id)

            self.destinations.pop(id)

            self.delete_agent_sim(id)

            return (True, "")
        except Exception, e:
            print e
            traceback.print_exc()
            return (False, "Error: Could not delete agent "+id)

    def _create_thing_marker(self, thing_id, thing_obj):
        self.marker_server.add_marker(thing_id, Type.fromStr(thing_obj.type), callback=lambda message: self._marker_callback(thing_id,message),name=thing_obj.name)
        self.visual_marker_server.add_marker(thing_id, Type.fromStr(thing_obj.type), Pose(), name=thing_obj.name)
        self.marker_server.set_color(thing_id, r=thing_obj.color["r"],g=thing_obj.color["g"],b=thing_obj.color["b"], disableRefresh=True)
        self.visual_marker_server.set_color(thing_id, r=thing_obj.color["r"], g=thing_obj.color["g"], b=thing_obj.color["b"], disableRefresh=True)
        self.marker_server.set_size(thing_id, thing_obj.size, disableRefresh=True)
        self.visual_marker_server.set_size(thing_id, thing_obj.size, disableRefresh=True)
        self.marker_server.set_movable(thing_id, thing_obj.movable, disableRefresh=True)

        position = Point(
            x=thing_obj.position['x'],
            y=thing_obj.position['y'],
            z=thing_obj.position['z'])
        self.marker_server.set_position(thing_id, position, disableRefresh=True)
        self.visual_marker_server.set_position(thing_id, position, disableRefresh=True)
        eulerOrientation = thing_obj.orientation
        (x,y,z,w) = tf.transformations.quaternion_from_euler(
            eulerOrientation['x'],
            eulerOrientation['y'],
            eulerOrientation['z'],
            'sxyz')
        quaternionOrientation = Quaternion(x=x,y=y,z=z,w=w)
        self.marker_server.set_orientation(thing_id, quaternionOrientation, disableRefresh=True)
        self.visual_marker_server.set_orientation(thing_id, quaternionOrientation, disableRefresh=True)
        self.marker_server.refresh(thing_id)
        self.visual_marker_server.refresh(thing_id)

    def _create_thing(self):
        try:
            thing_id = self._create_id("thing")
            thing_obj = Thing(thing_id)

            self._create_thing_marker(thing_id,thing_obj)

            if thing_obj.type == Type.CONTAINER.value:
                scene.add_mesh(
                    name=message.id,
                    pose=PoseStamped(
                        header=Header(frame_id=robot.get_planning_frame()),
                        pose=Pose(
                            position=Vector3(thing_obj.position['x'],thing_obj.position['y'],thing_obj.position['z']),
                            orientation=Quaternion(x,y,z,w))),
                    filename=self._CONTAINER_COLLISION_MESH_FILE ,
                    size=[thing.size]*3)

            self.things[thing_id] = thing_obj
            self.destinations[thing_id] = thing_obj
            self._refresh_tofs(thing_id, mode='update')
            rospy.loginfo(self.tofs)
            return (True, "", thing_id)
        except Exception, e:
            print e
            traceback.print_exc(file=sys.stdout)
            return (False, "Error: Could not create thing.", '')

    def _delete_thing(self,id):
        try:
            self.marker_server.delete_marker(id)
            self.visual_marker_server.delete_marker(id)

            # Remove references to this thing in therbligs
            for therblig_id, therblig_obj in self.therbligs.items():
                if "thing" in therblig_obj.parameters and therblig_obj.parameters_field["thing"] == id:
                    therblig_obj.set({"thing":None})
                if "destination" in therblig_obj.parameters and therblig_obj.parameters_field["destination"] == id:
                    therblig_obj.set({"destination":None})

            if self.things[id].type == Type.CONTAINER.value:
                scene.remove_world_object(id)

            rospy.loginfo("refreshing tofs")
            self._refresh_tofs(id, mode='delete')
            rospy.loginfo(self.tofs)
            self.things.pop(id)
            self.destinations.pop(id)
            return (True, "")
        except:
            return (False, "Error: Could not delete thing "+id)

    def _create_destination_marker(self, dest_id, dest_obj):
        self.marker_server.add_marker(dest_id, Type.DESTINATION, callback=lambda message: self._marker_callback(dest_id,message),name=dest_obj.name)
        self.visual_marker_server.add_marker(dest_id, Type.DESTINATION, Pose(), name=dest_obj.name)
        self.marker_server.set_color(dest_id, r=dest_obj.color["r"],g=dest_obj.color["g"],b=dest_obj.color["b"], disableRefresh=True)
        self.visual_marker_server.set_color(dest_id, r=dest_obj.color["r"],g=dest_obj.color["g"],b=dest_obj.color["b"], disableRefresh=True)
        self.marker_server.set_size(dest_id, 0.1, disableRefresh=True)
        self.visual_marker_server.set_size(dest_id, 0.1, disableRefresh=True)
        self.marker_server.set_movable(dest_id,dest_obj.movable, disableRefresh=True)

        position = Point(
            x=dest_obj.position['x'],
            y=dest_obj.position['y'],
            z=dest_obj.position['z'])
        self.marker_server.set_position(dest_id, position, disableRefresh=True)
        self.visual_marker_server.set_position(dest_id, position, disableRefresh=True)
        eulerOrientation = dest_obj.orientation
        (x,y,z,w) = tf.transformations.quaternion_from_euler(
            eulerOrientation['x'],
            eulerOrientation['y'],
            eulerOrientation['z'],
            'sxyz')
        quaternionOrientation = Quaternion(x=x,y=y,z=z,w=w)
        self.marker_server.set_orientation(dest_id, quaternionOrientation, disableRefresh=True)
        self.visual_marker_server.set_orientation(dest_id, quaternionOrientation, disableRefresh=True)
        self.marker_server.refresh(dest_id)
        self.visual_marker_server.refresh(dest_id)

    def _create_destination(self):
        try:
            dest_id = self._create_id("destination")
            dest_obj = Destination(dest_id)

            self._create_destination_marker(dest_id, dest_obj)

            self.destinations[dest_id] = dest_obj
            self._refresh_tofs(dest_id, mode='update')
            rospy.loginfo(self.tofs)
            return (True, "", dest_id)
        except:
            return (False, "Error: Could not create destination.", '')

    def _delete_destination(self,id):
        try:
            self.marker_server.delete_marker(id)
            self.visual_marker_server.delete_marker(id)

            # Remove references to this thing in therbligs
            for therblig_id, therblig_obj in self.therbligs.items():
                if "thing" in therblig_obj.parameters and therblig_obj.parameters_field["thing"] == id:
                    therblig_obj.set({"thing":None})
                if "destination" in therblig_obj.parameters and therblig_obj.parameters_field["destination"] == id:
                    therblig_obj.set({"destination":None})
            self._refresh_tofs(id, mode='delete')
            rospy.loginfo(self.tofs)
            self.destinations.pop(id)
            return (True, "")
        except:
            return (False, "Error: Could not delete destination "+id)

    def _create_id(self, type):
        return "{0}-{1}-{2}".format(self.version, type, uuid.uuid4())

    def _marker_callback(self,id,message):
        self.destinations[id].pose = message.pose
        self.visual_marker_server.set_pose(id,message.pose)

        rospy.loginfo("Updating the element on the frontend due to changes in marker")
        self.publish_cache_to_topic()

        if id in self.things.keys() and self.things[id].type == Type.CONTAINER.value:
            scene.remove_world_object(id)
            scene.add_mesh(
                name=id,
                pose=PoseStamped(
                    header=Header(frame_id=robot.get_planning_frame()),
                    pose=message.pose),
                filename=self._CONTAINER_COLLISION_MESH_FILE ,
                size=[self.things[id].size]*3)

        if id in self.agents.keys():

            self.job_submission_request.publish(JobSubmissionRequest(id+'_append_to_make_distinct_',
                id,
                'set_pose',
                json.dumps({
                    'pose':pose_eulerDictFromQuaternionMsg(message.pose),
                    'nonblocking': False
                })))

        agent = self.get_robot_agent_id()
        if agent != None:

            self.job_submission_request.publish(JobSubmissionRequest(id,agent,'validate_pose',json.dumps({
                'pose': pose_eulerDictFromQuaternionMsg(message.pose)
            })))

        self._refresh_tofs(id, mode='update')

    def _refresh_tofs(self, changed_destination, mode='update', retries=0):
        try:
            if mode == 'update':
                rospy.loginfo("Updating tofs for "+changed_destination)

                for destination_id in self.destinations.keys():
                    if changed_destination != destination_id:
                        self.tofs[destination_id+"_"+changed_destination] = 'pending'
                        self.tofs[changed_destination+"_"+destination_id] = 'pending'

                        # new tofs to calculate
                        self.job_submission_request.publish(JobSubmissionRequest(
                            destination_id+"_"+changed_destination,
                            self.get_robot_agent_id(),
                            'tof_pose',
                            json.dumps({
                                'pose_start':pose_eulerDictFromEulerMsg(self.destinations[destination_id].pose),
                                'pose_end':pose_eulerDictFromEulerMsg(self.destinations[changed_destination].pose),
                                'retries':retries})))

                        self.job_submission_request.publish(JobSubmissionRequest(
                            changed_destination+"_"+destination_id,
                            self.get_robot_agent_id(),
                            'tof_pose',json.dumps({
                                'pose_start':pose_eulerDictFromEulerMsg(self.destinations[changed_destination].pose),
                                'pose_end':pose_eulerDictFromEulerMsg(self.destinations[destination_id].pose),
                                'retries':retries})))

                    else:
                        self.tofs[destination_id+"_"+changed_destination] = 0

            elif mode == 'delete':
                rospy.loginfo("Deleting tofs for "+changed_destination)
                rospy.loginfo(self.tofs.keys())

                for destination_id in self.destinations.keys():

                    # send cancel to sim, ignore if not found
                    self.job_submission_request.publish(JobSubmissionRequest(destination_id+"_"+changed_destination,self.get_robot_agent_id(),'cancel',json.dumps({})))
                    self.job_submission_request.publish(JobSubmissionRequest(changed_destination+"_"+destination_id,self.get_robot_agent_id(),'cancel',json.dumps({})))

                    # delete from trajectory cache
                    self.clear_trajectory_cache_entry('joint_trajectory',destination_id+"_"+changed_destination)
                    self.clear_trajectory_cache_entry('joint_trajectory',changed_destination+"_"+destination_id)

                    # remove from plan data structure
                    if destination_id+"_"+changed_destination in self.tofs.keys():
                        rospy.loginfo("Removing "+destination_id+"_"+changed_destination)
                        self.tofs.pop(destination_id+"_"+changed_destination)
                    if changed_destination+"_"+destination_id in self.tofs.keys():
                        rospy.loginfo("Removing "+destination_id+"_"+changed_destination)
                        self.tofs.pop(changed_destination+"_"+destination_id)
                    rospy.loginfo(self.tofs.keys())

                # Ensure self-tofs are deleted
                try:
                    self.tofs.pop(changed_destination+"_"+changed_destination)
                except:
                    pass
        except Exception, e:
            print e
            traceback.print_exc(file=sys.stdout)
            raise e

    def _job_result(self, result):
        if result.type == 'tof_pose' or result.type == 'tof_gripper':
            if result.id in self.tofs.keys():
                tof = -2 if result.error else json.loads(result.result)['tof']
                self.tofs[result.id] = 'error' if tof == -2 else ('invalid' if tof == -1 else tof)

        elif result.type == 'validate_pose' or result.type == 'validate_gripper':
            if result.id in self.destinations.keys():
                if result.error:
                    self.destinations[result.id].reachable = False
                else:
                    self.destinations[result.id].reachable = json.loads(result.result)['valid']

        elif result.type == 'set_pose' or result.type == 'set_gripper':
            pass

        self.publish_cache_to_topic()

    def _job_submission_response(self, response):
        updated = False

        if response.type == 'tof_pose' or response.type == 'tof_gripper':
            if response.id in self.tofs.keys():
                if response.error:
                    updated = True
                    self.tofs[response.id] = 'error'

        elif response.type == 'validate_pose' or response.type == 'validate_gripper':
            if response.id in self.destinations.keys():
                if response.error:
                    updated = True
                    self.destinations[response.id].reachable = False

        elif response.type == 'set_pose' or response.type == 'set_gripper':
            if not response.error:
                rospy.loginfo('Set pose')
            else:
                rospy.loginfo('Did not set pose')

        if updated:
            self.publish_cache_to_topic()

    def _verify(self, async=True, log=True):
        if async:
            if self.verify_thread_lock.locked():
                self.needs_verification = True
                rospy.loginfo("Verify thread has received a request, but is currently running.")
            else:
                self.needs_verification = False
                self.verify_thread_lock.acquire()
                self._verify(async=False,log=False)
                self.verify_thread_lock.release()
                if self.needs_verification:
                    rospy.loginfo("Verify thread has received a request to update since starting. Starting again.")
                    self._verify(async=True,log=False)
                self.needs_verification = False
        else:
            if log:
                rospy.loginfo("Verifying plan (sync).")
            if self.verifier_type == 'advanced':
                v = AdvancedVerifier(self.plan_dict)
            else:
                v = OrigVerifier(self.plan_dict)
            error_count, errors = v.check()
            for therblig in errors.keys():
                if therblig in self.therbligs.keys():
                    self.therbligs[therblig].verification_errors = errors[therblig]
            if log:
                rospy.loginfo("Verified plan.")
            self.publish_cache_to_topic()

    def _expand(self):
        rospy.loginfo("producing expanded plan")
        pd = self.plan_dict
        if self.verifier_type == 'advanced':
            v = AdvancedVerifier(pd)
        else:
            v = OrigVerifier(pd)
        verify_error_count, verify_errors = v.check()
        valid=False
        errors = {}
        rospy.loginfo("Verify Errors: {0}".format(verify_error_count))
        if verify_error_count == 0:
            param_error_count, param_errors = param_check(pd,enforce_agents=False)
            rospy.loginfo("Param Errors: {0}".format(param_error_count))
            if param_error_count == 0:
                v = VerOpt(pd["agents"].keys(),pd["things"].keys(),pd["destinations"].keys())
                valid, errors, trace, expanded, _, _, _ = v.plan(pd)
                rospy.loginfo(valid)
                rospy.loginfo(trace)
            else:
                expanded = {}
                rospy.loginfo("Param Errors: {0}".format(param_errors))
        else:
            expanded = {}
            param_error_count = 0
            param_errors = {}
            rospy.loginfo("Verify Errors: {0}".format(verify_errors))

            print("\n\n\n\nplan errors\n\n\n\n\n")
            print(errors)
        return {"verify_error_count":verify_error_count,
                "param_error_count":param_error_count,
                "verify_errors":verify_errors,
                "param_errors":param_errors,
                "expanded_plan":expanded,
                "veropt_is_valid":valid,
                "veropt_errors":errors}

    def _visual_marker_set_pose(self, message):
        self.visual_marker_server.set_pose(message.id,message.pose)

    def _set_visible_marker_set(self, message):
        if message.data == "all":
            self.marker_server.visible(True)
            self.visual_marker_server.visible(True)
        elif message.data == 'interactive':
            self.marker_server.visible(True)
            self.visual_marker_server.visible(False)
        else:
            self.marker_server.visible(False)
            self.visual_marker_server.visible(True)


if __name__ == "__main__":
    authr_plan = AuthrPlan()
    rospy.loginfo("Authr Plan is running.")
    rospy.spin()
    authr_plan.end_update_thread()
