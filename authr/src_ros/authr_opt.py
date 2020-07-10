#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from subprocess import call
from authr.srv import OptimizeQuery, OptimizeQueryResponse
from authr_tools.opt import plan_to_pddl
import os
import json
import re
import sys
import glob
from pprint import pprint

# 2018
# John Balis
# for support, email balisujohn@gmail.com


class AuthrOpt:




	def __init__(self):
		self.is_busy = False
		self.domain = os.path.dirname(os.path.realpath(__file__))+ '/../tmp/domain.pddl'
		self.instance = os.path.dirname(os.path.realpath(__file__))+ '/../tmp/instance.pddl'
		self.client_ID_dict = {}

		# self.domaintemplate = """
		# (define (domain %s)
		# (:requirements :typing :durative-actions :action-costs :typing)
		# (:types human robot - agent  %s - constraint world)
		# (:predicates %s  (unconstrained ?a - agent) (free ?a - agent)(constrained ?a - agent) (busy ?a - agent) (nonecomplete ?w - world)%s )
		#
		# (:functions (total-cost))
		#
		# %s
		#
		# ) """
		#
		# self.action_template = """
		# (:durative-action %s
		#     :parameters%s
		#     :duration (= ?duration %s)
		#     :condition(and %s)
		#     :effect(and %s  (increase (total-cost)  %s )  )
		# ) """
		#
		# self.instance_template = """
		# (define (problem %sins)
		# (:domain %s)
		# (:objects
		# w - world
		# human1 - human
		# robot1 - robot
		# %s
		#
		# )
		# (:init ( nonecomplete w)(free human1)(free robot1)(unconstrained human1)(unconstrained robot1))
		# (:goal (and %s ))
		#
		#
		# (:metric (and (minimize (total-time))(minimize(total-cost))))
		#
		#
		# )
		# """

		rospy.init_node('authr_opt', anonymous=True)
		self.opt_srv = rospy.Service("authr_opt/optimize", OptimizeQuery, self.optimize)
		# Migrated below to be a service
		# rospy.Subscriber("optimization_queue", Constraints, self.callback)
		# self.pub = rospy.Publisher('result_optimizations', Plan, queue_size=10)


	###
	#translates raw output of MODAE-YAHSP planner to JSON broadcasts it to result_optimizations
	###
	def optimize(self, message):
		id = message.id
		if self.is_busy:
			return OptimizeQueryResponse(id,"Optimizer Busy",{})
		else:
			self.is_busy = True


		# Create Files
		if os.path.exists(self.domain):
			os.remove(self.domain)
		if os.path.exists(self.instance):
			os.remove(self.instance)
		for file_name in glob.glob(os.path.dirname(os.path.realpath(__file__))+ '/../tmp/plan.soln.*'):
			if os.path.exists(file_name):
				os.remove(file_name)

		self.generate_files(message.constraints)

		# Call Optimizer
		os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/../tmp/')
		call([os.path.dirname(os.path.realpath(__file__))+ '/../planner/descarwin/build/dae/dae', '--domain=' + self.domain , '--instance=' + self.instance,  '-r=5'])


		best_plan_cost = float("inf")
		best_plan = None
		plan_list = []

		data = {}
		#data = message.constraints


		domain_data = json.loads(message.constraints)
		#pprint(domain_data)


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
				    #print(tokens)
				    data['plan'][tokens[1][1:-5]] = {'starttime': re.sub('[\:]', '', tokens[0]),
				                                     'agent': tokens[3][0],
				                                     'duration': tokens[len(tokens) - 2].strip('[]\n'),
								     'cost' : tokens[len(tokens)-1].strip('[]\n')}
				    #print(line)
				    if counter == len(lines) - 2:
				        data['time'] = float(re.sub('[\:]', '', tokens[0])) + float(
				            tokens[len(tokens) - 2].strip('[]\n'))
				counter += 1
			    #print(data)

			    cost = float(data['cost'])
			    duration = float(data['time'])
			    plan_cost = (cost * domain_data['costweight']) +( duration * domain_data['timeweight'])
			    if plan_cost < best_plan_cost:
				best_plan = data
				best_plan_cost = plan_cost

		#rospy.loginfo(best_plan)
		constraints = str(data)
		rospy.loginfo("Optimization Success")
		self.is_busy = False

		return OptimizeQueryResponse(id,"Optimizer Success",constraints)





	###
	#generates predicates to specify task completion to the PDDL planner
	###
	def get_predicates(self,tasks):
	    output = ''
	    for task, items in tasks.items():
		output += '\n( ' + task + 'complete ?w - world)'
	    return output

	###
	#generates predicates to specify task ordering constraints
	###
	def get_constraint_predicates(self,constraints):
	    output=''
	    for constraint in constraints:
		output += '(constrained'+constraint+' ?a - agent  ?c - ' + constraint + ' )'
	    return output

	###
	#generates PDDL actions to specify desired tasks(email balisujohn@gmail.com for questions)
	###
	def create_action_list(self,tasks):
	    action_list = ''
	    for name, task in tasks.items():


		for agenttype, duration in task['duration'].items():

			if duration != -1:
				param_template = '( ?w - world  ?a - '+ agenttype+ ' %s )'

				condition_string = ' (at start(free ?a))'
				param_string = param_template % ''
				effect_string = '(at start (busy ?a))(at start(not (free ?a)))(at end (not(busy ?a)))(at end(free ?a))( at end ( ' + name + 'complete ' + '?w))'

				cost = task['cost'][agenttype]
				constraint = task['constraint'].replace(' ' ,'').split(':')
	       			if constraint[0] == 'none':
		        		condition_string += '(at start(unconstrained ?a))'
	       			elif constraint[0] == 'start':
		    			condition_string += '(at start(unconstrained ?a))'
		   			effect_string += '(at end (constrained ?a))(at end(not(unconstrained ?a)))(at end (constrained'+constraint[1]+' ?a ?c))'
		    			param_string = param_template % ('?c - ' + constraint[1] )
				elif constraint[0] == 'end':
	       		        	param_string = param_template % ('?c - ' + constraint[1] )
					effect_string += '(at end (not(constrained ?a)))(at end(unconstrained ?a))(at end (not(constrained'+constraint[1]+' ?a ?c)))'
		       			condition_string += '(at start (constrained ?a))(at start (constrained'+constraint[1]+' ?a ?c))'
				else:
		    			param_string = param_template % ('?c - ' + constraint[0] )
		    			condition_string += '(at start (constrained ?a))(at start (constrained'+constraint[0]+' ?a ?c))'


				for precondition in task['preconditions'].split(" "):
		    			condition_string += '(at start ( ' + precondition + 'complete ?w ))'

				new_action = self.action_template % (name + agenttype, param_string , duration , condition_string, effect_string,cost)
				action_list += '\n' + new_action

	    return action_list

	###
	#generates PDDL goal list from task list
	###
	def create_goal_list(self ,tasks):
	    output = ''
	    for name, task in tasks.items():
		output += '\n(' + name + 'complete w)'
	    return output
	###
	#obtains a list of constraints from the list of tasks looks specifically for the start keyword
	#constraints chains not starting with "start : <constraintname>" will produce undefined behaviour
	###
	def get_constraints(self, tasks):
	    constraints = []
	    for name, task in tasks.items():
		setting = task['constraint'].replace(' ', '').split(':')
		if setting[0] == 'start':
		    if setting[1] not in constraints:
		        constraints.append(setting[1])
	    return constraints

	###
	#creates list of constraints for domain file
	###
	def create_constraint_string(self, constraints):
	    output = ''
	    counter = 0
	    for constraint in constraints:
		output += 'p' + str(counter) + ' - ' + constraint + '\n'
		counter += 1
	    return output
	###
	#creates list of object types for instance file
	###
	def create_type_string(self, obj_types):
	    output = ''
	    for obj_type in obj_types:
		output += obj_type + ' '
	    return output

	###
	#generates files for pddl planner
	###
	def generate_files(self, data_string):

		data = json.loads(data_string)

		#generating necesarry strings
		# constraint_list = self.get_constraints(domain_data['tasks'])
		# constraint_predicate_list = self.get_constraint_predicates(constraint_list)
		# predicate_string = self.get_predicates(domain_data['tasks'])
		# constraint_type_string = self.create_type_string(constraint_list)
		# action_string = self.create_action_list(domain_data['tasks'])
		# constraint_string = self.create_constraint_string(constraint_list)
		# goal_string = self.create_goal_list(domain_data['tasks'])
		#
		# #domain and instance created from templates
		# domain_output = self.domaintemplate % (domain_data['name'],constraint_type_string,constraint_predicate_list , predicate_string, action_string)
		# instance_output = self.instance_template % (domain_data['name'],domain_data['name'],constraint_string, goal_string)
		domain_output, instance_output = plan_to_pddl(data)

		rospy.loginfo(domain_output)
		rospy.loginfo(instance_output)

		domfile = open(os.path.dirname(os.path.realpath(__file__))+ '/../tmp/domain.pddl', 'w')
		domfile.write(domain_output)
		domfile.close()

		insfile = open(os.path.dirname(os.path.realpath(__file__)) + '/../tmp/instance.pddl', 'w')
		insfile.write(instance_output)
		insfile.close()







if __name__ == '__main__':
	authr_opt = AuthrOpt()
	rospy.loginfo("Authr Opt is running.")
	rospy.spin()
