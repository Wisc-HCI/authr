(define (problem CrewPlanning_2crew_2day_100utilization)
(:domain CrewPlanning)
(:objects
	d0 d1 d2 d3 - Day

	c1 c2 - CrewMember
	mcs1 mcs2 - MedicalState

	spaceshipFilter - FilterState

	rpcm1 - RPCM

	pa1_1 pa1_2 pa1_3 pa1_4 pa1_5 pa1_6 pa1_7 pa1_8 pa1_9 pa2_1 pa2_2 pa2_3 pa2_4 pa2_5 pa2_6 pa2_7 pa2_8 pa2_9 pa2_10 pa2_11 pa2_12 pa2_13 pa2_14 pa2_15 pa2_16 - PayloadAct

	e1 - ExerEquipment
)

(:init
	(currentday c1 d0)
	(done_sleep c1 d0)
	(available c1)
	(currentday c2 d0)
	(done_sleep c2 d0)
	(available c2)
	(initiated d1)
	(next d0 d1)
	(next d1 d2)
	(next d2 d3)
	
	(unused e1)
	)

(:goal
(and
	(done_sleep c1 d1)
	(done_sleep c1 d2)
	(done_sleep c2 d1)
	(done_sleep c2 d2)
	(initiated d3)

	(mcs_finished mcs2 d1)
	(mcs_finished mcs2 d2)

	(changed spaceshipFilter d2)

	(done_rpcm rpcm1 d1)

	(payload_act_completed pa1_1 d1)
	(payload_act_completed pa1_2 d1)
	(payload_act_completed pa1_3 d1)
	(payload_act_completed pa1_4 d1)
	(payload_act_completed pa1_5 d1)
	(payload_act_completed pa1_6 d1)
	(payload_act_completed pa1_7 d1)
	(payload_act_completed pa1_8 d1)
	(payload_act_completed pa1_9 d1)
	(payload_act_completed pa2_1 d2)
	(payload_act_completed pa2_2 d2)
	(payload_act_completed pa2_3 d2)
	(payload_act_completed pa2_4 d2)
	(payload_act_completed pa2_5 d2)
	(payload_act_completed pa2_6 d2)
	(payload_act_completed pa2_7 d2)
	(payload_act_completed pa2_8 d2)
	(payload_act_completed pa2_9 d2)
	(payload_act_completed pa2_10 d2)
	(payload_act_completed pa2_11 d2)
	(payload_act_completed pa2_12 d2)
	(payload_act_completed pa2_13 d2)
	(payload_act_completed pa2_14 d2)
	(payload_act_completed pa2_15 d2)
	(payload_act_completed pa2_16 d2)
)
)
(:metric (and (minimize (total-time)) (minimize (total-cost))))
)
