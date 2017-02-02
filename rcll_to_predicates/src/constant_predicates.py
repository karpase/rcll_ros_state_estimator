#!/usr/bin/env python

import rospy
from common import *
from rosplan_publisher import *

def generate_constant_predicates():	
	preds = []
	preds += map(lambda i: Predicate("next",[number_to_object(i), number_to_object(i+1)]), xrange(max_num_materials_for_ring))	
	for team in teams.keys():
		for i in xrange(1,4):
			preds.append(Predicate("robotOwner", [robot_to_object(team, i), teams[team]]))
		for machine in machines:
			preds.append(Predicate("machineOwner", [team + "-" + machine, teams[team]]))
		for m in cs_machines:
			preds.append(Predicate("hasShelf", [team + "-" + m]))
			preds.append(Predicate("hasShelf", [team + "-" + m]))
		for i in xrange(num_gates):
			preds.append(Predicate("slideOn", [team + "-" + "DS", gate_to_object(i)]))
		for c in color_dict["base"].keys():
			magazine = team + "-" + "BSMagazine" + str(c)		
			preds.append(Predicate("magazineInMachineSide", [magazine, team + "-" + "BS", "OutputSide"]))
	preds.append(Predicate("control",["CyanTeam"])) # todo: how do we know which team we are?
	return preds


def main():	
	rospy.init_node('constant_predicates', anonymous=True)	   

	publisher = ROSPlanPublisher()
	rate = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		preds = generate_constant_predicates()        
		publisher.publish_predicates(preds)
		rate.sleep()

if __name__ == "__main__":
    main()	
