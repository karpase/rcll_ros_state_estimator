import rospy
from common import *

def publish_constant_predicates():	
	preds = []
	preds += map(lambda i: Predicate("next",[number_to_object(i), number_to_object(i+1)]), xrange(max_num_materials_for_ring))	
	for team in teams.keys():
		for i in xrange(1,4):
			preds.append(Predicate("robotOwner", [robot_to_object(team, i), teams[team]]))
		for machine in machines:
			preds.append(Predicate("machineOwner", [team + "-" + machine, teams[team]]))
		preds.append(Predicate("hasShelf", [team + "-" + "CS1"]))
		preds.append(Predicate("hasShelf", [team + "-" + "CS2"]))
		for i in xrange(num_gates):
			preds.append(Predicate("slideOn", [team + "-" + "DS", gate_to_object(i)]))
		for i in xrange(num_magazines):
			preds.append(Predicate("magazineInMachineSide", [magazine_to_object(team, i) ,team + "-" + "BS", "OutputSide"]))
	preds.append(Predicate("control",["CyanTeam"])) # todo: how do we know which team we are?
	publish_predicates(preds)	


def main():
	rospy.init_node('constant_predicates', anonymous=True)	   
	rate = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		publish_constant_predicates()        
		rate.sleep()

if __name__ == "__main__":
    main()	