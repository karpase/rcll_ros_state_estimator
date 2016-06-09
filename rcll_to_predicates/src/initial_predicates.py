import rospy
from common import *

	
def publish_initial_predicates():	
	preds = []	
	
	for team in teams.keys():
		for i in xrange(1,4):
			preds.append(Predicate("empty-gripper", [robot_to_object(team, i)]))
		for machine in machines:
			for side in sides:
				preds.append(Predicate("machineSideClear", [team + "-" + machine, side]))
		preds.append(Predicate("numberOfLoadedBases", [team + "-" + "RS1", number_to_object(0)]))
		preds.append(Predicate("numberOfLoadedBases", [team + "-" + "RS2", number_to_object(0)]))		
		preds.append(Predicate("slideClear", [team + "-" + "CS1"]))
		preds.append(Predicate("slideClear", [team + "-" + "CS2"]))
		for p in shelfpos:
			preds.append(Predicate("shelfPosEmpty", [team + "-" + "CS1", p]))
			preds.append(Predicate("shelfPosEmpty", [team + "-" + "CS2", p]))

	for i in xrange(num_orders):
		preds.append(Predicate("filled", [order_to_object(i)]))
	
	publish_predicates(preds)		

def main():
	rospy.init_node('initial_predicates', anonymous=True)	   
	publish_initial_predicates()
	rospy.sleep(1.0)

if __name__ == "__main__":
    main()	