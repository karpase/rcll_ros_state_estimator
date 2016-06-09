import rospy
from common import *

objects = {}

	
def publish_initial_predicates():	
	preds = []	
	
	for i in xrange(num_orders):
		preds.append(Predicate("filled", [order_to_object(i)]))

	for team in teams.keys():
		for i in xrange(1,4):
			preds.append(Predicate("empty-gripper", [robot_to_object(team, i)]))
			preds.append(Predicate("atMachine", [robot_to_object(team, i), team + "-" + "Start"]))
			preds.append(Predicate("atMachineSide", [robot_to_object(team, i), "InputSide"]))
			
		for machine in machines:
			for side in sides:
				preds.append(Predicate("machineSideClear", [team + "-" + machine, side]))
		for m in rs_machines:
			preds.append(Predicate("numberOfLoadedBases", [team + "-" + m, number_to_object(0)]))
		for m in cs_machines:			
			preds.append(Predicate("slideClear", [team + "-" + m]))
			
		for p in shelfpos:
			for m in cs_machines:			
				preds.append(Predicate("shelfPosEmpty", [team + "-" + m, p]))				

		for c in color_dict["cap"].keys():
			for i in xrange(num_spare_caps):		
				base_obj = team + "_" + "sc_b_" + str(i) + "_" + color_dict["cap"][c]
				cap_obj = team + "_" + "sc_c_" + str(i) + "_" + color_dict["cap"][c]
				preds.append(Predicate("above",[cap_obj, base_obj]))
				preds.append(Predicate("baseof",[cap_obj, base_obj]))
				preds.append(Predicate("clear",[cap_obj]))
				preds.append(Predicate("color",[cap_obj, color_to_object("cap", c)]))
				preds.append(Predicate("color",[base_obj, color_to_object("base", spare_cap_base_color)]))

				objects[base_obj] = "BaseWorkpiece"
				objects[cap_obj] = "CapWorkpiece"

		for c in color_dict["ring"].keys():
			for i in xrange(num_spare_rings):
				for m in rs_machines:
					ring_obj = team + "_" + "sr_r_" + str(i) + "_" + color_dict["ring"][c] + "_" + m	
					preds.append(Predicate("canPlaceRing",[team + "-" + m, ring_obj]))			
					preds.append(Predicate("color",[ring_obj, color_to_object("ring", c)]))
					objects[ring_obj] = "RingWorkpiece"
				
		for c in color_dict["base"].keys():
			magazine = team + "-" + "BSMagazine" + str(c)
			magazine_end = team + "-" + "MagazineEndWorkpiece" + str(c)
			base_objs = map(lambda i: team + "_" + "sb_b_" + str(i) + "_" + color_dict["base"][c], xrange(num_spare_bases))
			preds.append(Predicate("magazineTop",[magazine, base_objs[0]]))			
			for i in xrange(num_spare_bases-1):				
				preds.append(Predicate("nextInMagazine",[magazine, base_objs[i], base_objs[i+1]]))
			preds.append(Predicate("nextInMagazine",[magazine, base_objs[num_spare_bases-1], magazine_end]))
			for base_obj in base_objs:
				objects[base_obj] = "BaseWorkpiece"

	publish_predicates(preds)		

def main():
	rospy.init_node('initial_predicates', anonymous=True)	   
	publish_initial_predicates()
	rospy.sleep(1.0)
	for o in objects:
		print o + " - " + objects[o]

if __name__ == "__main__":
    main()	