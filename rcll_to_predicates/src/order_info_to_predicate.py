import rospy
from rcll_ros_msgs.msg import *
from std_msgs.msg import String
#from pike_msgs.msg import Predicates

teams = {"M": "MagentaTeam", "C":"CyanTeam"}
machines = ["BS","DS","CS-1","CS-2","RS-1", "RS-2"]
sides = ["InputSide","OutputSide"]
max_num_materials_for_ring = 2
num_gates = 3
num_magazines = 3

def pred_to_string(head, args):
	return  "(" + head + " " + " ".join(args) + ")"

def gate_to_object(gate):
	return "gate" + str(gate)

def color_to_object(color):
	return "color" + str(color)

def number_to_object(num):
	return "n" + str(num)

def order_to_object(order_id):
	return "order" + str(order_id)

def robot_to_object(team, robot_id):
	return team + "-Robot" + str(robot_id)

def magazine_to_object(team, magazine_id):
	return team + "-BSMagazine" + str(magazine_id)	

def publish_predicates(preds):
	pred_pub.publish(" ".join(preds))

def order_to_predicates(order):
	obj = order_to_object(order.id)
	complexity = pred_to_string("orderType", [obj, "OrderType" + str(order.complexity)])
	base = pred_to_string("orderBaseColor", [obj, color_to_object(order.base_color)])
	cap = pred_to_string("orderCapColor", [obj, color_to_object(order.cap_color)])
	delivery = pred_to_string("orderPossibleDeliverySlide", [obj, gate_to_object(order.delivery_gate)])
	rings = []
	for i in xrange(order.complexity):
		rings.append(pred_to_string("orderRing" + str(i+1) + "Color", [obj, color_to_object(order.ring_colors[i])]))
	preds = [complexity, base, cap, delivery] + rings
	# handle filled(order) - output it as true for orders that are fulfilled or haven't appeared yet, and false for orders that have appeared and are pending
	return preds

def process_order_info(order_info):
	preds = []
	for order in order_info.orders:
		preds += order_to_predicates(order)
	publish_predicates(preds)

def process_ring_info(ring_info):
	preds = []
	for r in ring_info.rings:
		preds.append(pred_to_string("numBasesForColor", [color_to_object(r.ring_color), number_to_object(r.raw_material)]))
	publish_predicates(preds)

def process_machine_info(machine_info):
	preds = []
	for m in machine_info.machines:
		if m.type == "RS":
			for c in m.rs_ring_colors:
				preds.append(pred_to_string("hasColor", [m.name, color_to_object(c)]))
	publish_predicates(preds)

def publish_constant_predicates():	
	preds = []
	preds += map(lambda i: pred_to_string("next",[number_to_object(i), number_to_object(i+1)]), xrange(max_num_materials_for_ring))	
	for team in teams.keys():
		for i in xrange(1,4):
			preds.append(pred_to_string("robotOwner", [robot_to_object(team, i), teams[team]]))
		for machine in machines:
			preds.append(pred_to_string("machineOwner", [team + "-" + machine, teams[team]]))
		preds.append(pred_to_string("hasShelf", [team + "-" + "CS1"]))
		preds.append(pred_to_string("hasShelf", [team + "-" + "CS2"]))
		for i in xrange(num_gates):
			preds.append(pred_to_string("slideOn", [team + "-" + "DS", gate_to_object(i)]))
		for i in xrange(num_magazines):
			preds.append(pred_to_string("magazineInMachineSide", [magazine_to_object(team, i) ,team + "-" + "BS", "OutputSide"]))
	preds.append(pred_to_string("control","CyanTeam")) # todo: how do we know which team we are?
	publish_predicates(preds)	
	
def publish_initial_predicates():	
	preds = []	
	
	for team in teams.keys():
		for i in xrange(1,4):
			preds.append(pred_to_string("empty-gripper", [robot_to_object(team, i)]))
		for machine in machines:
			for side in sides:
				preds.append(pred_to_string("machineSideClear", [team + "-" + machine, side]))
		preds.append(pred_to_string("numberOfLoadedBases", [team + "-" + "RS1", number_to_object(0)]))
		preds.append(pred_to_string("numberOfLoadedBases", [team + "-" + "RS2", number_to_object(0)]))		
		preds.append(pred_to_string("slideClear", [team + "-" + "CS1"]))
		preds.append(pred_to_string("slideClear", [team + "-" + "CS2"]))

	
	publish_predicates(preds)		


pred_topic = '/Pike/Predicates'
order_topic = '/robot1/rcll/order_info'
ringinfo_topic = '/robot1/rcll/ring_info'
machineinfo_topic = '/robot1/rcll/machine_info'

rospy.init_node('order_info_to_predicate', anonymous=True)
pred_pub = rospy.Publisher(pred_topic, String, queue_size=10)
order_sub = rospy.Subscriber(order_topic, OrderInfo, process_order_info)
ringinfo_sub = rospy.Subscriber(ringinfo_topic, RingInfo, process_ring_info)
machineinfo_sub = rospy.Subscriber(machineinfo_topic, MachineInfo, process_machine_info)


rospy.spin()