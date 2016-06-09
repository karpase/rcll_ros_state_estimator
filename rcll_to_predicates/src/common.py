import rospy
from std_msgs.msg import String
from pike_msgs.msg import Predicates

teams = {"M": "MagentaTeam", "C":"CyanTeam"}
machines = ["BS","DS","CS-1","CS-2","RS-1", "RS-2"]
sides = ["InputSide","OutputSide"]
max_num_materials_for_ring = 2
num_gates = 3
num_magazines = 3
num_orders = 9

pred_topic = '/Pike/Predicates'
order_topic = '/robot1/rcll/order_info'
ringinfo_topic = '/robot1/rcll/ring_info'
machineinfo_topic = '/robot1/rcll/machine_info'
pred_pub = rospy.Publisher(pred_topic, Predicates, queue_size=10)

def publish_predicates(preds):
	msg = Predicates()
	msg.predicates = map(lambda x: str(x), preds)
	msg.probabilities = map(lambda x: float(x.isTrue), preds)
	pred_pub.publish(msg)



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

class Predicate:
	def __init__(self, head, args=[], isTrue=True):
		self.head = head
		self.args = args
		self.isTrue = isTrue

	def __repr__(self):		
		return  "(" + self.head + " " + " ".join(self.args) + ")"
		