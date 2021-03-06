import rospy
from std_msgs.msg import String


teams = {"M": "MagentaTeam", "C":"CyanTeam"}

cs_machines = ["CS1","CS2"]
rs_machines = ["RS1", "RS2"]
machines = ["BS","DS"] + cs_machines + rs_machines

machine_states = ["IDLE", "BROKEN", "PREPARED", "PROCESSING", "PROCESSED", "READY-AT-OUTPUT", "WAIT-IDLE", "DOWN"]


sides = ["InputSide","OutputSide"]
shelfpos = ["ShelfLeft","ShelfMiddle","ShelfRight"]
max_num_materials_for_ring = 2
num_gates = 3
num_magazines = 3
num_orders = 10

num_spare_caps = 20
spare_cap_base_color = 1

num_spare_rings = 20

num_spare_bases = 20


color_dict = {
	"ring": {
		1: "blue",
		2: "green",
		3: "orange",
		4: "yellow"		
	},
	"base": {
		1: "red",
		2: "black",
		3: "silver"
	},
	"cap": {
		1: "black",
		2: "grey"
	}
}






def gate_to_object(gate):
	return "gate" + str(gate)

def color_to_object(type, color):
	return color_dict[type][color]	

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
		if self.isTrue:
			return  "(" + self.head + " " + " ".join(self.args) + ")"
		else:
			return  "(not (" + self.head + " " + " ".join(self.args) + "))"
class Instance:
	def __init__(self, name, obj_type):
		self.name = name
		self.obj_type = obj_type

	def __repr__(self):
		return self.name + " - " + self.obj_type
		
