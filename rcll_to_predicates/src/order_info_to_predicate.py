#!/usr/bin/env python

import rospy
from rcll_ros_msgs.msg import *
from std_msgs.msg import String
from common import *
from rosplan_publisher import *

order_topic = '/robot1/rcll/order_info'
ringinfo_topic = '/robot1/rcll/ring_info'
machineinfo_topic = '/robot1/rcll/machine_info'

publisher = ROSPlanPublisher()

def order_to_predicates(order):
	obj = order_to_object(order.id)
	complexity = Predicate("orderType", [obj, "OrderType" + str(order.complexity)])
	base = Predicate("orderBaseColor", [obj, color_to_object("base", order.base_color)])
	cap = Predicate("orderCapColor", [obj, color_to_object("cap", order.cap_color)])
	delivery = Predicate("orderPossibleDeliverySlide", [obj, gate_to_object(order.delivery_gate)])
	rings = []
	for i in xrange(order.complexity):
		rings.append(Predicate("orderRing" + str(i+1) + "Color", [obj, color_to_object("ring", order.ring_colors[i])]))
	filled = Predicate("filled", [obj], False)
	preds = [filled, complexity, base, cap, delivery] + rings
	# handle filled(order) - output it as true for orders that are fulfilled or haven't appeared yet, and false for orders that have appeared and are pending
	return preds

def process_order_info(order_info):
	preds = []
	for order in order_info.orders:
		preds += order_to_predicates(order)
	publisher.publish_predicates(preds)

def process_ring_info(ring_info):
	preds = []
	for r in ring_info.rings:
		preds.append(Predicate("numBasesForColor", [color_to_object("ring", r.ring_color), number_to_object(r.raw_material)]))
	publisher.publish_predicates(preds)

def process_machine_info(machine_info):
	preds = []
	for m in machine_info.machines:
		if m.type == "RS":
			for c in m.rs_ring_colors:
				preds.append(Predicate("hasColor", [m.name, color_to_object("ring", c)]))
		preds.append(Predicate("machineState", [m.name, m.state]))
		for ms in machine_states:
			if ms != m.state:
				preds.append(Predicate("machineState", [m.name, ms], False)))
	publisher.publish_predicates(preds)

def main():
	rospy.init_node('order_info_to_predicate', anonymous=True)
	
	order_sub = rospy.Subscriber(order_topic, OrderInfo, process_order_info)
	ringinfo_sub = rospy.Subscriber(ringinfo_topic, RingInfo, process_ring_info)
	machineinfo_sub = rospy.Subscriber(machineinfo_topic, MachineInfo, process_machine_info)

	rospy.spin()

if __name__ == "__main__":
    main()	
