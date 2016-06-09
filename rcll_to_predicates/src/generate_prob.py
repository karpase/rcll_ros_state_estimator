from common import *

from initial_predicates import generate_initial_predicates_and_objects
from constant_predicates import generate_constant_predicates

def main():
	init_preds, objects = generate_initial_predicates_and_objects()
	const_preds = generate_constant_predicates()

	preds = const_preds + init_preds

	print "(define (problem Planning_Problem)"
	print "	(:domain RCLL)"
	print "	(:objects"
	for o in objects:
		print o + " - " + objects[o]
	print "	)"
	print "	(:init"
	for p in preds:
		print p
	print "	)"
	print "	(:goal"
	print "		(and "
	for i in xrange(num_orders):
		print Predicate("filled", [order_to_object(i)])
	print "      )"
	print "	)    "
	print ")"
	

	

if __name__ == "__main__":
    main()	
