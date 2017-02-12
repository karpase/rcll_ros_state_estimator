from common import *
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue


rosplan_knowledge_update_service_array = 'kcl_rosplan/update_knowledge_base_array'
rosplan_knowledge_update_service = 'kcl_rosplan/update_knowledge_base'

class ROSPlanPublisher:
	ADD_KNOWLEDGE = 0

	def __init__(self):
		#rospy.wait_for_service(rosplan_knowledge_update_service)
		rospy.wait_for_service(rosplan_knowledge_update_service_array)
		self.update_knowledge_array = rospy.ServiceProxy(rosplan_knowledge_update_service_array, KnowledgeUpdateServiceArray)
		#self.update_knowledge = rospy.ServiceProxy(self.service_name, KnowledgeUpdateService)

	def publish(self, ka, array):
		self.update_knowledge_array(self.ADD_KNOWLEDGE, ka)	
		#if array:
		#	self.update_knowledge_array(self.ADD_KNOWLEDGE, ka)	
		#else:
		#	for k in ka:
		#		self.update_knowledge(self.ADD_KNOWLEDGE, k)	

	def publish_predicates(self, preds, array=True):
		ka = []	
		for p in preds:
			k = KnowledgeItem()
			k.knowledge_type = k.FACT
			k.attribute_name = p.head.lower()
			for i in xrange(len(p.args)):
				kv = KeyValue()
				kv.key = 'arg_' + str(i).lower()
				kv.value = p.args[i].lower()
				k.values.append(kv)
			ka.append(k)
		self.publish(ka, array)
		


	def publish_instances(self, object_instances, array=False):
		"""publish instances. input is dictionary where name is key and type is value"""
		print "adding: ", object_instances
		ka = []
		for obj_name in object_instances:
			k = KnowledgeItem()
			k.knowledge_type = k.INSTANCE
			k.instance_name = obj_name.lower()
			k.instance_type = object_instances[obj_name].lower()
			print k
			ka.append(k)
		self.publish(ka, array)
