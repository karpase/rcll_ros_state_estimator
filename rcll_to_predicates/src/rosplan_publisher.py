from common import *
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, GetDomainAttributeService
from rosplan_knowledge_msgs.msg import KnowledgeItem, DomainFormula
from diagnostic_msgs.msg import KeyValue


rosplan_knowledge_update_service_array = 'kcl_rosplan/update_knowledge_base_array'
rosplan_get_domain_predicates_service = '/kcl_rosplan/get_domain_predicates'

class ROSPlanPublisher:
	ADD_KNOWLEDGE = 0

	def __init__(self):
		rospy.wait_for_service(rosplan_knowledge_update_service_array)
		self.update_knowledge_array = rospy.ServiceProxy(rosplan_knowledge_update_service_array, KnowledgeUpdateServiceArray)
		self.update_predicate_info()

	def update_predicate_info(self):
		self.predicate_key_name = {}

		rospy.wait_for_service(rosplan_get_domain_predicates_service)
		domain_predicates_service_proxy = rospy.ServiceProxy(rosplan_get_domain_predicates_service, GetDomainAttributeService)
		pred_info_list = domain_predicates_service_proxy()
		for pred_info in pred_info_list.items:
			i = 0
			for tp in pred_info.typed_parameters:
				self.predicate_key_name[(pred_info.name, i)] = tp.key
				print (pred_info.name, i), " = ", tp.key
				i = i + 1


	def publish(self, ka):
		self.update_knowledge_array(self.ADD_KNOWLEDGE, ka)	
		

	def publish_predicates(self, preds):
		ka = []	
		for p in preds:
			k = KnowledgeItem()
			k.knowledge_type = k.FACT
			k.attribute_name = p.head.lower()
			for i in xrange(len(p.args)):
				kv = KeyValue()
				kv.key = self.predicate_key_name[(k.attribute_name, i)]
				kv.value = p.args[i].lower()
				k.values.append(kv)
			ka.append(k)
		self.publish(ka)
		


	def publish_instances(self, object_instances):
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
		self.publish(ka)
