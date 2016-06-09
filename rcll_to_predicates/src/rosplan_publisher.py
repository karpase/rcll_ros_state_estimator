from common import *
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue


rosplan_knowledge_update_service = 'knowledge_update_service'

class ROSPlanPublisher:
	ADD_KNOWLEDGE = 0

	def __init__(self, service_name = rosplan_knowledge_update_service):
		self.service_name = service_name		
		rospy.wait_for_service(service)
		self.update_knowledge = rospy.ServiceProxy(self.service_name, KnowledgeUpdateService)


	def publish_predicates(self, preds):		
		for p in preds:
			k = KnowledgeItem()
			k.knowledge_type = k.FACT
			k.attribute_name = p.head
			for i in xrange(len(p.args)):
				kv = KeyValue()
				kv.key = 'arg_' + str(i)
				kv.value = p.args[i]
				k.values.append(kv)
			self.update_knowledge(self.ADD_KNOWLEDGE, k)