from common import *
from pike_msgs.msg import Predicates

pike_pred_topic = '/Pike/Predicates'

class PikePublisher:
	def __init__(self, topic = pike_pred_topic):
		self.pred_pub = rospy.Publisher(topic, Predicates, queue_size=10)


	def publish_predicates(self, preds):		
		msg = Predicates()
		msg.predicates = map(lambda x: str(x), preds)
		msg.probabilities = map(lambda x: float(x.isTrue), preds)
		self.pred_pub.publish(msg)	