#!/usr/bin/env python

import roslib; roslib.load_manifest('tablet_broker')
import rospy
import lcm
from time import *
from threading import Thread
from lcmtypes import SpeechList, TextToSpeechMsg
from std_msgs.msg import String
from speech_msgs.msg import Speech

class TabletBroker():

	def __init__(self):
		self.lc = lcm.LCM()
		self.lcm_subscription = self.lc.subscribe("SPEECH", self.lcm_handle)
		self.ros_subscription = rospy.Subscriber('text_to_speech', String, self.tts_callback)
		self.ros_publisher = rospy.Publisher('speech_results', Speech)
		rospy.init_node('tablet_broker')
	
	def lcm_handle(self, channel, data):
		msg = SpeechList.decode(data)
                text = []
		conf = []
		print "Received Message:"
		for s in msg.results:
			print '\t'+s.text
			text.append(s.text)
      			print '\t'+str(s.confidence)
			conf.append(s.confidence)
		ros_msg = Speech(tuple(text), tuple(conf))
		self.ros_publisher.publish(ros_msg)

	def tts_callback(self, data):
		text = data.data
		print 'Sent Message:'
		print '\t'+text
		msg = TextToSpeechMsg()
  		msg.text = text
  		msg.timestamp = time()
		self.lc.publish("SPEAK",msg.encode())

	def run_ros(self):
		rospy.init_node('tablet_broker')
		rospy.spin()
	
	def run_lcm(self):
		while True:
			self.lc.handle()

broker = TabletBroker()
print "=== Starting LCM broker ==="

t1 = Thread(target=broker.run_lcm, args=())
t1.daemon=True
t1.start()
rospy.spin()

print '\n=== Closing LCM broker ==='
