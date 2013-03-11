#!/usr/bin/env python

import roslib; roslib.load_manifest('quad_speech_control')
import rospy
from std_msgs.msg import String
from speech_msgs.msg import Speech
import operator
from bag_of_words import *
import os
import lcm
from threading import Thread

class Controller:
	def __init__(self):
		self.lc=lcm.LCM()
		self.filename = os.path.dirname(os.path.abspath(__file__))+'/../data/quad_corpus.txt'
		self.bag = bag_of_words(self.filename, 1)
		self.pub = rospy.Publisher('/ar_marker_switch', String)
		self.speechPub = rospy.Publisher('/text_to_speech', String)
		self.subscription = self.lc.subscribe("CHOICE", self.lcmCallback)
		rospy.Subscriber("/speech_results", Speech, self.rosCallback)


	def rosCallback(self, data):
		result=[]
		self.lastMsg = data
		for i in data.msg:
			dictionary = self.bag.infer_label(i)
			print dictionary
			label = max(dictionary, key=lambda x: dictionary[x])
			result.append((label, dictionary[label]))
		msg = max(result, key=lambda x: x[1])[0]
		print result
		print "CONFIDENCE: "+ str(max(result, key=lambda x: x[1])[1])
		if max(result, key=lambda x: x[1])[1] < 0.7:
			fake = ''
			self.lc.publish("CONTROL", fake)
			self.speechPub.publish("I did not understand, select one of the following")
		else:	
			self.pub.publish(msg)
			print msg

	def lcmCallback(self, channel, data):
    		print data
		print self.lastMsg
		corpusFile = open(self.filename,'a')
		for i in self.lastMsg.msg:
			toAdd = i+" "+data.replace("\00", "")+"\n"
			corpusFile.write(toAdd)
		corpusFile.close()
		self.bag = bag_of_words(self.filename, 1)
		self.pub.publish(data.replace("\00", ""))
	
	def lcmHandler(self):
		while True:
        		self.lc.handle()

	def rosHandler(self):
		rospy.spin()


rospy.init_node('quad_speech_control')
myController = Controller()
#lc = lcm.LCM()

#sacca = bag_of_words(os.path.dirname(os.path.abspath(__file__))+'/../data/quad_corpus.txt', 1)
#pub = rospy.Publisher('/ar_marker_switch', String)
#rospy.Subscriber("speech_results", Speech, callback)
#rospy.spin()

t1=Thread(target=myController.rosHandler)
t2=Thread(target=myController.lcmHandler)

t1.start()
t2.start()               

