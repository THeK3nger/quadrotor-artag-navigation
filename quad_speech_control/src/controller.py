#!/usr/bin/env python

import roslib; roslib.load_manifest('quad_speech_control')
import rospy
from std_msgs.msg import String
from speech_msgs.msg import Speech
import operator
from bag_of_words import *
import os


def callback(data):
	result=[]
	for i in data.msg:
		dictionary = sacca.infer_label(i)
		label = max(dictionary, key=lambda x: dictionary[x])
		result.append((label, dictionary[label]))
	msg = max(result, key=lambda x: x[1])[0]
	pub.publish(msg)


rospy.init_node('quad_speech_control')
sacca = bag_of_words(os.path.dirname(os.path.abspath(__file__))+'/../data/quad_corpus.txt', 1)
pub = rospy.Publisher('/ar_marker_switch', String)
rospy.Subscriber("speech_results", Speech, callback)
rospy.spin()
