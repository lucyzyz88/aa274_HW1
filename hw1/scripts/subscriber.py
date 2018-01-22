#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo('Received: %s', data.data)
def subscriber():
	rospy.init_node('subscriber', anonymous=True)
	### YOUR CODE HERE ###
	#Get message
	rospy.Subscriber('random_strings',String, callback)
	rospy.loginfo('Listening on the random_strings topic...')
	### END OF YOUR CODE ###
	rospy.spin()
if __name__ == '__main__':
	subscriber()
