#!/usr/bin/env python
import rospy
from std_msgs.msg import String
def publisher():
	pub = rospy.Publisher('random_strings',String, queue_size = 10)
	rospy.init_node('publisher',anonymous = True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		### YOUR CODE HERE ###
		#print out First and Last Name
		msg = "Lucy Zhuang"
		pub.publish(msg)
		### END OF YOUR CODE ###
		rate.sleep()
if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass