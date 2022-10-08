#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


def LaserData(msg):

	print("///////////////////////////////////////////////")
	laser = []
	# print(type(msg))
	# print(msg.ranges)
	# print(len(msg.ranges))
	# print('180',msg.ranges[180])
	# print('len',len(msg.ranges)/2)
	range_center = msg.ranges[int(len(msg.ranges)/2)]
	# print('centre',range_center)
	range_left = msg.ranges[len(msg.ranges)-1]
	# print('left', range_left)
	range_right = msg.ranges[0]
	# print('right',range_right)
	print ("range ahead: left - %0.1f" %range_left, " center- %0.1f" %range_center," right - %0.1f" %range_right)





def main():
	rospy.init_node('laser_read')
	laser_topic = "/sensor_msgs/Laser_scan"
	rospy.Subscriber(laser_topic, LaserScan,LaserData)
	rospy.spin()


if __name__ == '__main__':
	main()
