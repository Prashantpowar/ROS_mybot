#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan





def Robot_pose(msg):
	my_position = msg.pose.pose.position
	Shortest_distance(my_position)




def Shortest_distance(Current_Position):
	x = int(Current_Position.x)
	y = int(Current_Position.y)
	z = int(Current_Position.z)
	arr = np.array([x,y])
	desired_distance = np.array([3,3])
	dist = np.linalg.norm(desired_distance - arr)
	print("my dist",dist)
	move(dist,desired_distance)

	
	
def LaserData(data):
	range_center = data.ranges[int(len(data.ranges)/2)]
	print(range_center)

def move(dist, desired_distance):
	vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	vel = Twist()
	disto = 5




	while dist <= disto:
		vel.linear.x = 1
		vel.linear.y = 0
		vel.linear.z = 0

		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = 0
		print("here i am",dist)

		vel_publisher.publish(vel)
		# rospy.rate.sleep()
	vel.linear.x =0
	vel.angular.z =0
	rospy.spin()




if __name__ == '__main__':
	rospy.init_node('practice', anonymous=True)
	odo = "/odom"
	rospy.Subscriber(odo, Odometry, Robot_pose)
	laser_topic = "/sensor_msgs/Laser_scan"
	rospy.Subscriber(laser_topic, LaserScan,LaserData)
	rospy.spin()
	

 
 
