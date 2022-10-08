#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



def LaserData(msg):
	print("//////////////////////////////Laser Data//////////////////////////////////")
	range_center = msg.ranges[int(len(msg.ranges)/2)]
	print("range_center",range_center)
	move(range_center)




def move(range_center):
	velocity_publisher =rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	my_range = int(range_center)
	print('my_range',my_range)
	vel_msg = Twist()
	rate = rospy.Rate(10)
	while my_range !=1:
		print('vel=======',vel_msg)
		print(my_range)
		vel_msg.linear.x =1
		vel_msg.linear.y =0
		vel_msg.linear.z =0
		velocity_publisher.publish(vel_msg)
	vel_msg.linear.x =0
	vel_msg.angular.z =0
	velocity_publisher.publish(vel_msg)
	rospy.spin()
	rate.sleep(5)









if __name__=='__main__':
	rospy.init_node('obstacle_avoidance', anonymous=True)
	laser_topic = "/sensor_msgs/Laser_scan"
	rospy.Subscriber(laser_topic, LaserScan, LaserData)
	rospy.spin()
	# range_center = laser_range
	# move(laser_range)



        

        