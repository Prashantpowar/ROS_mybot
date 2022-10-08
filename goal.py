#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt




class Mybot:
	def __init__(self):
		rospy.init_node('goal', anonymous= True)
		self.pose = rospy.Subscriber('/odom', Odometry, self.Robot_pose)
		self.vel  = rospy.Publisher('/cmd_vel',Twist, queue_size= 10)
		self.laser = rospy.Subscriber('/sensor_msgs/Laser_scan', LaserScan, self.LaserData)

		# self.pose = Odometry()
		self.range_center = LaserScan()
		self.rate = rospy.Rate(10)


	def Robot_pose(self, data):
		self.pose = data.pose.pose.position
		self.posew = data.pose.pose.orientation
		self.posew.w = int(self.posew.w)
		self.pose.x= int(self.pose.x)
		self.pose.y = int(self.pose.y)
		self.pose = np.array((self.pose.x,self.pose.y))
		# print("self pose",self.posew.w)


	def LaserData(self, msg):
		# print("here in laser")
		self.range_center = msg.ranges[int(len(msg.ranges)/2)]
		# print(self.range_center)


	def Shortest_distance(self, goal):
		return np.linalg.norm(goal - self.pose)

	def linear_Vel(self,goal, constant = 0.1):
		return constant* self.Shortest_distance(goal)

	def steering_angle(self, goal):
		# return atan2(goal.pose.pose.position.y - self.pose[1], goal.pose.pose.position.x - self.pose[0])
		return atan2(goal[1] - self.pose[1], goal[0] - self.pose[0])


	def angular_vel(self, goal, constant = 6):
		return constant * (self.steering_angle(goal) - self.posew.w)



	def move(self):

		goal = Odometry()
		# print("here under goal",goal)

		x = int(input("x:",))
		y = int(input("y:",))
		# print("goal value",goal)
		goal.pose.pose.position.x=x
		goal.pose.pose.position.y=y
		goal = np.array((goal.pose.pose.position.x,goal.pose.pose.position.y))
		print(goal)

		closness = int(input("nearest dist to goal:",))
		obstacle = 1

		vel = Twist()
		while self.Shortest_distance(goal) >= closness:
			if self.range_center >= obstacle:
				vel.linear.x = self.linear_Vel(goal)
				vel.linear.y = 0
				vel.linear.z =0


				vel.angular.x = 0
				vel.angular.y = 0
				vel.angular.z = self.angular_vel(goal)
				print("linear vel :",vel.linear.x)
				self.vel.publish(vel)
				print("goal dist:",self.Shortest_distance(goal))
				print("range:",self.range_center)
				rospy.sleep(10)
				# self.rate.sleep()

			vel.linear.x =0
			vel.angular.z =0
			self.vel.publish(vel)
			print("vel outside:",self.vel.publish(vel))
			self.rate.sleep()
			break
			rospy.spin()
		vel.linear.x =0
		vel.angular.z =0
		self.vel.publish(vel)
		self.rate.sleep()
		# rospy.spin()


if __name__ == "__main__":
	try:
		x = Mybot()
		x.move()

	except rospy.ROSInterruptException:
		pass