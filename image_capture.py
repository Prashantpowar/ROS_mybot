#! /usr/bin/python


import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print("e")
    else:
        # Save your OpenCV2 image as a jpeg 
        print("saving")
        time = msg.header.stamp
        cv2.imwrite(''+str(time)+'.jpeg', cv2_img)
        rospy.sleep(1)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/cam/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
