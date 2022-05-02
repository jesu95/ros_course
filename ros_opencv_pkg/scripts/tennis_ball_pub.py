#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
video_capture = cv2.VideoCapture('../media/tennis-ball-video.mp4')

def video_pub():
  
  #initialize the node
  rospy.init_node('tennis_ball_pub', anonymous=True)
  #create a new publisher.
  pub = rospy.Publisher('tennis_ball_image', Image, queue_size=5)
  #set the loop rate
  rate = rospy.Rate(10) # hz
  
  global bridge
  
  while not rospy.is_shutdown():
    ret, cv_frame = video_capture.read()

    #convert ros_image into an opencv-compatible image
    try:
      ros_frame = bridge.cv2_to_imgmsg(cv_frame, "bgr8")
    except CvBridgeError as e:
      print(e)
      #from now on, you can work exactly like with ros
  
    pub.publish(ros_frame)
    print('Sended an image')
    rate.sleep()
  video_capture.release()

if __name__ == '__main__':
  try:
    video_pub()
  except rospy.ROSInterruptException:
    pass