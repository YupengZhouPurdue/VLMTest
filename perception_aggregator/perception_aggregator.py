#!/usr/bin/env python
import rospy
from novatel_oem7_msgs.msg import BESTGNSSPOS
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

last_save_time = 0
SAVE_INTERVAL = 0.1  # seconds (10 Hz)

def image_callback(msg):
    global last_save_time
    now = time.time()
    if now - last_save_time < SAVE_INTERVAL:
        return  # Skip saving if less than 0.1s since last save
    last_save_time = now
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        save_path = 'VLM_test_platform/perception_aggregator'
        cv2.imwrite(save_path, cv_image)
        rospy.loginfo(f"Saved image to {save_path}")
    except Exception as e:
        rospy.logerr(f"Failed to save image: {e}")

def main():
    rospy.init_node('perception_aggregator', anonymous=True)
    rospy.Subscriber('camera_lf/image_raw', Image, image_callback)
    rospy.loginfo("Subscribed to camera_lf/image_raw. Waiting for images...")
    rospy.spin()

if __name__ == '__main__':
    main()
