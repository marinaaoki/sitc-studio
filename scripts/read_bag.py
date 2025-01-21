#!/usr/bin/env python2
import cv2
import rosbag

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bag = rosbag.Bag('/media/ubi-lab-desktop/Extreme Pro/kinect/data/Person001/L2_FALL_DOWN/image_raw.bag')


for topic, msg, stamp in bag.read_messages(topics=["depth/image_raw"]):
    cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    # normalise and then rescale to greyscale
    cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
    cv_image = cv2.convertScaleAbs(cv_image)
    cv2.imshow("Depth Image", cv_image)
    cv2.waitKey(1)

bag.close()
    
