#!/usr/bin/env python2
import cv2
import rosbag

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

person_id = 9
activity = "D1_EAT"

bag = rosbag.Bag('/media/ubi-lab-desktop/disk6s2/sitc_data/Person00' + str(person_id) + '/' + activity + '/image_raw.bag')


for i, (topic, msg, stamp) in enumerate(bag.read_messages(topics=["depth/image_raw"])):
    cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    # normalise and then rescale to greyscale
    cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
    cv_image = cv2.convertScaleAbs(cv_image)
    cv2.imshow("Depth Image", cv_image)
    # show at 30 fps
    cv2.waitKey(1000/30)

    
    
bag.close()
    
