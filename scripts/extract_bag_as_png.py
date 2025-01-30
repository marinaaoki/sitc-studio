#!/usr/bin/env python2
import cv2
import os
import rosbag

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

kitchen_sensor = 1
person_id = 11

activities = [
    "B1_BED_OUT",
    "B2_BED_OUT",
    "B1_JACKET_ON",
    "B2_JACKET_ON",
    "_FRIDGE_OPEN",
    "_FRIDGE_CLOSE",
    "_PREPARE_MEAL",
    "D1_WATER",
    "D2_WATER",
    "D1_EAT",
    "D2_EAT",
    "L1_SIT_DOWN",
    "L2_SIT_DOWN",
    "L1_WATCH_TV",
    "L2_WATCH_TV",
    "L1_STAND_UP",
    "L2_STAND_UP",
    "L1_FALL_DOWN",
    "L2_FALL_DOWN",
    "E1_SHOES_ON",
    "E1_LEAVE_HOUSE",
    "E1_ENTER_HOUSE",
    "E1_SHOES_OFF",
    "B1_JACKET_OFF",
    "B2_JACKET_OFF",
    "B1_BED_IN",
    "B2_BED_IN",
]

for activity in activities:
    if activity.startswith("_"):
        activity = "K" + str(kitchen_sensor) + activity
    print("Activity: " + activity)

    bag = rosbag.Bag('/media/ubi-lab-desktop/disk6s2/sitc_data/Person0' + str(person_id) + '/' + activity + '/image_raw.bag')

    for i, (topic, msg, stamp) in enumerate(bag.read_messages(topics=["depth/image_raw"])):
        cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # normalise and then rescale to greyscale
        cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        cv_image = cv2.convertScaleAbs(cv_image)
        cv2.imshow("Depth Image", cv_image)
        # show at 60 fps
        cv2.waitKey(1000/60)
        # save as png
        if not os.path.exists('/media/ubi-lab-desktop/disk6s2/sitc_data/Person0' + str(person_id) + '/' + activity + '/depthImages/'):
            os.makedirs('/media/ubi-lab-desktop/disk6s2/sitc_data/Person0' + str(person_id) + '/' + activity + '/depthImages/')
        cv2.imwrite('/media/ubi-lab-desktop/disk6s2/sitc_data/Person0' + str(person_id) + '/' + activity + '/depthImages/' + str(i) + '.png', cv_image)
    print("Total frames: " + str(i))

    bag.close()
    raw_input("Press Enter to continue...")
    
