#!/usr/bin/env python2

import rosbag

from sensor_msgs.msg import Image

bag = rosbag.Bag('/media/ubi-lab-desktop/Extreme Pro/kinect/data/Person099/B1_BED_OUT/image_raw.bag')

for topic, msg, t in bag.read_messages():
    print(topic)

    print(type(msg))

    print(msg.header)

bag.close()