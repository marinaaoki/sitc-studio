#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from sitc_studio.sitc_recorder import SmartHomeStudio


if __name__ == '__main__':
    shs = SmartHomeStudio()
    shs.launch_experiment()