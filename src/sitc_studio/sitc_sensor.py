from sitc_studio.sitc_objects import Location

import os
import json
import rospy
import rosbag
import threading

from std_msgs.msg import String

class Kinect(object):
    SENSOR_CONFIG_FILE = "/media/ubi-lab-desktop/Extreme Pro/kinect/sitc_ws/src/sitc_ak_controller/src/sitc_studio/sensor_config.json"

    def __init__(self, participant_id, sensor_id, sensor_sn, activity, location, save_loc, synced=False):
        self.participant_id = participant_id
        self.sensor_id = sensor_id
        self.sensor_sn = sensor_sn
        
        self.activity = activity
        self.location = location
        self.save_loc = save_loc
        self.bag_name = "%s/Person%03d/%s_%s.bag" % (self.save_loc, self.participant_id, self.sensor_id, self.activity.description)
        self.synced = synced

        if self.synced:
            self.sub_sensor = KinectSub.from_master(self)

        self.is_recording = False
        self.lock = threading.Lock()

        # TODO: need to change this since we are only collecting raw data
        self.topic = "/{}/body_tracking_data".format(sensor_id)
        self.subscriber = None
        self.bag = None

    
    @classmethod
    def from_activity(cls, participant_id, activity, save_loc, synced=False):
        location = activity.location

        if location != Location.BATH and location != Location.ENTRANCE and location != Location.SINK:
            synced = True

        with open(cls.SENSOR_CONFIG_FILE) as f:
            sensor_config = json.load(f)
        master_device = filter(lambda x: x['location'] == location.description and x['master'], sensor_config["sensors"])[0]

        sensor_id = master_device['sensor_id']
        sensor_sn = master_device['sensor_sn']

        return cls(participant_id, sensor_id, sensor_sn, activity, location, save_loc, synced)

    
    def start(self):
        """Launch the ROS subscriber for this Kinect sensor topic"""
        if not os.path.exists("%s/Person%03d" % (self.save_loc, self.participant_id)):
            os.makedirs("%s/Person%03d" % (self.save_loc, self.participant_id))

        self.bag = rosbag.Bag(self.bag_name, 'w')
        self.is_recording = True
        
        # TODO: adapt the data type according to the format we need
        self.subscriber = rospy.Subscriber(self.topic, String, self.callback)
        
        if self.synced:
            self.sub_sensor.is_recording = True
            self.sub_sensor.bag = rosbag.Bag(self.sub_sensor.bag_name, 'w')
            self.sub_sensor.subscriber = rospy.Subscriber(self.sub_sensor.topic, String, self.sub_sensor.callback)

        while True:
            key = raw_input("Press 'q' to stop recording: ")
            if key.lower() == 'q':
                break
        
        self.stop()
        
        
    def callback(self, data):
        """Callback function for the ROS subscriber. Store the data in a rosbag"""
        if self.is_recording and self.bag:
            with self.lock:
                self.bag.write(self.topic, data)


    def stop(self):
        """Stop the ROS subscriber"""
        self.is_recording = False
        self.subscriber.unregister()
        self.subscriber = None

        if self.synced:
            self.sub_sensor.stop()
            self.sub_sensor = None
            
        self.bag.close()
        self.bag = None
        print("SENSOR {}: Recording for {} saved to {}".format(self.sensor_id, self.activity.description, self.bag_name))

class KinectSub(Kinect):
    def __init__(self, participant_id, master_id, sensor_id, sensor_sn, activity, location, save_loc, synced=False):
        super(KinectSub, self).__init__(participant_id, sensor_id, sensor_sn, activity, location, save_loc, synced)
        self.master_id = master_id


    @classmethod
    def from_master(cls, master_kinect):
        activity = master_kinect.activity
        location = master_kinect.location
        
        sensor_config = json.load(open(cls.SENSOR_CONFIG_FILE))
        sub_device = filter(lambda x: x['location'] == location.description and not x['master'], sensor_config["sensors"])[0]

        sensor_id = sub_device['sensor_id']
        sensor_sn = sub_device['sensor_sn']

        return cls(participant_id=master_kinect.participant_id, master_id=master_kinect.sensor_id, sensor_id=sensor_id, sensor_sn=sensor_sn, activity=activity, location=location, save_loc=master_kinect.save_loc, synced=False)

        
