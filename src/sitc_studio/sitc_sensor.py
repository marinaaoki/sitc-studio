from sitc_studio.sitc_objects import Location, SENSOR_CONFIG_FILE, TOPICS_FILE, TOPIC_TYPES

import os
import json
import rospy
import rosbag
import threading

class Kinect(object):

    def __init__(self, participant_id, sensor_id, sensor_sn, activity, location, save_loc, synced=False):
        self.participant_id = participant_id
        self.sensor_id = sensor_id
        self.sensor_sn = sensor_sn
        
        self.activity = activity
        self.location = location
        self.save_loc = save_loc
        self.bag_name_app = "%s/Person%03d/%s_%s" % (self.save_loc, self.participant_id, self.sensor_id, self.activity.description)
        self.synced = synced

        if self.synced:
            self.sub_sensor = KinectSub.from_master(self)

        self.is_recording = False
        self.lock = threading.Lock()

        with open(TOPICS_FILE) as f:
            topic_config = json.load(f)

        self.topics = {topic["topic_name"]: topic["topic_type"] for topic in topic_config["topics"]}
        self.subscribers = []
        self.bags = []

    
    @classmethod
    def from_activity(cls, participant_id, activity, save_loc, synced=False):
        location = activity.location

        if location != Location.BATH and location != Location.ENTRANCE and location != Location.SINK:
            synced = True

        with open(SENSOR_CONFIG_FILE) as f:
            sensor_config = json.load(f)
        master_device = filter(lambda x: x['location'] == location.description and x['master'], sensor_config["sensors"])[0]

        sensor_id = master_device['sensor_id']
        sensor_sn = master_device['sensor_sn']

        return cls(participant_id, sensor_id, sensor_sn, activity, location, save_loc, synced)

    
    def start(self):
        """Launch the ROS subscriber for this Kinect sensor topic"""
        if self.activity.audio_only:
            return
        
        if not os.path.exists("%s" % self.bag_name_app):
            os.makedirs("%s" % self.bag_name_app)

        self.is_recording = True
        
        for topic, topic_type in self.topics.items():
            topic_name = topic.split('/')[-1]
            topic_type = TOPIC_TYPES[topic_type]
            bag = rosbag.Bag("%s/%s.bag" % (self.bag_name_app, topic_name), 'w')
            self.bags.append(bag)
            sensor_topic = "/%s/%s" % (self.sensor_id.lower(), topic)
            subscriber = rospy.Subscriber(sensor_topic, topic_type, self.callback, callback_args=topic)
            self.subscribers.append(subscriber)
        
        if self.synced:
            self.sub_sensor.start()
        
        
    def callback(self, data, topic):
        """Callback function for the ROS subscriber. Store the data in a rosbag"""
        bag = self.bags[self.topics.keys().index(topic)]

        if self.is_recording and bag:
            with self.lock:
                bag.write(topic, data)


    def stop(self):
        """Stop the ROS subscriber"""
        self.is_recording = False

        if self.activity.audio_only:
            return

        for subscriber in self.subscribers:
            subscriber.unregister()
        self.subscribers = []

        for bag in self.bags:
            try:
                bag.close()
            except:
                print("Error closing bag.")
        self.bags = []

        if self.synced:
            self.sub_sensor.stop()

        print("SENSOR {}: Recording for {} saved to {}".format(self.sensor_id, self.activity.description, self.bag_name_app))


class KinectSub(Kinect):
    def __init__(self, participant_id, master_id, sensor_id, sensor_sn, activity, location, save_loc, synced=False):
        super(KinectSub, self).__init__(participant_id, sensor_id, sensor_sn, activity, location, save_loc, synced)
        self.master_id = master_id


    @classmethod
    def from_master(cls, master_kinect):
        activity = master_kinect.activity
        location = master_kinect.location
        
        sensor_config = json.load(open(SENSOR_CONFIG_FILE))
        sub_device = filter(lambda x: x['location'] == location.description and not x['master'], sensor_config["sensors"])[0]

        sensor_id = sub_device['sensor_id']
        sensor_sn = sub_device['sensor_sn']

        return cls(participant_id=master_kinect.participant_id, master_id=master_kinect.sensor_id, sensor_id=sensor_id, sensor_sn=sensor_sn, activity=activity, location=location, save_loc=master_kinect.save_loc, synced=False)

        
