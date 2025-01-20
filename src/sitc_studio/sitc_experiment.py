from sitc_studio.sitc_objects import Activity, Location, ExperimentalState, DEFAULT_SAVE
from sitc_sensor import Kinect
from sitc_utils import init_sequence, init_recording_loop

import os
import time
import rospy
import pandas as pd

class Progress:
    def __init__(self, participant_id, save_loc=None):
        self.participant_id = participant_id
        self.save_loc = save_loc

        self.current_activity = None
        self.current_sensor = None

        self.recording_loop = init_recording_loop('explain')
        self.current_step = None


    def next(self, activity):
        """Progress to the next state and perform required tasks. Returns the new state."""
        self.current_activity = activity
        self.current_sensor = Kinect.from_activity(self.participant_id, activity, self.save_loc)
        self.current_step = self.recording_loop.next()
        
        if self.current_step == 'explain':
            ret = self.explain()
            if ret == ExperimentalState.EXPLAIN:
                self.record()
            else:
                return ret
        elif self.current_step == 'record':
            return self.record()
    
    def explain(self):
        """Idling while the participant receives and explanation of the next activity"""
        print("==========\nACTIVITY: {}".format(self.current_activity.name))
        print("LOCATION: {}".format(self.current_activity.location))
        print("SENSOR: {}".format(self.current_sensor.sensor_id))
        if self.current_sensor.synced:
            print("SENSOR: {}".format(self.current_sensor.sub_sensor.sensor_id))
        print("==========")

        print("The next activity is {}. Explain the activity to the participant and answer any questions.".format(self.current_activity.name))

        if self.current_activity.audio_only:
            print("This activity is AUDIO ONLY. No video data will be saved.")

        inp = raw_input("Type 'a' to abort, or press any other key to start recording...".format(self.current_activity.name))

        if inp == 'a':
            return ExperimentalState.ABORT
        
        return ExperimentalState.EXPLAIN

    def record(self):
        """Record the current state"""
        self.current_sensor.start()

        while True:
            key = raw_input("Type 'q' to stop recording: ")
            if key.lower() == 'q':
                break

        self.current_sensor.stop()

        raw_input("Press any key to continue...")
        return ExperimentalState.RECORD
    
    def __str__(self):
        return str(self.current_activity)

    
class Configuration:
    def __init__(self, participant_id, state, start_time=None, end_time=None, progress=None, save_loc=DEFAULT_SAVE, debug=False):
        """Configuration is used to save and load the progress of experiments that may have been interrupted."""
        self.activities = [a for a in Activity]
        self.selected_activities = [1] * len(self.activities)
        self.locations = [l for l in Location]

        self.participant_id = participant_id
        self.start_time = start_time
        self.end_time = end_time
        self.state = state
        self.progress = progress
        self.save_loc = save_loc
        self.debug = debug

    @classmethod
    def from_start(cls, participant_id, save_loc=DEFAULT_SAVE, debug=False):
        start_time = time.time()
        return cls(participant_id, ExperimentalState.SETUP, start_time=start_time, progress=Progress(participant_id, save_loc), save_loc=save_loc, debug=debug)
    
    @classmethod
    def from_resume(cls, participant_id, save_loc=DEFAULT_SAVE):
        # TODO: load progress from save_loc
        # start_time = None
        # save_loc = None
        # progress = None
        # debug = None
        # return cls(participant_id, ExperimentalState.RESUME, start_time, progress, save_loc, debug)
        config = cls(participant_id, ExperimentalState.RESUME, save_loc=save_loc)
        # configure time in load method
        config.load() 
        return config
    
    @classmethod
    def from_complete(cls, participant_id, save_loc=DEFAULT_SAVE):
        config_path = "{}/experiment_config.csv".format(save_loc)

        if not os.path.exists(config_path):
            print("Configuration file not found at {}".format(config_path))

        df = pd.read_csv(config_path)
        row = df[df['participant_id'] == participant_id]

        if row.empty:
            print("No saved progress found for participant {}".format(participant_id))

        row = row.iloc[0]
        start_time = row['start_time'] if not pd.isna(row['start_time']) else None
        end_time = row['end_time'] if not pd.isna(row['end_time']) else None
        progress = row['latest_activity'] 
        debug = True  

        return cls(participant_id, ExperimentalState.COMPLETE, start_time, end_time, progress, save_loc, debug)
        # for visualisation purposes
        # TODO: load progress from save_loc
        # start_time = None
        # end_time = None
        # save_loc = None
        # progress = None
        # debug = None
        # return cls(participant_id, ExperimentalState.COMPLETE, start_time, end_time, progress, save_loc, debug)


    def update(self, state, progress):
        self.state = state
        self.progress = progress
        if state == ExperimentalState.COMPLETE and not self.debug:
            self.end_time = time.time()
        
        if not self.debug:
            self.save()

    
    def save(self):
        if not os.path.exists("%s/experiment_config.csv" % self.save_loc):
            df = pd.DataFrame(columns=['participant_id', 'start_time', 'end_time', 'latest_state', 'latest_activity'])
            df.to_csv("%s/experiment_config.csv" % self.save_loc, index=False)

        df = pd.read_csv("%s/experiment_config.csv" % self.save_loc)
        row = {'participant_id': self.participant_id, 'start_time': self.start_time, 'end_time': self.end_time, 'latest_state': int(self.state), 'latest_activity': self.progress}
        # overwrite the row for this participant if it exists otherwise append
        df = df[df['participant_id'] != self.participant_id]
        df = df.append(row, ignore_index=True)
        df.to_csv("%s/experiment_config.csv" % self.save_loc, index=False)
        print("Experiment config saved for participant {}".format(self.participant_id))

    def load(self):
        config_path = "{}/experiment_config.csv".format(self.save_loc)
        if not os.path.exists(config_path):
            print("Configuration file not found at {}".format(config_path))
            return None
        try:
            df = pd.read_csv(config_path)
            row = df[df['participant_id'] == self.participant_id]
            for row in df:
                print(row)

            if row.empty:
                print("No saved progress found for participant {}".format(self.participant_id))

            row = row.iloc[0]
            self.start_time = row['start_time'] if not pd.isna(row['start_time']) else None
            self.end_time = row['end_time'] if not pd.isna(row['end_time']) else None
            self.state = ExperimentalState(int(row['latest_state']))
            self.progress = row['latest_activity'] 
            print(row)
            print("Loaded configuration for participant {}".format(self.participant_id))
            return row
        except Exception as e:
            print("Error loading configuration: {}".format(e))
            return None

    def delete(self):
        config_path = "{}/experiment_config.csv".format(self.save_loc)
        try:
            if os.path.exists(config_path):
                df = pd.read_csv(config_path)
                df = df[df['participant_id'] != self.participant_id]
                df.to_csv(config_path, index=False)
                print("Deleted configuration for participant {}".format(self.participant_id))
        except Exception as e:
            print("Error deleting configuration: {}".format(e))
            return None


class Experiment:
    def __init__(self, configuration, activity_sequence=None):
        self.configuration = configuration
        self.activity_sequence = activity_sequence

        self.state = configuration.state
        self.progress = configuration.progress

        rospy.init_node('sitc_experiment', anonymous=True)
    
    @classmethod
    def from_participant_id(cls, participant_id):
        if not os.path.exists("%s/Person%03d" % (DEFAULT_SAVE, participant_id)):
            return cls(Configuration.from_start(participant_id), init_sequence([a for a in Activity], None))
        else:
            return cls(Configuration.from_resume(participant_id), init_sequence([a for a in Activity], None))
    
    def save(self):
        self.configuration.save()
    
    def load(self):
        self.configuration.load()
    
    def delete(self):
        dirs = os.listdir("%s/Person%03d" % (self.configuration.save_loc, self.configuration.participant_id))
        for d in dirs:
            files = os.listdir("%s/Person%03d/%s" % (self.configuration.save_loc, self.configuration.participant_id, d))
            for f in files:
                os.remove("%s/Person%03d/%s/%s" % (self.configuration.save_loc, self.configuration.participant_id, d, f))
            os.rmdir("%s/Person%03d/%s" % (self.configuration.save_loc, self.configuration.participant_id, d))

        os.rmdir("%s/Person%03d" % (self.configuration.save_loc, self.configuration.participant_id))

        self.configuration.delete()

    def next(self):
        try:
            next_activity = self.activity_sequence.next()
        except StopIteration:
            next_activity = None

        if next_activity is None:
            self.state = ExperimentalState.COMPLETE
            self.configuration.update(self.state, self.progress)
            return self.state

        self.state = self.progress.next(next_activity)
        if self.state == None:
            self.state = ExperimentalState.EXPLAIN
        self.configuration.update(self.state, self.progress)

        return self.state
        



