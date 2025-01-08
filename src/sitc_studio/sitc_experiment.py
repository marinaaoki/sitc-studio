from sitc_studio.sitc_objects import Activity, Location, ExperimentalState
from sitc_sensor import Kinect
from sitc_utils import init_sequence, init_recording_loop

import time
import rospy

class Progress:
    def __init__(self, participant_id, save_loc=None):
        self.participant_id = participant_id
        self.save_loc = save_loc

        self.current_activity = None
        self.current_sensor = None

        self.recording_loop = init_recording_loop('explain')
        self.current_step = None

    def save(self):
        pass

    def load(self):
        pass

    def next(self, activity):
        """Progress to the next state and perform required tasks. Returns the new state."""
        self.current_activity = activity
        self.current_sensor = Kinect.from_activity(self.participant_id, activity, self.save_loc)
        self.current_step = self.recording_loop.next()

        if self.current_step == 'explain':
            self.explain()
        elif self.current_step == 'record':
            return self.record()
    
    def explain(self):
        """Idling while the participant receives and explanation of the next activity"""
        inp = raw_input("The next activity is {}. Explain the activity to the participant and answer any questions.\nType 'a' to abort, or press any other key to continue...".format(self.current_activity.name))

        if inp == 'a':
            return ExperimentalState.ABORT

        self.next(self.current_activity)

    def record(self):
        """Record the current state"""
        print("ACTIVITY: {}".format(self.current_activity.name))
        print("LOCATION: {}".format(self.current_activity.location))

        print("SENSOR: {}".format(self.current_sensor.sensor_id))
        self.current_sensor.start()

        # stop recording on CTRL^C
        self.current_sensor.stop()
        print("Recording stopped.")

        raw_input("Press any key to continue...")
        return ExperimentalState.RECORD

    
class Configuration:
    DEFAULT_SAVE = "/media/ubi-lab-desktop/Extreme Pro/kinect/data"

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
        start_time = None
        save_loc = None
        progress = None
        debug = None
        return cls(participant_id, ExperimentalState.RESUME, start_time, progress, save_loc, debug)
    
    @classmethod
    def from_complete(cls, participant_id, save_loc=DEFAULT_SAVE):
        # for visualisation purposes
        # TODO: load progress from save_loc
        start_time = None
        end_time = None
        save_loc = None
        progress = None
        debug = None
        return cls(participant_id, ExperimentalState.COMPLETE, start_time, end_time, progress, save_loc, debug)

    def update(self, state, progress):
        self.state = state
        self.progress = progress
        if state == ExperimentalState.COMPLETE and not self.debug:
            self.end_time = time.time()
            self.save()
        elif state == ExperimentalState.ABORT and not self.debug:
            self.save()

    
    def save(self):
        # TODO: save progress to experiment_config.json
        # this should include the participant id, last reached activity, and the current state along with the progress in the recording loop if the state is RECORD
        # this is a csv file that is updated using pandas as a database of all experiments
        pass

    def load(self):
        pass


class Experiment:
    def __init__(self, configuration, activity_sequence=None):
        self.configuration = configuration
        self.activity_sequence = activity_sequence

        self.state = configuration.state
        self.progress = configuration.progress

        rospy.init_node('sitc_experiment', anonymous=True)
    
    @classmethod
    def from_participant_id(cls, participant_id):
        # TODO: check if participant id exists. if yet, then we are resuming an experiment
        # if not, we are starting a new experiment
        return cls(Configuration.from_start(participant_id), init_sequence([a for a in Activity], None))
    
    def save(self):
        self.configuration.save()
    
    def load(self):
        pass
    
    def delete(self):
        pass

    def next(self):
        next_activity = self.activity_sequence.next()

        if next_activity is None:
            self.state = ExperimentalState.COMPLETE
            self.configuration.update(self.state)
            return

        self.state = self.progress.next(next_activity)
        self.configuration.update(self.state, self.progress)

        return
        



