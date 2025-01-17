from sitc_studio.sitc_experiment import Experiment, Configuration
from sitc_studio.sitc_objects import Activity, Location, ExperimentalState
from sitc_utils import get_next_participant_id

import rospy

class SmartHomeStudio:

    def __init__(self):
        self.experiment = None

    def launch_experiment(self):
        """Entry point for all experiments"""
        
        print("Welcome to the Smart Home Studio!")
        print("What would you like to do?")
        print("1. Start a new experiment")
        print("2. Load an existing experiment")
        print("3. Delete an existing experiment")
        print("4. Exit")

        choice = raw_input("")

        if choice == "1":
            self.start_experiment()
        elif choice == "2":
            self.load_experiment()
        elif choice == "3":
            self.delete_experiment()
        elif choice == "4":
            return
        else:
            print("Invalid choice. Please try again.")
            self.launch_experiment()

    def start_experiment(self):
        """Start an experiment with a new participant"""
        save_loc = None

        debug = raw_input("Enter debug mode (T/F): ")

        if debug is "T":
            debug = True
            participant_id = 99
        else:
            debug = False
            participant_id = get_next_participant_id()
            change = raw_input("Auto-generated participant ID: {}. Do you want to change it? (Y/N)".format(participant_id))
            if change == "Y":
                participant_id = input("Enter the participant ID: ")

        self.experiment = Experiment.from_participant_id(participant_id)
        self.experiment.save()

        print("Experiment started for participant {}".format(participant_id))
        print("Press any key to begin...")
        raw_input()

        self.recording_loop()

    def save_experiment(self):
        """Save the current experiment"""
        self.experiment.save()

    def load_experiment(self):
        """Load an experiment"""
        participant_id = input("Which participant ID would you like to load? ")
        self.experiment = Experiment.from_participant_id(participant_id)
        self.experiment.load()

        print("Experiment loaded for participant {}. Resuming recording...".format(participant_id))

        self.recording_loop()

    def delete_experiment(self):
        """Delete an experiment"""
        participant_id = input("Which participant ID would you like to delete? ")
        self.experiment = Experiment.from_participant_id(participant_id)
        self.experiment.delete()

    def recording_loop(self):
        """Main loop for the recording process"""
        while not rospy.is_shutdown():
            ret = self.experiment.next()
            if ret == ExperimentalState.ABORT:
                print("Experiment aborted.")
                break
            elif ret == ExperimentalState.COMPLETE:
                print("Experiment finished.")
                break
        
