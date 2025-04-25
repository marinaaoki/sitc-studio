# SITC Studio

This package provides functionalities to start, stop, and save recordings in the smart home. Before starting a recording, ensure that the Kinect camera drivers are up and running. Run the script located in `launch_experiment.py` to start a new experiment.

## Provided functionalities:

- `launch_experiment.py`: start, stop, and save recordings
- `crop_sequence.py`: crop the recorded sequence by specifying start and end frame indices
- `extract_bag_as_png.py`: extract the depth frames from the saved ROS bag files, and save them in the specified location
- `read_bag.py`: read images from the ROS bag file and visualise them using OpenCV