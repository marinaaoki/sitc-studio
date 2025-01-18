import os

from itertools import cycle, compress
from sitc_objects import DEFAULT_SAVE

def get_next_participant_id(save_loc=DEFAULT_SAVE):
    participant_id = 1
    while True:
        if not os.path.exists("%s/Person%03d" % (save_loc, participant_id)):
            return participant_id
        participant_id += 1

def init_sequence(activities, selectors):
    if selectors is None:
        selectors = [1] * len(activities)
    iterable = compress(activities, selectors)

    return iterable


def init_recording_loop(start_mode):
    states = ['explain', 'record']
    if start_mode not in states:
        raise ValueError("Invalid start mode. Must be one of {}".format(states))

    iterable = cycle(states)
    for _ in states[:states.index(start_mode)]:
        iterable.next()
    
    return iterable