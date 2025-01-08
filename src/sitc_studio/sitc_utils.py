from itertools import chain, cycle, compress

def get_next_participant_id():
    pass

def init_sequence(activities, selectors):
    if selectors is None:
        selectors = [1] * len(activities)
    iterable = compress(activities, selectors)

    return iterable


def init_recording_loop(start_mode):
    states = ['explain', 'record', 'stop']
    if start_mode not in states:
        raise ValueError("Invalid start mode. Must be one of {}".format(states))

    iterable = cycle(states)
    for _ in states[:states.index(start_mode)]:
        iterable.next()
    
    return iterable