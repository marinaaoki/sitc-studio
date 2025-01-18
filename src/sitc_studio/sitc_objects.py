from enum import IntEnum, Enum 

from sensor_msgs.msg import Image, CameraInfo

SENSOR_CONFIG_FILE = "/media/ubi-lab-desktop/Extreme Pro/kinect/sitc_ws/src/sitc_ak_controller/src/sitc_studio/sensor_config.json"
TOPICS_FILE = "/media/ubi-lab-desktop/Extreme Pro/kinect/sitc_ws/src/sitc_ak_controller/src/sitc_studio/sensor_topics.json"
DEFAULT_SAVE = "/media/ubi-lab-desktop/Extreme Pro/kinect/data"

TOPIC_TYPES = {
    "sensor_msgs/Image" : Image,
    "sensor_msgs/CameraInfo" : CameraInfo
}

class ExperimentalState(IntEnum):
    SETUP = 0
    EXPLAIN = 1
    RECORD = 2
    ABORT = 3
    RESUME = 4
    COMPLETE = 5
    ERROR = 99


class Location(Enum):
    def __new__(cls, description):
        value = len(cls.__members__) + 1
        obj = object.__new__(cls)
        obj._value_ = value
        obj.description = description
        return obj 
    
    BEDROOM = 'Bedroom'
    KITCHEN = 'Kitchen'
    LIVING = 'Living'
    DINING = 'Dining'
    BATH = 'Bath'
    SINK = 'Sink'
    ENTRANCE = 'Entrance'

class Activity(Enum):
    def __new__(cls, description, location):
        value = len(cls.__members__) + 1
        obj = object.__new__(cls)
        obj._value_ = value
        obj.description = description
        obj.location = location
        obj.audio_only = True if description == 'VACUUM' or description == 'PC' else False
        obj.synced = True if location != Location.BATH and location != Location.ENTRANCE and location != Location.SINK else False
        return obj 

    __order__ = 'BED_OUT JACKET_ON FRIDGE_OPEN FRIDGE_CLOSE PREPARE_MEAL WATER EAT SIT_DOWN WATCH_TV STAND_UP FALL_DOWN SHOES_ON LEAVE_HOUSE ENTER_HOUSE SHOES_OFF BRUSH_TEETH TAKE_BATH CLEAN_BATH VACUUM PC JACKET_OFF BED_IN'
    BED_OUT = 'BED_OUT', Location.BEDROOM
    JACKET_ON = 'JACKET_ON', Location.BEDROOM
    FRIDGE_OPEN = 'FRIDGE_OPEN', Location.KITCHEN
    FRIDGE_CLOSE = 'FRIDGE_CLOSE', Location.KITCHEN
    PREPARE_MEAL = 'PREPARE_MEAL', Location.KITCHEN
    WATER = 'WATER', Location.DINING
    EAT = 'EAT', Location.DINING
    SIT_DOWN = 'SIT_DOWN', Location.LIVING
    WATCH_TV = 'WATCH_TV', Location.LIVING
    STAND_UP = 'STAND_UP', Location.LIVING
    FALL_DOWN = 'FALL_DOWN', Location.LIVING
    SHOES_ON = 'SHOES_ON', Location.ENTRANCE
    LEAVE_HOUSE = 'LEAVE_HOUSE', Location.ENTRANCE
    ENTER_HOUSE = 'ENTER_HOUSE', Location.ENTRANCE
    SHOES_OFF = 'SHOES_OFF', Location.ENTRANCE
    BRUSH_TEETH = 'BRUSH_TEETH', Location.BATH
    TAKE_BATH = 'TAKE_BATH', Location.BATH
    CLEAN_BATH = 'CLEAN_BATH', Location.BATH
    VACUUM = 'VACUUM', Location.BEDROOM
    PC = 'PC', Location.BEDROOM
    JACKET_OFF = 'JACKET_OFF', Location.BEDROOM
    BED_IN = 'BED_IN', Location.BEDROOM

    def __str__(self):
        return self.description





