from enum import IntEnum
from robot_utils import ControlModes


UNIT_B = 'B'
UNIT_KB = 'KB'
UNIT_MB = 'MB'
TARGET_FRAME_SIZE = (900, 600)


class ControlCommandsTexts:
    """ Commands text to be displayed in the GUI """
    TEXT_IDLE = 'Waiting for commands...'
    TEXT_ACCELERATING = 'Accelerating...'
    TEXT_DECELERATING = 'Decelrating...'
    TEXT_DRIVING_FORWARD = 'Driving Forward...'
    TEXT_DRIVING_BACKWARD = 'Driving Backward...'
    TEXT_TURNING_RIGHT = 'Turning Right...'
    TEXT_TURNING_LEFT = 'Turning Left...'
    TEXT_ACTIVATING_BRAKES = 'Activating Brakes...'
    TEXT_NOT_CONNECTED = 'Not connected'
    TEXT_CONNECTING = 'Connecting...'
    TEXT_CONNECTED = 'Connected'
    TEXT_REQUESTING_STREAM = 'Realtime Video Stream requested'
    TEXT_CONTROL_MODE_AUTO = 'Changed to Automatic Control'
    TEXT_CONTROL_MODE_MANUAL = 'Changed to Manual Control'
    TEXT_REACHED_MAX_SPEED = 'Reached Maximum Speed'
    TEXT_REACHED_MIN_SPEED = 'Reached Minimum Speed'
    TEXT_REACHED_HARD_RIGHT = 'Reached Hard Right'
    TEXT_REACHED_HARD_LEFT = 'Reached Hard Left'
    TEXT_LFA_MODE_ENABLED = 'Automatic LF Control Is Enabled'
    TEXT_SWITCH_TO_LFA_MODE = 'Switch to LFA control mode'
    TEXT_SWITCH_TO_MANUAL_MODE = 'Switch to Manual control mode'
    TEXT_START_STREAM = 'Start Robot Stream'
    TEXT_STOP_STREAM = 'Stop Robot Stream'
    TEXT_START_RECORDING_STREAM = 'Record Stream'
    TEXT_STOP_RECORDING_STREAM = 'Stop Recording'
    TEXT_REQUESTING_STREAM = 'Requesting Robot Stream...'
    TEXT_FAILED_TO_CONNECT = 'Failed to connect to Robot'


class Signals:
    """ Signals to use between the robot and WCU """
    SIGNAL_HELLO = 'HELLO'
    SIGNAL_BYE = 'BYE'
    SIGNAL_OK = 'OK'
    SIGNAL_READY = 'READY'
    SIGNAL_ACCELERATE = 'ACCELERATE'
    SIGNAL_DECELERATE = 'DECELERATE'
    SIGNAL_TURN_RIGHT = 'TURN_RIGHT'
    SIGNAL_TURN_LEFT = 'TURN_LEFT'
    SIGNAL_START_STREAM = 'START_STREAM'
    SIGNAL_END_STREAM = 'END_STREAM'
    SIGNAL_SWITCH_CONTROL = 'SWITCH_CONTROL'


class Colors:
    TXTCOLOR_RED = '\033[1;31;40m'
    TXTCOLOR_GREEN = '\033[1;32;40m'
    TXTCOLOR_YELLOW = '\033[1;33;40m'
    TXTCOLOR_BLUE = '\033[1;34;40m'
    TXTCOLOR_WHITE = '\033[1;37;40m'


class StatusCode(IntEnum):
    STATUS_IDLE: int
    STATUS_LOADING: int
    STATUS_NO_LOAD: int
    STATUS_STREAMING: int
    STATUS_NO_STREAM: int


class Status:
    NOT_CONNECTED = -1
    CONNECTING = 0
    CONNECTED = 1


def cvt_size(how_many=0, unit=UNIT_KB) -> int:
    if unit == UNIT_KB:
        return how_many * 1024
    else:  # MBs
        return how_many * 1024 * 1024


def get_size(how_many_bytes=0) -> str:
    if how_many_bytes < 1024 * 1024:
        return f'{how_many_bytes / 1024:.2f} {UNIT_KB}'
    elif how_many_bytes < 1024 * 1024 * 1024:
        return f'{how_many_bytes / (1024 * 1024) :.2f} {UNIT_MB}'
    else:
        return f'{how_many_bytes} {UNIT_B}'


def get_switched_ctrl_mode(ctrl_mode: str):
    return ControlModes.CONTROL_MODE_MANUAL if ctrl_mode == ControlModes.CONTROL_MODE_AUTOMATIC else ControlModes.CONTROL_MODE_AUTOMATIC
