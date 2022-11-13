import json as JSON
from numbers import Number
from socket import gethostbyname
from typing import Any, Tuple, Type

DEFAULT_BUFFER_SIZE = 2 * 1024  # 2 KB per buffer
FRAME_BUFFER_SIZE = 128 * 1024  # 128 KB per buffer
PORT_DATA_SOCKET = 2001
PORT_RTV_SOCKET = 2002


class Signals:
    """ Signals that used by WCU and Robot to control everything in-between"""
    SIGNAL_START_STREAM = 'ss'
    SIGNAL_CLOSE_STREAM = 'cs'
    SIGNAL_SWITCH_CONTROL_MODE = 'scm'
    SIGNAL_DISCONNECT = 'disconnect'


class Directions:
    """ Control commands used to drive the robot """
    CMD_DRIVE_FORWARD = 'df'
    CMD_DRIVE_BACKWARD = 'db'
    CMD_ROTATE_RIGHT = 'rr'
    CMD_ROTATE_LEFT = 'rl'


class ControlModes:
    CONTROL_MODE_MANUAL = 'mcm'
    CONTROL_MODE_AUTOMATIC = 'acm'


class Speeds:
    NO_SPEED = 0
    DEFAULT_SPEED = 60
    ROTATING_SPEED = 80
    MAX_SPEED = 100


class ArduinoSignals:
    SIGNAL_CONTROL_MODE_MANUAL = 0
    SIGNAL_CONTROL_MODE_LFA = 51
    SIGNAL_ROTATE_LEFT = 102
    SIGNAL_DRIVE_FORWARD = 153
    SIGNAL_DRIVE_BACKWARD = 204
    SIGNAL_ROTATE_RIGHT = 255


class Colors:
    TXTCOLOR_RED = '\033[1;31;40m'
    TXTCOLOR_GREEN = '\033[1;32;40m'
    TXTCOLOR_YELLOW = '\033[1;33;40m'
    TXTCOLOR_BLUE = '\033[1;34;40m'
    TXTCOLOR_WHITE = '\033[1;37;40m'


class RobotError(Exception):

    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class DataModel:

    def __init__(self, data: dict[str, Any] | str | None = None) -> None:
        # Parse data if provided in JSON format
        if isinstance(data, str):
            try:
                data = JSON.loads(data)
            except JSON.JSONDecodeError:
                data = dict[str, Any]()
        else:
            # Convert data to dictionary
            self.data = dict[str, Any]() if data is None else data

    @property
    def signal(self) -> str | Any:
        return self.data.get('signal', None)

    @property
    def cmd(self) -> str | Any:
        return self.data.get('cmd', None)

    @property
    def speed(self) -> int | Any:
        return self.data.get('speed', None)

    @property
    def controlMode(self) -> str | Any:
        return self.data.get('ctrl_mode', None)

    @property
    def isStreaming(self) -> bool | Any:
        return self.data.get('streaming', None)

    @property
    def durations(self) -> tuple[int, int, int, int] | None:
        durations: dict[str, int] = self.data.get("durations", None)
        if durations:
            return (durations.get('forward', 10),
                    durations.get('backward', 10),
                    durations.get('right', 10),
                    durations.get('left', 10))
        else:
            return None

    def __str__(self) -> str:
        return 'DataModel[signal=\'{}\', command=\'{}\',speed=\'{}\',ctrlMode=\'{}\',isStreaming=\'{}\']'.format(
            self.signal, self.cmd, self.speed, self.controlMode,
            self.isStreaming)


def model2json(signal: str | None = None,
               control_command: str | None = None,
               speed: int = -1,
               control_mode: str | None = None,
               is_streaming: bool | None = None) -> str:

    try:
        model_data = {}
        if signal:
            model_data['signal'] = signal
        if control_command:
            model_data['cmd'] = control_command
        if speed >= 0 and speed <= 255:
            model_data['speed'] = speed
        if control_mode:
            model_data['ctrl_mode'] = control_mode
        if is_streaming is not None:
            model_data['streaming'] = is_streaming
        # Serialize model to JSON str
        return JSON.dumps(model_data)
    except Exception as e:
        print(f'[model2json]: Error encoding model. Msg=[{e}].')
        return ''


def json2model(json: str) -> 'DataModel':
    try:
        return DataModel(JSON.loads(json))
    except JSON.JSONDecodeError:
        return DataModel()


def get_my_ip() -> str:
    return gethostbyname('localhost')


def constrain(value: Any, min: Any, max: Any):
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value
