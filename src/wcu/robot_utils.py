import json as JSON
from numbers import Number
from socket import gethostbyname
from typing import Any


DEFAULT_BUFFER_SIZE = 2 * 1024  # 2 KB per buffer
STREAM_BUFFER_SIZE = 1024 * 1024  # 1 MB per buffer
PORT_DATA_SOCKET = 2001
PORT_RTV_SOCKET = 2002


class Signals:
    """ Signals that used by WCU and Robot to control everything in-between"""
    SIGNAL_START_STREAM = 'ss'
    SIGNAL_CLOSE_STREAM = 'cs'
    SIGNAL_SWITCH_CONTROL_MODE = 'scm'
    SIGNAL_DISCONNECT = 'disconnect'
    SIGNAL_POWEROFF = 'poweroff'


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
    DEFAULT_SPEED = 150
    ROTATING_SPEED = 200
    MAX_SPEED = 255


class ArduinoSignals:
    SIGNAL_CONTROL_MODE_MANUAL = 0
    SIGNAL_CONTROL_MODE_LFA = 51
    SIGNAL_ROTATE_LEFT = 102
    SIGNAL_DRIVE_FORWARD = 153
    SIGNAL_DRIVE_BACKWARD = 204
    SIGNAL_ROTATE_RIGHT = 255


class RobotError(Exception):

    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class DataModel:

    # FIXME: Needs some extra work

    def __init__(self, data: dict[str, Any] | None = None) -> None:
        if data is None:
            data = dict()
        # Set data to model vars
        self.signal: str | None = data.get('signal', None)
        self.cmd: str | None = data.get('cmd', None)
        self.speed: int | None = data.get('speed', None)
        self.controlMode: str | None = data.get('ctrl_mode', None)
        self.isStreaming: bool | None = data.get('streaming', False)

    def __repr__(self) -> str:
        return 'DataModel[signal=\'{}\', command=\'{}\',speed=\'{}\',ctrlMode=\'{}\',isStreaming=\'{}\']'.format(
            self.signal, self.cmd, self.speed, self.controlMode, self.isStreaming)


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


def json2model(json: str | bytes) -> 'DataModel':
    try:
        if type(json) is bytes:
            json = json.decode('utf-8')
        return DataModel(JSON.loads(json))
    except JSON.JSONDecodeError as e:
        print(f'[json2model]: Error decoding JSON. Msg=[{e}].')
        return DataModel()


def get_robot_ip() -> str:
    # CODEME: Scan local network searching for the robot and return its IP address
    return gethostbyname('localhost')


def constrain(value: Any, min: Any, max: Any):
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value
