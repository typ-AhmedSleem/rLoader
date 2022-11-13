from streamer import Streamer, CAMERA_SOURCE
from sockets import ServerSocket
from time import sleep
import threading
import random
import rpi
from robot_utils import (
    constrain,
    get_my_ip,
    json2model,
    model2json,
    Speeds,
    Signals,
    DataModel,
    Directions,
    ControlModes,
    ArduinoSignals,
    PORT_RTV_SOCKET,
    PORT_DATA_SOCKET,
    DEFAULT_BUFFER_SIZE,
)


def create_thread(name: str, target):
    return threading.Thread(name=name, target=target)


class Robot:
    """Robot class acts like a server which communicates to WCU"""

    def __init__(self) -> None:
        # Communication runtime
        self.initialized = False
        self.running = False
        self.mHost = get_my_ip()
        # Control runtime
        self.gpio = rpi.GPIO()
        self.speed = Speeds.NO_SPEED
        self.control_mode = ControlModes.CONTROL_MODE_MANUAL
        # Data Server Socket
        self.dataServerSocket = ServerSocket()
        self.dataServerSocket.settimeout(5)
        # RTV Server Socket
        self.rtvServerSocket = ServerSocket()
        self.rtvServerSocket.settimeout(5)
        # Client Sockets
        self.dataSocket = None
        self.rtvSocket = None
        # Stream
        self.streamer = Streamer(CAMERA_SOURCE)
        # Threads
        self.ardThread = create_thread("Ard-Thread", self._ard_thread_task)
        self.comThread = create_thread("Com-Thread", self._com_thread_task)
        self.rtvThread = create_thread("RTV-Thread", self._rtv_thread_task)

    def _ard_thread_task(self):
        print("[Robot]: Started arduino thread work")
        # Setup GPIO
        self.gpio.setup()
        # Setup GPIO pins
        if self.control_mode == ControlModes.CONTROL_MODE_AUTOMATIC:
            # Pins are for INPUT
            self.gpio.pinMode(rpi.PIN_SPEED, rpi.INPUT)
            self.gpio.pinMode(rpi.PIN_SIGNALS, rpi.INPUT)
        else:
            # Pins are for OUTPUT
            self.gpio.pinMode(rpi.PIN_SPEED, rpi.OUTPUT)
            self.gpio.pinMode(rpi.PIN_SIGNALS, rpi.OUTPUT)
        # Start thread work
        while self.running:
            if self.control_mode == ControlModes.CONTROL_MODE_AUTOMATIC:
                # Read speed and direction-mask from pins
                self.speed = self.gpio.analogRead(rpi.PIN_SPEED)
                masked_ctrl_cmd = self.gpio.analogRead(rpi.PIN_SIGNALS)
                masked_ctrl_cmd = random.choice(
                    [
                        ArduinoSignals.SIGNAL_ROTATE_LEFT,
                        ArduinoSignals.SIGNAL_DRIVE_FORWARD,
                        ArduinoSignals.SIGNAL_DRIVE_BACKWARD,
                        ArduinoSignals.SIGNAL_ROTATE_RIGHT,
                    ]
                )  # HACK
                # UnMask the masked control mode
                match masked_ctrl_cmd:
                    case ArduinoSignals.SIGNAL_DRIVE_BACKWARD:
                        self.speed = Speeds.DEFAULT_SPEED
                        control_command = Directions.CMD_DRIVE_BACKWARD
                    case ArduinoSignals.SIGNAL_ROTATE_RIGHT:
                        self.speed = Speeds.ROTATING_SPEED
                        control_command = Directions.CMD_ROTATE_RIGHT
                    case ArduinoSignals.SIGNAL_ROTATE_LEFT:
                        self.speed = Speeds.ROTATING_SPEED
                        control_command = Directions.CMD_ROTATE_LEFT
                    case _:
                        self.speed = Speeds.DEFAULT_SPEED
                        control_command = Directions.CMD_DRIVE_FORWARD
                # Send report to WCU
                self.send_report(control_command, self.speed)
            # Delay for 500 ms to warm up
            sleep(0.5)

    def _com_thread_task(self):
        print("[Robot]: Started Communication-Thread work")
        while self.running:
            try:
                # Wait for WCU to connect
                print("[Robot]: Waiting for data connection to establish...")
                self.dataSocket, addr = self.dataServerSocket.accept()
                self.dataSocket.settimeout(5)
                print(f"[Robot]: Opened (DataChannel) with WCU on address [{addr}]")
                # Handle incoming data
                while self.dataSocket.opened:
                    try:
                        # Receive data json
                        rcvd_json = self.dataSocket.receive(DEFAULT_BUFFER_SIZE // 2)
                        # Check if data socket disconnected
                        if not rcvd_json or len(rcvd_json) == 0:
                            self.dataSocket.close()
                            print(
                                "[Robot]: Received empty packet which means connection has lost."
                            )
                            break
                        # Parse the json data to model
                        print("[Robot]: Received data from WCU -> {}".format(rcvd_json.decode("utf-8")))  # type: ignore
                        dataModel: DataModel = json2model(rcvd_json.decode("utf-8"))
                        # Handle signal (if specified)
                        if dataModel.signal:
                            match dataModel.signal:
                                case Signals.SIGNAL_START_STREAM:
                                    self.start_stream()
                                case Signals.SIGNAL_CLOSE_STREAM:
                                    self.stop_stream()
                                case Signals.SIGNAL_SWITCH_CONTROL_MODE:
                                    self.switch_control_mode(dataModel.controlMode)
                                case "poweroff":
                                    self.poweroff()
                                case Signals.SIGNAL_DISCONNECT:
                                    self.dataSocket.close()
                                    print("[Robot-COM]: Received (Disconnect) signal. Disconnecting...")
                                    break
                        # Handle control command (if specified)
                        if dataModel.cmd:
                            self.moveRobot(dataModel.cmd)
                        # Handle speed command (if specified)
                        if dataModel.speed is not None:
                            self.setSpeed(dataModel.speed)
                        # Check if has duration change
                        if dataModel.durations:
                            # Send change durations signal to arduino
                            self.changeDurations(dataModel.durations)
                    except AttributeError as e:
                        self.dataSocket.close()
                        break
                    except TimeoutError:
                        continue
                    except (
                        ConnectionResetError,
                        ConnectionAbortedError,
                        ConnectionRefusedError,
                    ):
                        print("[Robot-COM]: Connection unexpectedly closed.")
                        self.dataSocket.close()
                        break
                    except OSError as e:
                        print(f"[Robot-COM]:  Socket OSError -> {e.args[1]}")
                        self.dataSocket.close()
                        break
            except (
                ConnectionAbortedError,
                ConnectionRefusedError,
                ConnectionResetError,
                TimeoutError,
                OSError,
            ) as e:
                # print(f'[Robot-COM]: Can\'t open connection. Faced {type(e).__name__}')
                sleep(2)

    def _rtv_thread_task(self):
        print("[Robot]: Started Streaming-Thread work")
        while self.running:
            try:
                print("[Robot]: Waiting for rtv connection to establish...")
                self.rtvSocket, addr = self.rtvServerSocket.accept()
                print(f"[Robot]: Opened (RTVChannel) with WCU at address [{addr}].")
                if not self.streamer.streaming:
                    # Attach rtv-socket to streamer instance
                    self.streamer.socket = self.rtvSocket
                    # Start streaming
                    self.streamer.start_stream()
                sleep(2)
            except (
                ConnectionAbortedError,
                ConnectionRefusedError,
                ConnectionResetError,
                TimeoutError,
                OSError,
            ) as e:
                # print(f'[Robot-RTV]: Can\'t open connection. Faced {type(e).__name__}')
                sleep(2)

    def poweron(self):
        if self.initialized:
            print("[Robot]: Already powered on.")
            return
        print("[Robot]: Powering on Robot...")
        print("-" * 100)
        print("[Robot]: Current configuration: (speed={} | ctrlMode={})".format(self.speed, self.control_mode))
        print("-" * 100)
        # Bind sockets
        self.dataServerSocket.bind((self.mHost, PORT_DATA_SOCKET))
        self.dataServerSocket.listen(1)
        print(
            f"[Robot]: Bound Data Server Socket to address: ({self.mHost}:{PORT_DATA_SOCKET})"
        )
        self.rtvServerSocket.bind((self.mHost, PORT_RTV_SOCKET))
        self.rtvServerSocket.listen(1)
        print(
            f"[Robot]: Bound RTV Server Socket to address: ({self.mHost}:{PORT_RTV_SOCKET})"
        )
        # Create threads
        self.ardThread = create_thread("Ard-Thread", self._ard_thread_task)
        self.comThread = create_thread("Com-Thread", self._com_thread_task)
        self.rtvThread = create_thread("RTV-Thread", self._rtv_thread_task)
        # Update runtime
        self.running = True
        self.initialized = True
        # Start threads
        self.ardThread.start()
        self.comThread.start()
        # Log
        print("[Robot]: Powered on successfully.")

    def poweroff(self):
        print("[Robot]: Powering off Robot...")
        # Update runtime
        self.running = False
        # Cleanup GPIO
        self.gpio.cleanup()
        # Stop streamer
        self.streamer.stop_stream()
        # Close data socket
        if self.dataSocket:
            try:
                self.dataSocket.close()
                self.dataServerSocket.close()
                print("[Robot]: Closed data socket successfully.")
            except:
                print("[Robot]: Error closing data socket.")
        # Join thread
        try:
            self.comThread.join()
            print("[Robot]: Joined Com-Thread successfully.")
        except RuntimeError:
            print("[Robot]: Error joining Com-Thread.")
        # Log
        print(
            "[Robot]: Powered off successfully. {} | {}".format(
                self.comThread.is_alive(), self.rtvThread.is_alive()
            )
        )
        exit(0)

    def start_stream(self):
        if not self.streamer.streaming:
            if (self.rtvThread) and (self.rtvThread.is_alive()):
                print("[Robot]: Closing opened stream")
                try:
                    self.rtvThread.join()
                    print("[Robot]: Joined RTV-Thread successfully.")
                except RuntimeError:
                    print("[Robot]: Error joining RTV-Thread.")
                finally:
                    self.rtvThread = create_thread("RTV-Thread", self._rtv_thread_task)
            # Start the thread
            self.rtvThread.start()
        else:
            print("[Robot]: Already streaming.")

    def stop_stream(self):
        if not self.streamer.streaming:
            try:
                print(f"[Robot]: Stopping stream")
                self.streamer.stop_stream()
                self.rtvThread.join()
                print(f"[Robot]: Stopped streaming to WCU successfully.")
            except RuntimeError as e:
                print(f"[Robot]: Error joining RTV-Thread.  Msg=({e})")
        else:
            print("[Robot]: Not streaming already. You must call start_stream() first.")

    def send_report(self, ctrl_cmd: str, speed: int):
        if self.dataSocket and self.dataSocket.opened:
            # Send the report
            report = model2json(control_command=ctrl_cmd, speed=speed)
            if self.dataSocket.send(report) > 0:
                print("[Robot]: Sent report ({}) to WCU.".format(report))

    def switch_control_mode(self, new_control_mode: str | None):
        if new_control_mode:
            if new_control_mode == self.control_mode:
                return  # Control mode not changed
            # Change control mode to new mode
            self.control_mode = new_control_mode
            # Switch control mode
            if self.control_mode == ControlModes.CONTROL_MODE_AUTOMATIC:
                print(f"[Robot]: Switched to Manual Control Mode.")
                arduino_signal = ArduinoSignals.SIGNAL_CONTROL_MODE_MANUAL
            else:
                print(f"[Robot]: Switched to Automatic Control Mode.")
                arduino_signal = ArduinoSignals.SIGNAL_CONTROL_MODE_LFA
            # Switch control mode for Arduino board
            self.gpio.analogWrite(rpi.PIN_SIGNALS, arduino_signal)

    def setSpeed(self, newSpeed: int):
        if self.speed == newSpeed:
            return  # Speed not changed
        # Update speed (NOTE: idk why am constraining speed again as it's already constrained)
        self.speed = constrain(newSpeed, Speeds.NO_SPEED, Speeds.MAX_SPEED)
        # Send update speed for motors (if in manual control mode) for Arduino board
        if self.control_mode == ControlModes.CONTROL_MODE_MANUAL:
            self.gpio.analogWrite(
                rpi.PIN_SPEED, self.speed
            )  # INFO: Already mapped speed between 0 and 100
        print(f"[Robot]: Changed speed to ({self.speed})")

    def moveRobot(self, dirCommand: str):
        if self.control_mode == ControlModes.CONTROL_MODE_AUTOMATIC:
            return
        # Move the robot
        match dirCommand:
            case Directions.CMD_DRIVE_FORWARD:
                self.gpio.analogWrite(
                    rpi.PIN_SIGNALS, ArduinoSignals.SIGNAL_DRIVE_FORWARD
                )
                print(f"[Robot]: Moved Forward.")
            case Directions.CMD_DRIVE_BACKWARD:
                self.gpio.analogWrite(
                    rpi.PIN_SIGNALS, ArduinoSignals.SIGNAL_DRIVE_BACKWARD
                )
                print(f"[Robot]: Moved Backward.")
            case Directions.CMD_ROTATE_RIGHT:
                self.gpio.analogWrite(
                    rpi.PIN_SIGNALS, ArduinoSignals.SIGNAL_ROTATE_RIGHT
                )
                print(f"[Robot]: Rotated Right.")
            case Directions.CMD_ROTATE_LEFT:
                self.gpio.analogWrite(
                    rpi.PIN_SIGNALS, ArduinoSignals.SIGNAL_ROTATE_LEFT
                )
                print(f"[Robot]: Rotated Left.")

    def changeDurations(self, durations: tuple[int, int, int, int] | None = None):
        if durations and len(durations) == 4:
            # TODO: Send change durations signal through signals channel pin first.
            # TODO: Send durations in same order of tuple through data channel pin.
            pass


if __name__ == "__main__":
    Robot().poweron()
