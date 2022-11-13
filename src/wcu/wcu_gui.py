from email.mime import image
import socket
import struct
import sys
import pickle
import threading
from time import sleep
import cv2 as cv
import numpy as np
from datetime import datetime
from sockets import ClientSocket
from PyQt5 import QtGui, QtWidgets, QtCore
from stream_recorder import StreamRecorder
from wcu_utils import (
    ControlCommandsTexts as Texts,
    Status,
    TARGET_FRAME_SIZE,
    get_switched_ctrl_mode,
)
from robot_utils import (
    ControlModes,
    RobotError,
    Directions,
    DataModel,
    Signals,
    Speeds,
    constrain,
    model2json,
    json2model,
    get_robot_ip,
    PORT_RTV_SOCKET,
    PORT_DATA_SOCKET,
    STREAM_BUFFER_SIZE,
)


def create_thread(name, target):
    return threading.Thread(name=name, target=target)


class DataHandler:
    def __init__(
        self,
        on_connecting,
        on_connect,
        on_handle_report,
        on_connect_fail,
        on_disconnect,
    ) -> None:
        self.on_connecting = on_connecting
        self.on_connect = on_connect
        self.on_handle_report = on_handle_report
        self.on_connect_fail = on_connect_fail
        self.on_disconnect = on_disconnect


class DataWorker:
    def __init__(self, robot_ip: str, data_handler: DataHandler = None) -> None:  # type: ignore
        self.robot_ip = robot_ip
        self.lock = threading.Lock()
        self.working = False
        self.connecting = False
        self.data_handler = data_handler
        self.data_socket = ClientSocket()
        self._data_thread = create_thread("Data-Thread", self._work_task)

    @property
    def connected(self) -> bool:
        return self.working and self.data_socket.opened

    def start_work(self):
        # Check if thread has acquired lock
        if self.lock.locked():
            raise RobotError("[DataWorker]: Worker is handling another method call.")
        # Acquire lock
        self.lock.acquire()
        # Check if attempts to start while already running
        if self.working:
            raise RobotError("[DataWorker]: Worker is already running.")
        try:
            # Start the thread or start new one
            self._data_thread.start()
        except RuntimeError:
            # Create a new thread object
            self._data_thread = create_thread("Data-Thread", self._work_task)
            # Start the thread
            self._data_thread.start()
        # Release lock
        self.lock.release()

    def finish_work(self):
        # Check if thread has acquired lock
        if self.lock.locked():
            raise RobotError("[DataWorker]: Worker is handling another method call.")
        # Acquire lock
        self.lock.acquire()
        # Update runtime
        self.working = False
        # Close socket
        self.data_socket.close()
        try:
            # Join thread
            self._data_thread.join()
        except RuntimeError as e:
            print("Cant join thread. {}".format(e))
        finally:
            self._data_thread = create_thread("Data-Thread", self._work_task)
        # Release lock
        self.lock.release()

    def send(self, data: str) -> int:
        if data is not None:
            # Encode data if not yet encoded
            if self.connected:
                try:
                    # Send the data
                    return self.data_socket.send(data.encode("utf-8"))
                except (
                    ConnectionAbortedError,
                    ConnectionRefusedError,
                    ConnectionResetError,
                    OSError,
                ):
                    return -1
            else:
                return -1
        else:
            return 0  # Sent nothing.

    def _work_task(self):
        try:
            # Open connection with Robot at first
            self.data_socket = ClientSocket()
            self.data_socket.settimeout(10)
            self.data_handler.on_connecting()
            self.connecting = True
            self.data_socket.connect((self.robot_ip, PORT_DATA_SOCKET))
            self.connecting = False
            self.working = True
            self.data_socket.settimeout(1)
            self.data_handler.on_connect()
            while self.connected:
                # Handle incoming report (Only works on Auto control mode)
                try:
                    report_json = self.data_socket.receive(STREAM_BUFFER_SIZE)
                    if not report_json or len(report_json) == 0:
                        # Lost connection
                        self.data_socket.close()
                        self.data_handler.on_disconnect()
                        self.working = False
                        print("[DataWorker]: Received empty packet which means that connection was lost.")
                        break
                    else:
                        if self.data_handler:
                            self.data_handler.on_handle_report(json2model(report_json))
                except TimeoutError:
                    continue
                except (ConnectionResetError, ConnectionAbortedError, OSError):
                    # Notify GUI
                    self.data_socket.close()
                    self.data_handler.on_disconnect()
                    self.working = False
        except TimeoutError:
            self.data_handler.on_connect_fail()
        except (
            ConnectionAbortedError,
            ConnectionRefusedError,
            ConnectionResetError,
            OSError,
        ) as e:
            if self.connected:
                # Lost connection
                self.data_socket.close()
                self.data_handler.on_disconnect()
                self.working = False
            else:
                # Failed to connect
                self.data_handler.on_connect_fail()


class StreamHandler:
    def __init__(
        self,
        on_connecting,
        on_connect,
        on_receive_first_frame,
        on_handle_frame,
        on_disconnect,
    ) -> None:
        self.on_connecting = on_connecting
        self.on_connect = on_connect
        self.on_receive_first_frame = on_receive_first_frame
        self.on_handle_frame = on_handle_frame
        self.on_disconnect = on_disconnect


class StreamWorker:
    def __init__(self, robot_ip: str, stream_handler: StreamHandler = None) -> None:  # type: ignore
        self.robot_ip = robot_ip
        self.lock = threading.Lock()
        self.working = False
        self.recorder = StreamRecorder()
        self.rtv_socket = ClientSocket()
        self.stream_handler = stream_handler
        self._rtv_thread = create_thread("RTV-Thread", self._work_task)

    @property
    def connected(self) -> bool:
        return self.working and self.rtv_socket.opened

    def start_work(self):
        # Check if thread has acquired lock
        if self.lock.locked():
            raise RobotError("[StreamWorker]: Worker is handling another method call.")
        # Acquire lock
        self.lock.acquire()
        # Check if attempts to start while already running
        if self.working:
            raise RobotError("[StreamWorker]: Worker is already running.")
        try:
            # Start the thread or start new one
            self._rtv_thread.start()
        except RuntimeError:
            # Create a new thread object
            self._rtv_thread = create_thread("RTV-Thread", self._work_task)
            # Start the thread
            self._rtv_thread.start()
        # Release lock
        self.lock.release()

    def finish_work(self):
        # Check if thread has acquired lock
        if self.lock.locked():
            raise RobotError("[StreamWorker]: Worker is handling another method call.")
        # Acquire lock
        self.lock.acquire()
        # Update runtime
        self.working = False
        # Close connection
        self.rtv_socket.close()
        # Close recorder (if already running)
        if self.recorder.recording:
            self.recorder.stop_recording()
        try:
            # Join thread
            self._rtv_thread.join()
        except RuntimeError as e:
            print("[StreamWorker]: Cant join thread. {}".format(e))
        finally:
            self._rtv_thread = create_thread("RTV-Thread", self._work_task)
        # Release lock
        self.lock.release()

    def _work_task(self):
        try:
            received_first_frame = False
            notified_callback = False
            self.rtv_socket = ClientSocket()
            self.rtv_socket.settimeout(1)
            # Open socket connection with Robot
            self.stream_handler.on_connecting()
            self.rtv_socket.connect((self.robot_ip, PORT_RTV_SOCKET))
            # Notify Handler callback
            self.working = True
            self.stream_handler.on_connect()
            while self.connected:
                # Check if worker has notified callback about first frame received
                if received_first_frame and not notified_callback:
                    notified_callback = True
                    self.stream_handler.on_receive_first_frame()
                # Receive stream from Robot
                received_bytes = b""
                payload_size = struct.calcsize("L")
                try:
                    # Receive the first bytes that relates to frame size
                    while len(received_bytes) < payload_size:
                        packet = self.rtv_socket.receive(STREAM_BUFFER_SIZE)
                        if not packet:
                            print("[StreamWorker]: Received empty packet which means that connection was lost.")
                            self.rtv_socket.close()
                            self.working = False
                            break
                        # Append packet to rcvd bytes
                        received_bytes += packet
                    # Split frame size from frame data
                    packed_frame_size = received_bytes[:payload_size]
                    received_bytes = received_bytes[payload_size:]
                    # Unpack bytes to get real frame size
                    frame_size = struct.unpack("L", packed_frame_size)[0]
                    # Receive remaining frame bytes
                    while len(received_bytes) < frame_size:
                        packet = self.rtv_socket.receive(STREAM_BUFFER_SIZE)
                        if not packet:
                            print("[StreamWorker]: Received empty packet which means that connection was lost.")
                            self.rtv_socket.close()
                            self.working = False
                            break
                        received_bytes += packet
                    # Split complete frame bytes from received bytes
                    frame_bytes = received_bytes[:frame_size]
                    received_bytes = received_bytes[frame_size:]
                    # Unpickle the frame bytes
                    frame = pickle.loads(frame_bytes)
                    # Convert the frame to np array
                    frame = np.array(frame)
                    # Convert the frame into QT displayable image
                    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                    h, w, ch = frame.shape
                    bytes_per_line = ch * w
                    image = QtGui.QImage(
                        frame.data,
                        w,
                        h,
                        bytes_per_line,
                        QtGui.QImage.Format.Format_RGB888,
                    )
                    image = image.scaled(
                        TARGET_FRAME_SIZE[0],
                        TARGET_FRAME_SIZE[1],
                        QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                    )
                    # Send image to GUI to be handled
                    self.stream_handler.on_handle_frame(image)
                    # Write the frame (if recording)
                    if self.recorder.recording:
                        self.recorder.write_frame(frame)
                    # Check if this is the first frame
                    if not received_first_frame:
                        received_first_frame = True
                except TimeoutError:
                    continue
                except (struct.error, pickle.UnpicklingError) as e:
                    print(f"[StreamWorker]: {type(e).__name__}")
                    self.rtv_socket.close()
                    self.working = False
                    self.stream_handler.on_disconnect()
                    break
                except (
                    ConnectionAbortedError,
                    ConnectionRefusedError,
                    ConnectionResetError,
                    OSError,
                ) as e:
                    if self.connected:
                        self.rtv_socket.close()
                        self.working = False
                        self.stream_handler.on_disconnect()
                        break
        except TimeoutError:
            pass
        except (
            ConnectionAbortedError,
            ConnectionRefusedError,
            ConnectionResetError,
            OSError,
        ) as e:
            if self.connected:
                self.rtv_socket.close()
                self.working = False
                self.stream_handler.on_disconnect()
        sleep(2)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super(MainWindow, self).__init__()
        # Setup constants
        self.SPEED_STEP = 10
        self.MARGIN_SIZE = 15
        self.SCREEN_W = self.screen().size().width()
        self.SCREEN_H = self.screen().size().height()
        self.WINDOW_SIZE = QtCore.QSize(self.SCREEN_W, self.SCREEN_H)
        # Setup runtime
        self.speed = 0
        self.robot_ip = get_robot_ip()
        self.controlMode = ControlModes.CONTROL_MODE_MANUAL
        self.stream_worker = StreamWorker(
            self.robot_ip,
            StreamHandler(
                self._on_stream_worker_connecting,
                self._on_stream_worker_connect,
                self._on_receive_first_frame,
                self._on_handle_stream_frame,
                self._on_stream_worker_disconnect,
            ),
        )
        self.data_worker = DataWorker(
            self.robot_ip,
            DataHandler(
                self._on_data_worker_connecting,
                self._on_data_worker_connect,
                self._handle_report,
                self._on_data_worker_connect_fail,
                self._on_data_worker_disconnect,
            ),
        )
        # Setup root and container
        root = QtWidgets.QWidget(self)
        root.setFixedSize(self.WINDOW_SIZE)
        # INFO: Setup (RTV & Status) container
        self.rtvStatusContainer = QtWidgets.QWidget(root)
        self.rtvStatusContainer.setGeometry(
            0,
            0,
            self.WINDOW_SIZE.width() // 2,
            self.WINDOW_SIZE.height() - self.MARGIN_SIZE,
        )
        # Realtime Video stream ImageView
        self.imgRTV = QtWidgets.QLabel(self.rtvStatusContainer)
        self.imgRTV.setScaledContents(True)
        self.imgRTV.setMinimumSize(900, 600)
        self.imgRTV.setMaximumSize(900, 600)
        self.imgRTV.setGeometry(self.MARGIN_SIZE, self.MARGIN_SIZE, 900, 600)
        # Robot current speed label
        self._lblLogList = QtWidgets.QLabel(self.rtvStatusContainer)
        self._lblLogList.setText("WCU Log")
        self._lblLogList.setFont(QtGui.QFont("monospace", 15))
        self._lblLogList.setGeometry(
            self.MARGIN_SIZE,
            self.imgRTV.height() + self.MARGIN_SIZE,
            self.imgRTV.width(),
            50,
        )
        # WCU log ListView
        self.listLog = QtWidgets.QListWidget(self.rtvStatusContainer)
        self.listLog.setSpacing(1)
        self.listLog.setFixedWidth(900)
        self.listLog.setSelectionRectVisible(False)
        self.listLog.setSelectionMode(QtWidgets.QListView.SelectionMode.NoSelection)
        self.listLog.setGeometry(
            self.MARGIN_SIZE,
            self._lblLogList.geometry().bottom(),
            self.imgRTV.width(),
            self.WINDOW_SIZE.height() - self._lblLogList.geometry().bottom() + 1 - 2 * self.MARGIN_SIZE)
        # INFO: Speed container
        # Calculate X coordinate for speed container
        targetXCoord = self.rtvStatusContainer.width() - self.MARGIN_SIZE
        targetWidth = self.WINDOW_SIZE.width() - targetXCoord - 1 * self.MARGIN_SIZE
        self.speedContainer = QtWidgets.QWidget(root)
        self.speedContainer.setGeometry(
            targetXCoord, 0, targetWidth, self.WINDOW_SIZE.height() - self.MARGIN_SIZE
        )
        # Calculate X coordinate for its children
        targetXCoord = self.MARGIN_SIZE
        targetWidth = self.speedContainer.width() - 1 * self.MARGIN_SIZE
        # Robot current speed label
        self._lblRobotSpeed = QtWidgets.QLabel(self.speedContainer)
        self._lblRobotSpeed.setMaximumHeight(50)
        self._lblRobotSpeed.setMinimumHeight(50)
        self._lblRobotSpeed.setFixedSize(targetWidth, 50)
        self._lblRobotSpeed.setText("Robot Current Speed")
        self._lblRobotSpeed.setFont(QtGui.QFont("monospace", 15))
        self._lblRobotSpeed.setGeometry(0, self.MARGIN_SIZE, targetWidth, 100)
        # Robot speed lcd number display
        self.lcdSpeed = QtWidgets.QLCDNumber(3, self.speedContainer)
        self.lcdSpeed.display(self.speed)
        self.lcdSpeed.setMinimumHeight(100)
        self.lcdSpeed.setMaximumHeight(100)
        self.lcdSpeed.setObjectName("lcdSpeed")
        self.lcdSpeed.setFixedSize(targetWidth, 100)
        self.lcdSpeed.setStyleSheet("color: #052BF3;")
        self.lcdSpeed.setFrameStyle(QtWidgets.QFrame.Shape.NoFrame)
        self.lcdSpeed.setSegmentStyle(QtWidgets.QLCDNumber.SegmentStyle.Flat)
        self.lcdSpeed.setGeometry(
            self.MARGIN_SIZE,
            self.MARGIN_SIZE + self._lblRobotSpeed.geometry().bottom() + 1,
            300,
            100,
        )
        # Control commands label
        self._lblCommands = QtWidgets.QLabel(self.speedContainer)
        self._lblCommands.setMaximumHeight(50)
        self._lblCommands.setMinimumHeight(50)
        self._lblCommands.setFixedSize(targetWidth, 50)
        self._lblCommands.setText("Robot Control Commands")
        self._lblCommands.setFont(QtGui.QFont("monospace", 15))
        self._lblCommands.setGeometry(
            0,
            self.lcdSpeed.geometry().bottom() + 1 + self.MARGIN_SIZE,
            self._lblRobotSpeed.width(),
            100,
        )
        # Robot control commands display
        self.lblControlCommands = QtWidgets.QLabel(self.speedContainer)
        self.lblControlCommands.setMaximumHeight(50)
        self.lblControlCommands.setMinimumHeight(100)
        self.lblControlCommands.setStyleSheet("color: #052BF3;")
        self.lblControlCommands.setFixedSize(targetWidth, 50)
        self.lblControlCommands.setText(Texts.TEXT_NOT_CONNECTED)
        self.lblControlCommands.setFont(QtGui.QFont("monospace", 25))
        self.lblControlCommands.setAlignment(QtCore.Qt.AlignmentFlag.AlignJustify)
        self.lblControlCommands.setGeometry(
            0,
            self._lblCommands.geometry().bottom() + 1 + self.MARGIN_SIZE,
            self._lblRobotSpeed.width(),
            100,
        )
        # Connect/Disconnect button
        self.btnConnectDisconnect = QtWidgets.QPushButton(self.speedContainer)
        self.btnConnectDisconnect.setText("Connect")
        self.btnConnectDisconnect.setMaximumHeight(100)
        self.btnConnectDisconnect.setMinimumHeight(100)
        self.btnConnectDisconnect.setFont(QtGui.QFont("monospace", 15))
        self.btnConnectDisconnect.clicked.connect(self.connect_disconnect)
        self.btnConnectDisconnect.setGeometry(0, self.lblControlCommands.geometry().bottom() + 1 + self.MARGIN_SIZE, targetWidth, 100,)
        # Start/Stop Stream button
        self.btnStartStopStream = QtWidgets.QPushButton(self.speedContainer)
        self.btnStartStopStream.setText(Texts.TEXT_START_STREAM)
        self.btnStartStopStream.setMaximumHeight(100)
        self.btnStartStopStream.setMinimumHeight(100)
        self.btnStartStopStream.setFont(QtGui.QFont("monospace", 15))
        self.btnStartStopStream.clicked.connect(self.start_stop_stream)
        self.btnStartStopStream.setGeometry(0, self.btnConnectDisconnect.geometry().bottom() + 1 + self.MARGIN_SIZE, targetWidth, 100)
        # Start/Stop Stream button
        self.btnRecordStream = QtWidgets.QPushButton(self.speedContainer)
        self.btnRecordStream.setText(Texts.TEXT_START_STREAM)
        self.btnRecordStream.setMaximumHeight(100)
        self.btnRecordStream.setMinimumHeight(100)
        self.btnRecordStream.setFont(QtGui.QFont("monospace", 15))
        self.btnRecordStream.clicked.connect(self.start_stop_record)
        self.btnRecordStream.setGeometry(0, self.btnStartStopStream.geometry().bottom() + 1 + self.MARGIN_SIZE, targetWidth, 100)
        # Change control mode button
        self.btnSwitchControlMode = QtWidgets.QPushButton(self.speedContainer)
        self.btnSwitchControlMode.setMaximumHeight(100)
        self.btnSwitchControlMode.setMinimumHeight(100)
        self.btnSwitchControlMode.setText("Switch Control Mode")
        self.btnSwitchControlMode.setFont(QtGui.QFont("monospace", 15))
        self.btnSwitchControlMode.clicked.connect(self.switch_control_mode)
        self.btnSwitchControlMode.setGeometry(
            0,
            self.btnRecordStream.geometry().bottom() + 1 + self.MARGIN_SIZE,
            targetWidth,
            100,
        )
        # Help button
        self.btnHelp = QtWidgets.QPushButton(self.speedContainer)
        self.btnHelp.setText("Show Help")
        self.btnHelp.setMaximumHeight(100)
        self.btnHelp.setMinimumHeight(100)
        self.btnHelp.clicked.connect(self.showHelp)
        self.btnHelp.setFont(QtGui.QFont("monospace", 15))
        self.btnHelp.setGeometry(0, self.btnSwitchControlMode.geometry().bottom() + 1 + self.MARGIN_SIZE, targetWidth, 100)
        # Exit button
        self.btnQuit = QtWidgets.QPushButton(self.speedContainer)
        self.btnQuit.setText("Exit WCU")
        self.btnQuit.setMaximumHeight(100)
        self.btnQuit.setMinimumHeight(100)
        self.btnQuit.clicked.connect(self.close)
        self.btnQuit.setFont(QtGui.QFont("monospace", 15))
        self.btnQuit.setGeometry(0, self.btnHelp.geometry().bottom() + 1 + self.MARGIN_SIZE, targetWidth, 100)
        # Set container and root to window
        self.grabKeyboard()
        self.setCentralWidget(root)
        self.setWindowTitle("Robot Wireless Control Unit")
        self.setMinimumSize(self.WINDOW_SIZE.width(), self.WINDOW_SIZE.height())
        self.setMaximumSize(self.WINDOW_SIZE.width(), self.WINDOW_SIZE.height())
        self.showFullScreen()

    @property
    def status(self) -> int:
        return Status.CONNECTED if self.data_worker.connected else Status.NOT_CONNECTED

    def _on_data_worker_connecting(self):
        self.logToList("DataWorker trying to establish connection...")
        self.btnStartStopStream.setEnabled(False)
        self.btnSwitchControlMode.setEnabled(False)
        self.btnConnectDisconnect.setEnabled(False)
        self.lblControlCommands.setText(Texts.TEXT_CONNECTING)
        self.btnStartStopStream.setText(Texts.TEXT_START_STREAM)
        self.btnConnectDisconnect.setText(Texts.TEXT_CONNECTING)

    def _on_data_worker_connect(self):
        if self.status == Status.CONNECTED:
            self.update_ui_to_connected()
            # Send current configuration to robot
            if (self.data_worker.send(model2json(signal=Signals.SIGNAL_SWITCH_CONTROL_MODE, control_mode=self.controlMode, speed=self.speed)) > 0):
                self.logToList("Sent current configuration to Robot.")
            self.logToList("DataWorker established connection on address ({}:{}).".format(self.robot_ip, PORT_DATA_SOCKET))

    def _handle_report(self, report: DataModel):
        if report:
            # Update GUI
            if report.speed:
                self.lcdSpeed.display(report.speed)
            if report.cmd:
                match report.cmd:
                    case Directions.CMD_DRIVE_FORWARD:
                        self.lblControlCommands.setText(Texts.TEXT_ACCELERATING)
                    case Directions.CMD_DRIVE_BACKWARD:
                        self.lblControlCommands.setText(Texts.TEXT_DECELERATING)
                    case Directions.CMD_ROTATE_RIGHT:
                        self.lblControlCommands.setText(Texts.TEXT_TURNING_RIGHT)
                    case Directions.CMD_ROTATE_LEFT:
                        self.lblControlCommands.setText(Texts.TEXT_TURNING_LEFT)

    def _on_data_worker_connect_fail(self):
        self.imgRTV.setEnabled(False)
        self.lcdSpeed.setEnabled(False)
        self.btnRecordStream.setEnabled(False)
        self.btnStartStopStream.setEnabled(False)
        self.btnConnectDisconnect.setEnabled(True)
        self.btnSwitchControlMode.setEnabled(False)
        self.btnConnectDisconnect.setText("Reconnect to Robot")
        self.btnStartStopStream.setText(Texts.TEXT_START_STREAM)
        self.lblControlCommands.setText(Texts.TEXT_FAILED_TO_CONNECT)
        self.btnRecordStream.setText(Texts.TEXT_START_RECORDING_STREAM)
        self.logToList("Can't connect to Robot. Press reconnect to try again..")

    def _on_data_worker_disconnect(self):
        if self.status == Status.NOT_CONNECTED:
            self.update_ui_to_disconnected()
            self.logToList("DataWorker has lost connection.")

    def _on_stream_worker_connecting(self):
        # TODO: Show loader image in imgRTV
        self.imgRTV.setEnabled(True)
        self.btnStartStopStream.setEnabled(False)
        self.btnStartStopStream.setText(Texts.TEXT_REQUESTING_STREAM)
        self.logToList("StreamWorker trying to establish connection...")

    def _on_stream_worker_connect(self):
        # TODO: Show (waiting for stream) image in imgRTV
        self.imgRTV.setEnabled(True)
        self.logToList("StreamWorker established connection on address ({}:{})".format(self.robot_ip, PORT_RTV_SOCKET))

    def _on_receive_first_frame(self):
        self.imgRTV.setEnabled(True)
        self.btnRecordStream.setEnabled(True)
        self.btnStartStopStream.setEnabled(True)
        self.btnStartStopStream.setText(Texts.TEXT_STOP_STREAM)
        self.btnRecordStream.setText(Texts.TEXT_START_RECORDING_STREAM)

    def _on_handle_stream_frame(self, image: QtGui.QImage):
        if image:
            self.imgRTV.setPixmap(QtGui.QPixmap().fromImage(image))

    def _on_stream_worker_disconnect(self):
        # TODO: Show disconnected image in imgRTV
        self.imgRTV.setEnabled(False)
        self.btnRecordStream.setEnabled(False)
        self.btnStartStopStream.setEnabled(True)
        self.logToList("StreamWorker has lost connection.")
        self.btnStartStopStream.setText(Texts.TEXT_START_STREAM)
        self.btnRecordStream.setText(Texts.TEXT_START_RECORDING_STREAM)
        # Stop recorder if already recording
        if self.recording:
            self.stream_worker.recorder.stop_recording()

    def showEvent(self, event: QtGui.QShowEvent) -> None:
        if self.status == Status.NOT_CONNECTED:
            self.logToList("GUI Initialized Successfully.")
            self.update_ui_to_disconnected()
            self.logToList("Press (Connect) button to connect to Robot.")
        return super().showEvent(event)

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.logToList("Closing WCU...")
        # Send off signal to Robot
        if self.data_worker.send(model2json(signal=Signals.SIGNAL_DISCONNECT)) > 0:
            # Release workers
            if self.data_worker.working:
                self.logToList("Releasing data worker...")
                self.data_worker.finish_work()
                self.logToList("Released data worker successfully.")
            if self.stream_worker.working:
                self.logToList("Releasing stream worker...")
                self.stream_worker.finish_work()
                self.logToList("Released stream worker successfully.")
        return super().closeEvent(a0)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        # Check if WCU is connected or not
        if self.status == Status.NOT_CONNECTED:
            return
        # Operate according to pressed key
        match event.key():
            case QtCore.Qt.Key.Key_W | QtCore.Qt.Key.Key_Up:
                if self.controlMode == ControlModes.CONTROL_MODE_MANUAL:
                    # Send drive forward command to robot
                    if (self.data_worker.send(model2json(control_command=Directions.CMD_DRIVE_FORWARD)) > 0):
                        self.logToList("Sent (DRIVE FORWARD) command to Robot.")
                        self.lblControlCommands.setText(Texts.TEXT_DRIVING_FORWARD)
                        self.lcdSpeed.display(Speeds.MAX_SPEED // 255 * 100)
                    else:
                        if self.speed >= 15 and self.speed < 100:
                            self.setSpeed(self.speed + self.SPEED_STEP)
            case QtCore.Qt.Key.Key_A | QtCore.Qt.Key.Key_Left:
                if self.controlMode == ControlModes.CONTROL_MODE_MANUAL:
                    # Send rotate left command to robot
                    if (self.data_worker.send(model2json(control_command=Directions.CMD_ROTATE_LEFT)) > 0):
                        self.logToList("Sent (ROTATE LEFT) command to Robot.")
                        self.lblControlCommands.setText(Texts.TEXT_TURNING_LEFT)
                        self.lcdSpeed.display(round((Speeds.ROTATING_SPEED / 255) * 100))
            case QtCore.Qt.Key.Key_S | QtCore.Qt.Key.Key_Down:
                if self.controlMode == ControlModes.CONTROL_MODE_MANUAL:
                    # Send drive backward command to robot
                    if (self.data_worker.send(model2json(control_command=Directions.CMD_DRIVE_FORWARD)) > 0):
                        self.logToList("Sent (DRIVE BACKWARD) command to Robot.")
                        self.lblControlCommands.setText(Texts.TEXT_DRIVING_BACKWARD)
                        self.lcdSpeed.display(Speeds.MAX_SPEED // 255 * 100)
                    else:
                        if self.speed > 10 and self.speed < 100:
                            self.setSpeed(self.speed - self.SPEED_STEP)
            case QtCore.Qt.Key.Key_D | QtCore.Qt.Key.Key_Right:
                if self.controlMode == ControlModes.CONTROL_MODE_MANUAL:
                    # Send rotate right command to robot
                    if (self.data_worker.send(model2json(control_command=Directions.CMD_ROTATE_RIGHT)) > 0):
                        self.logToList("Sent (ROTATE RIGHT) command to Robot.")
                        self.lblControlCommands.setText(Texts.TEXT_TURNING_RIGHT)
                        self.lcdSpeed.display(round((Speeds.ROTATING_SPEED / 255) * 100))
            case QtCore.Qt.Key.Key_Space | QtCore.Qt.Key.Key_B:
                if self.controlMode == ControlModes.CONTROL_MODE_MANUAL:
                    # Send stop command to robot
                    if self.data_worker.send(model2json(speed=0)) > 0:
                        self.logToList("Sent (STOP) command to Robot.")
                        self.lblControlCommands.setText(Texts.TEXT_ACTIVATING_BRAKES)
                        self.lcdSpeed.display(Speeds.NO_SPEED)
            case QtCore.Qt.Key.Key_Escape | QtCore.Qt.Key.Key_Q:
                # Exit the application
                self.close()
        return super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        # Check if WCU is connected or not
        if self.status == Status.CONNECTED:
            if self.controlMode == ControlModes.CONTROL_MODE_MANUAL:
                self.lblControlCommands.setText(Texts.TEXT_IDLE)
                self.lcdSpeed.display(Speeds.NO_SPEED)
            else:
                self.lblControlCommands.setText(Texts.TEXT_LFA_MODE_ENABLED)
        else:
            self.lblControlCommands.setText(Texts.TEXT_NOT_CONNECTED)
        return super().keyReleaseEvent(event)

    def setSpeed(self, newSpeed: int):
        # Check if WCU is connected or not
        if self.status == Status.NOT_CONNECTED:
            self.lblControlCommands.setText(Texts.TEXT_NOT_CONNECTED)
            return
        # Quit invocation if new speed equals the current speed
        if newSpeed == self.speed:
            return
        # Check if newSpeed exceeds the limits
        if newSpeed > 100:
            self.lblControlCommands.setText(Texts.TEXT_REACHED_MAX_SPEED)
            return
        elif newSpeed < 0:
            self.lblControlCommands.setText(Texts.TEXT_REACHED_MIN_SPEED)
            return
        # Send new speed to robot
        if self.data_worker.send(model2json(speed=newSpeed)) > 0:
            # Update GUI
            if not newSpeed:
                self.lblControlCommands.setText(Texts.TEXT_ACTIVATING_BRAKES)
                self.logToList("Sent (Activate Brakes) signal to Robot.")
            else:
                if newSpeed > self.speed:
                    self.lblControlCommands.setText(Texts.TEXT_ACCELERATING)
                    self.logToList("Sent (Accelerating) signal to Robot.")
                elif newSpeed < self.speed:
                    self.lblControlCommands.setText(Texts.TEXT_DECELERATING)
                    self.logToList("Sent (Decelerating) signal to Robot.")
            # Update runtime
            self.speed = newSpeed
            self.lcdSpeed.display(self.speed)

    def logToList(self, msg: str):
        # Append msg to log list
        self.listLog.addItem(f'[{datetime.strftime(datetime.now(), "%H:%M:%S")}] -> {msg}')
        # Scroll to list bottom
        self.listLog.scrollToBottom()

    def update_ui_to_connected(self):
        # Update GUI
        self.lcdSpeed.display("0")
        self.imgRTV.setEnabled(True)
        self.lcdSpeed.setEnabled(True)
        self.btnStartStopStream.setEnabled(True)
        self.btnSwitchControlMode.setEnabled(True)
        self.btnConnectDisconnect.setEnabled(True)
        self.btnConnectDisconnect.setText("Disconnect from Robot")
        self.lblControlCommands.setText(
            Texts.TEXT_IDLE
            if self.controlMode == ControlModes.CONTROL_MODE_MANUAL
            else Texts.TEXT_LFA_MODE_ENABLED
        )
        self.btnSwitchControlMode.setText(
            Texts.TEXT_SWITCH_TO_LFA_MODE
            if self.controlMode == ControlModes.CONTROL_MODE_MANUAL
            else Texts.TEXT_SWITCH_TO_MANUAL_MODE
        )

    def update_ui_to_disconnected(self):
        # Update texts
        self.lcdSpeed.display("--")
        self.btnConnectDisconnect.setText("Connect to Robot")
        self.btnStartStopStream.setText(Texts.TEXT_START_STREAM)
        self.lblControlCommands.setText(Texts.TEXT_NOT_CONNECTED)
        self.btnRecordStream.setText(Texts.TEXT_START_RECORDING_STREAM)
        # Update states
        self.imgRTV.setEnabled(False)
        self.lcdSpeed.setEnabled(False)
        self.btnRecordStream.setEnabled(False)
        self.btnStartStopStream.setEnabled(False)
        self.btnConnectDisconnect.setEnabled(True)
        self.btnSwitchControlMode.setEnabled(False)

    def connect_disconnect(self):
        if self.status == Status.CONNECTED:
            # Stop the stream worker
            if self.stream_worker.working:
                self.stream_worker.finish_work()
                self.logToList("Released Stream Worker successfully.")
            # Stop the data worker
            if self.data_worker.working:
                if self.data_worker.send(Signals.SIGNAL_DISCONNECT) > 0:
                    self.data_worker.finish_work()
                    self.update_ui_to_disconnected()
                    self.logToList("Released Data Worker successfully.")
        else:
            # Finish existing data worker
            if self.data_worker.working:
                self.data_worker.finish_work()
            # Start data worker
            self.data_worker.start_work()

    def switch_control_mode(self):
        # Send switch control mode signal to Robot
        if (self.data_worker.send(model2json(signal=Signals.SIGNAL_SWITCH_CONTROL_MODE, control_mode=get_switched_ctrl_mode(self.controlMode))) > 0):
            # Update GUI
            if self.controlMode == ControlModes.CONTROL_MODE_MANUAL:
                self.logToList("Switched to Automatic Control Mode.")
                self.lblControlCommands.setText(Texts.TEXT_LFA_MODE_ENABLED)
                self.btnSwitchControlMode.setText(Texts.TEXT_SWITCH_TO_MANUAL_MODE)
                self.controlMode = ControlModes.CONTROL_MODE_AUTOMATIC
            else:
                self.logToList("Switched to Manual Control Mode.")
                self.controlMode = ControlModes.CONTROL_MODE_MANUAL
                self.btnSwitchControlMode.setText(Texts.TEXT_SWITCH_TO_LFA_MODE)
                self.lblControlCommands.setText(Texts.TEXT_IDLE)
                self.lcdSpeed.display(0)

    def start_stop_stream(self):
        if self.stream_worker.working:
            # Stop stream
            if (self.data_worker.send(model2json(signal=Signals.SIGNAL_CLOSE_STREAM)) > 0):
                self.logToList("Sent (STOP STREAM) signal to Robot.")
                self.stream_worker.finish_work()
                self.btnStartStopStream.setText(Texts.TEXT_START_STREAM)
            else:
                self.logToList("Can't send (STOP STREAM) signal to Robot. Try again.")
        else:
            # Start stream
            if (self.data_worker.send(model2json(signal=Signals.SIGNAL_START_STREAM)) > 0):
                self.logToList("Sent (START STREAM) signal to Robot.")
                self.stream_worker.start_work()
                self.btnStartStopStream.setText(Texts.TEXT_STOP_STREAM)
            else:
                self.logToList("Can't send (START STREAM) signal to Robot. Try again.")

    def start_stop_record(self):
        if self.recording:
            self.stream_worker.recorder.stop_recording()
            self.btnRecordStream.setText(Texts.TEXT_START_RECORDING_STREAM)
        else:
            self.stream_worker.recorder.start_recording()
            self.btnRecordStream.setText(Texts.TEXT_STOP_RECORDING_STREAM)

    @property
    def recording(self):
        return self.stream_worker.recorder.recording

    def showHelp(self):
        # TODO: Show help window that contains keymap and how to deal with the program
        pass


def myExceptHook(type, value, tback):
    sys.__excepthook__(type, value, tback)


if __name__ == "__main__":
    # Hook my exception handler before anything
    sys.__excepthook__ = myExceptHook
    # Create and start execution of the WCU app
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    try:
        code = app.exec_()
        print(f"WCU finished with code: {code}")
        sys.exit(code)
    except Exception as e:
        print(f"Exiting after error occurred. Type[{type(e)}] MSG[{e}]")
