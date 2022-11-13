from io import BytesIO
import pickle
import struct
from sockets import ClientSocket
from time import sleep
from typing import Tuple
import cv2 as cv
import numpy as np
from robot_utils import *
from threading import Lock

CAMERA_SOURCE = 0
VIDEO_SOURCE = 'E:\\Development\\Robotics\\rLoader\\WCU\\data\\Summon.mp4'
DEFAULT_FRAME_SIZE = (900, 600)


class VideoCapture:

    def __init__(self, stream_src: int | str, size: Tuple[int, int] = DEFAULT_FRAME_SIZE) -> None:
        self.cap = cv.VideoCapture(stream_src, cv.CAP_DSHOW)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, size[0])
        self.cap.set(cv.CAP_PROP_FPS, 60)

    @property
    def is_opened(self) -> bool:
        return self.cap.isOpened()

    def read(self) -> Tuple[bool, cv.Mat]:
        return self.cap.read()

    def release(self):
        self.cap.release()


class BaseCamera:
    """ Base class for camera which extends to 'cvCamera' & 'PiCamera' """

    def __init__(self, stream_src: str | int, stream_handler) -> None:
        # Setup runtime
        self.initialized = False
        self.streaming = False
        self.stream_src = stream_src
        self.stream_handler = stream_handler
        self.cap: VideoCapture = None  # type: ignore
        self.lock = Lock()

    def __enter__(self):
        self.start()

    def __exit__(self):
        self.stop()

    # def setup(self):
    #     """ Setup the stream runtime making to ready for streaming """
    #     pass

    def start(self):
        """ Setup the stream runtime making to ready for streaming (if needed) then Stream it """
        pass

    def stop(self):
        """ Stop currently running stream and release its resources """
        pass

    def grab_frame(self) -> Tuple[bool, cv.Mat]:
        """ Reads the next frame from the opened video capture

        Returns:
            >>> Tuple[bool, cv.Mat]: ('ret', 'frame') grabbed from video capture.
        """
        return (False, None)  # type: ignore

    def switch_source(self, stream_src: str | int) -> bool:
        """ Switch the stream source to the new one then setup it

        Args:
            stream_src (str | int): Stream source representing video file path or camera index

        Returns:
            >>> bool: True if switched else False
        """
        return False

    def start_preview(self):
        pass


class ocvCamera(BaseCamera):

    def __init__(self, stream_src: str | int, stream_handler=None) -> None:
        super().__init__(stream_src, stream_handler)

    def start(self):
        if self.streaming:
            print('[ocvCamera]: Already streaming.')
            return
        # Release lock if already acquired
        if self.lock.locked():
            self.lock.release()
        if not self.initialized:
            # Log
            print('[ocvCamera]: Starting stream...')
            # Lock current thread
            self.lock.acquire()
            # Create the video capture
            self.cap = VideoCapture(self.stream_src)
            self.streaming = True
            self.initialized = True
            # Log
            print('[ocvCamera]: Stream started.')
        else:
            self.streaming = True
            print('[ocvCamera]: Stream is already initialized.')
        # Release the lock
        self.lock.release()

    def stop(self):
        if not self.streaming:
            print(f'[ocvCamera]: Stream hasn\'t started yet to stop it.')
            return
        self.streaming = False
        self.initialized = False
        if self.cap:
            self.cap.release()
            cv.destroyAllWindows()
            print('[ocvCamera]: Released video stream resources.')

    def grab_frame(self) -> Tuple[bool, cv.Mat]:
        return self.cap.read() if self.cap else (False, None)  # type: ignore

    def switch_source(self, stream_src: int | str) -> bool:
        # Ignore if same stream is to be switched
        if self.stream_src == stream_src:
            return True
        # Switch the stream
        try:
            if self.cap and self.cap.is_opened:
                self.cap.release()
            self.cap = VideoCapture(stream_src)
            print(f'[ocvCamera]: Switched source from [{self.stream_src}] to [{stream_src}]')
            # Update runtime
            self.stream_src = stream_src
            return True
        except:
            return False


class PiCamera():

    def __init__(self, resolution=(640, 480), framerate=30) -> None:
        from vidgear.gears import PiGear
        # Camera attributes
        self.framerate = framerate
        self.resolution = resolution
        # Runtime vars
        self.streaming = False
        # Setup camera instance
        self.__camera = PiGear(resolution=resolution, framerate=framerate)

    def start(self):
        if not self.streaming:
            self.streaming = True
            self.__camera.start()

    def stop(self):
        if self.streaming:
            self.__camera.stop()
            self.streaming = False

    def grab_frame(self):
        return self.__camera.read() if self.streaming else None


class Streamer:

    def __init__(self, source: str | int = CAMERA_SOURCE, socket: ClientSocket = None) -> None:  # type: ignore
        # Runtime
        self.socket = socket
        self.camera = ocvCamera(source)

    def start_stream(self):
        # Check if socket is None
        if self.socket is None:
            print('[Streamer]: Stream socket is None.')
            return
        # Setup camera stream
        self.camera.start()
        # Start stream
        while self.streaming:
            stream_finished, frame = self.camera.grab_frame()
            if stream_finished:
                # Handle the grabbed frame
                frame = cv.resize(frame, DEFAULT_FRAME_SIZE)
                # Convert the frame unto numpy array
                frame = np.array(frame)
                # Pickle the frame into bytes
                frame_bytes = pickle.dumps(frame)
                # Append the frame size at the very beginning of frame bytes
                frame_bytes = struct.pack('L', len(frame_bytes)) + frame_bytes
                try:
                    # Send the frame to WCU
                    frame_total_size = self.socket.send(frame_bytes)
                    print(f'Sent frame of size {round(frame_total_size / 1024)} KBs')
                except (ConnectionAbortedError, ConnectionRefusedError, ConnectionResetError, ConnectionError, AttributeError) as e:
                    print(f'[Streamer]: Lost connection with WCU.')
                    self.stop_stream()
                    break
                except OSError as e:
                    print(f'[Streamer]: {type(e).__name__} | {e.args[1]}')
                    self.stop_stream()
                    break
            else:
                print('[Streamer]: Stream video has reached its end.')
                self.stop_stream()
                break

    def stop_stream(self):
        # Close stream socket
        self.socket.close()
        # Stop camera
        self.camera.stop()

    @property
    def streaming(self):
        return self.camera.streaming and (self.socket and self.socket.opened)
