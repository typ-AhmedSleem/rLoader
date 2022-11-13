from logging import INFO, Logger
import os
from typing import Iterator
import cv2 as cv
from vidgear.gears import WriteGear, CamGear, VideoGear


class RecorderException(Exception):
    pass


class StreamRecorder:

    def __init__(self, framerate=20.0, size: tuple[int, int] = (900, 600)) -> None:
        self.size = size
        self._writer: WriteGear | None = None
        self.recording_file = None
        self.framerate = framerate
        self.recording = False
        self.output_params = {
            "-vcodec": "libx264",
            "-crf": 0,
            "-preset": "medium",
            "-input_framerate": self.framerate,
            # "-output_dimensions": self.size,
            # "-thread_queue_size": "512",
            # "-ac": "2",
            # "-ar": "48000",
            # "-f": "alsa",
            # "-i": "hw:1",
        }

    def __enter__(self):
        self.start_recording()
        return self

    def __exit__(self, *args):
        self.stop_recording()

    def start_recording(self):
        # Quit invocation if already recording
        if self.recording:
            return
        if self._writer is None:
            # Create the folder if not exists
            if not os.path.exists(self.default_path):
                os.mkdir(self.default_path)
            # Create new video writer instance
            ext = 'mp4'
            filename = f"Recording {len([rec for rec in os.listdir(self.default_path) if rec.endswith(('.mp4'))]) +1}.{ext}"
            self.recording_file = os.path.join(self.default_path, filename)
            try:
                self._writer = WriteGear(output_filename=self.recording_file, logging=False, **self.output_params)
            except cv.error as e:
                raise RecorderException(f"Error creating video writer instance.. cv.error[{e}]")
            # Start recording
            self.recording = True

    def write_frame(self, frame):  # type: ignore
        if self.recording:
            if frame is not None:
                try:
                    # Write the frame
                    self._writer.write(frame)  # type: ignore
                except ValueError:
                    print("Writer is already closed.")

    def stop_recording(self) -> None:
        if not self.recording:
            return
        # Close the video writer
        self._writer.close()  # type: ignore
        # Reset runtime
        self._writer = None
        self.recording_file = None
        self.recording = False

    @property
    def default_path(self) -> str:
        return os.path.join(os.getcwd(), "recordings")


if __name__ == '__main__':
    framerate, width, height = 30.0, 1920, 1080
    # camera_options = {"CAP_PROP_FPS": framerate, "CAP_PROP_FRAME_WIDTH": width, "CAP_PROP_FRAME_HEIGHT": height}
    import time
    logger = Logger("StreamRecorder", INFO)
    try:
        logger.info('Initializing...')
        frame_count = 0
        camera = CamGear(logging=True)
        # camera = cv.VideoCapture(0)
        # camera.set(cv.CAP_PROP_FPS, framerate)
        # camera.set(cv.CAP_PROP_FRAME_WIDTH, width)
        # camera.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        logger.info("Camera instance was initialized successfully.")
        with StreamRecorder(framerate, (width, height)) as recorder:
            start = time.perf_counter()
            logger.info(f"Recorder started on file '{recorder.recording_file}'")
            logger.info('Recording...')
            while recorder.recording:
                try:
                    frame = camera.read()
                    if frame is None:
                        break
                    # Write the frame
                    recorder.write_frame(frame)
                    frame_count += 1
                    # Display the frame
                    cv.imshow("Recorder", frame)
                    key = cv.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break
                except KeyboardInterrupt:
                    logger.debug("Stopping record...")
                    break
            end = time.perf_counter()
            camera.stop()
            cv.destroyAllWindows()
            logger.error(f'Total of {frame_count} frame was written successfully.')
            logger.info(f"Saved recording of length '{int(end - start)} seconds' successfully. %s")
            logger.warning('Recorder stopped.')
    except KeyboardInterrupt:
        logger.critical("Pressed 'Ctrl + C' while initializing recorder instance.")
        logger.info("Recorder finished.")
