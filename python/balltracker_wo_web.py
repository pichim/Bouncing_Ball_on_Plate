import threading
import queue
import numpy as np
from picamera2 import Picamera2
import cv2
from GSCrop import set_camera_crop


class CameraProcessor:
    def __init__(self, width=1000, height=1000):

        # --- Camera ---
        self.picam2 = Picamera2()
        self.picam2.post_callback = self._callback
        self.width = width
        self.height = height

             

        # --- Frame buffer (latest only) ---
        self.frame_q = queue.Queue(maxsize=1)

        # --- Shared state for main ---
        self.lock = threading.Lock()
        self.ball_pos = (0.0, 0.0)
        self.new_data = False

        # --- Worker thread ---
        self.running = False
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)

    # =========================================================
    # Camera callback
    # =========================================================
    def _callback(self, request):

        frame = request.make_array("main")
        frame_copy = frame.copy()

        # Keep only latest frame
        try:
            self.frame_q.get_nowait()
        except queue.Empty:
            pass

        try:
            self.frame_q.put_nowait(frame_copy)
        except queue.Full:
            pass

    # =========================================================
    # Worker thread (ball detection happens here)
    # =========================================================
    def _worker_loop(self):

        while self.running:
            try:
                frame = self.frame_q.get(timeout=0.1)
            except queue.Empty:
                continue

            x, y, r = self.detect_ball(frame)

            with self.lock:
                self.ball_pos = (x, y, r)
                self.new_data = True

    # =========================================================
    # Ball detection
    # =========================================================
    def detect_ball(self, frame: np.ndarray):
        x = y = radius = 0
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest)
            if radius > 5:
                center = (int(x), int(y))
                cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), 3)
        
        return x, y, radius



    # =========================================================
    # Public interface
    # =========================================================
    def start(self):
        # --- crop ---
        set_camera_crop(self.width, self.height)
        
        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height)},
            controls={
                "FrameDurationLimits": (2000, 2000),
                "ExposureTime": 1000
            }
        )
        #comment out for cropping
        self.picam2.configure(config)   
        self.picam2.start()
        self.running = True
        self.worker.start()

    def stop(self):
        self.running = False
        self.worker.join(timeout=1.0)
        self.picam2.stop()

    def get_ball_position(self):
        """
        Called from main loop.
        Returns (x, y) if new data available,
        otherwise returns None.
        """
        with self.lock:
            if self.new_data:
                pos = self.ball_pos
                self.new_data = False
                return pos
            else:
                return None