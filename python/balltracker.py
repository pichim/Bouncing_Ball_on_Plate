import threading
import queue
import numpy as np
from picamera2 import Picamera2
import cv2
ENABLE_WEB_STREAM = True  # <<< SET TO False TO DISABLE WEBSITE
if ENABLE_WEB_STREAM:
    from flask import Flask, Response
    import threading

class CameraProcessor:
    def __init__(self):

        # --- Web stream frame storage ---
        if ENABLE_WEB_STREAM:
            self.web_frame = None
            self.web_lock = threading.Lock()

        # --- Camera ---
        self.picam2 = Picamera2()
        self.picam2.post_callback = self._callback
            

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
    # Worker thread (ball detection)
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
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # range for pingpong ball
        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([25, 255, 255])

        # range for red massive ball
        #lower_orange = np.array([170, 120, 60])
        #upper_orange = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest)
            if radius > 5:
                center = (int(x), int(y))
                cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), 3)

            # Save frame for website (with drawings already on it)
            if ENABLE_WEB_STREAM:
                with self.web_lock:
                    self.web_frame = frame
        
        return x, y, radius



    # =========================================================
    # Public interface
    # =========================================================
    def start(self):

        
        config = self.picam2.create_video_configuration(
            controls={
                "FrameDurationLimits": (2000, 10000),
                "AeEnable": True,
                "ExposureTime": 1000
            }
        )
        self.picam2.configure(config)   
        self.picam2.start()
        if ENABLE_WEB_STREAM:
            start_web_server(self)
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
            

if ENABLE_WEB_STREAM:
    app = Flask(__name__)

    camera_instance = None  # will be assigned later

    @app.route('/')
    def index():
        return """
        <html>
            <head>
                <title>Ball Detection Stream</title>
            </head>
            <body>
                <h1>Ball Detection</h1>
                <img src="/video_feed">
            </body>
        </html>
        """

    def generate():
        global camera_instance
        while True:
            if camera_instance is None:
                continue

            with camera_instance.web_lock:
                frame = camera_instance.web_frame

            if frame is None:
                continue

            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' +
                   jpeg.tobytes() + b'\r\n')

    @app.route('/video_feed')
    def video_feed():
        return Response(generate(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    def start_web_server(cam):
        global camera_instance
        camera_instance = cam
        threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False),
            daemon=True
        ).start()

