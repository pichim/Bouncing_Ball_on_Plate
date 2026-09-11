import threading
import queue
import numpy as np
import time
from picamera2 import Picamera2
import cv2
import math


ENABLE_WEB_STREAM = True  # <<< SET TO False TO DISABLE WEBSITE

if ENABLE_WEB_STREAM:
    from flask import Flask, Response

class CameraProcessor:
    def __init__(self):
        # --- Web stream frame storage ---
        if ENABLE_WEB_STREAM:
            self.web_frame = None
            self.web_lock = threading.Lock()
            self.new_frame_event = threading.Event()

        # --- Camera ---
        self.picam2 = Picamera2()
        self.picam2.post_callback = self._callback
            
        # --- Frame buffer (latest only) ---
        self.frame_q = queue.Queue(maxsize=1)

        # --- Shared state for main ---
        self.lock = threading.Lock()
        self.ball_pos = (0.0, 0.0, 0.0, 0.0)
        self.new_data = False
        
        # =======================================================
        # --- EINSTELLUNGEN FÜR STROBOSKOP (HIER ANPASSEN!) ---
        # =======================================================
        self.RECORDING_DURATION = 1.4 - 0.0 # Dauer einer kompletten Aufnahme in Sekunden
        self.STAMP_INTERVAL_MS = 60.0  # Alle wie viel Millisekunden wird gestempelt? (z.B. 60ms)
        
        self.composite_canvas = None
        self.is_recording = False
        self.recording_start_time = 0.0
        self.last_stamp_time = 0.0

        # --- Worker thread ---
        self.running = False
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)

        self.K = np.array([
        [914.91763, 0.0, 663.41604981],
        [0.0, 917.4751116, 526.47839392],
        [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.D = np.array([
            [0.01584966],
            [0.01778682],
            [-0.14639213],
            [0.24211901]
        ], dtype=np.float64)

        BALANCE = 1.0  
        h = 1088
        w = 1456

        # --- build undistort maps ---
        self.new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K, self.D, (w, h), np.eye(3), balance=BALANCE
        )
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), self.new_K, (w, h), cv2.CV_16SC2
        )

    # =========================================================
    # Camera callback
    # =========================================================
    def _callback(self, request):
        frame = request.make_array("main")
        frame_copy = frame.copy()
        
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
        FULL_FRAME_UNDISTORT = False

        while self.running:
            try:
                frame = self.frame_q.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                start_time = time.time()

                if FULL_FRAME_UNDISTORT:
                    frame_process = cv2.remap(frame, self.map1, self.map2, 
                                            interpolation=cv2.INTER_LINEAR, 
                                            borderMode=cv2.BORDER_CONSTANT)
                    
                    x_px, y_px, r = self.detect_ball(frame_process)
                    f_avg = (self.new_K[0, 0] + self.new_K[1, 1]) / 2.0
                    x_distorted, y_distorted, r_dist = x_px, y_px, r

                else:
                    frame_process = frame
                    x_distorted, y_distorted, r_dist = self.detect_ball(frame_process)
                    
                    if r_dist > 5:
                        cx_dist = self.K[0, 2]
                        cy_dist = self.K[1, 2]

                        dx = x_distorted - cx_dist
                        dy = y_distorted - cy_dist
                        dist_center = math.hypot(dx, dy)

                        if dist_center < 1.0:
                            x_edge_dist = x_distorted + r_dist
                            y_edge_dist = y_distorted
                        else:
                            tan_x = -dy / dist_center
                            tan_y = dx / dist_center
                            x_edge_dist = x_distorted + (tan_x * r_dist)
                            y_edge_dist = y_distorted + (tan_y * r_dist)

                        pts_distorted = np.array([[[x_distorted, y_distorted], 
                                                [x_edge_dist, y_edge_dist]]], dtype=np.float64)

                        pts_undistorted = cv2.fisheye.undistortPoints(
                            pts_distorted, self.K, self.D, P=self.new_K
                        )
                        
                        x_px = pts_undistorted[0][0][0]
                        y_px = pts_undistorted[0][0][1]
                        x_edge = pts_undistorted[0][1][0]
                        y_edge = pts_undistorted[0][1][1]
                        
                processing_time_ms = (time.time() - start_time) * 1000

                # --- BERECHNUNG ---
                if r_dist > 5:
                    fx = self.new_K[0, 0]
                    fy = self.new_K[1, 1]
                    cx = self.new_K[0, 2]
                    cy = self.new_K[1, 2]

                    R_real = 20.0  # mm

                    v_center = np.array([(x_px - cx) / fx, (y_px - cy) / fy, 1.0])
                    v_edge = np.array([(x_edge - cx) / fx, (y_edge - cy) / fy, 1.0])

                    norm_center = np.linalg.norm(v_center)
                    norm_edge = np.linalg.norm(v_edge)
                    v_center_norm = v_center / norm_center
                    v_edge_norm = v_edge / norm_edge

                    cos_alpha = np.clip(np.dot(v_center_norm, v_edge_norm), -1.0, 1.0)
                    alpha = math.acos(cos_alpha)

                    if alpha > 0:
                        D = R_real / math.sin(alpha)
                        X = D * v_center_norm[0]
                        Y = D * v_center_norm[1]
                        Z = D * v_center_norm[2]

                        Zref = 185.7
                        X -= (669 - self.new_K[0, 2]) * Zref / fx
                        Y -= (509 - self.new_K[1, 2]) * Zref / fy

                        c0 = 185.7
                        c1 = -0.0025
                        c2 = 0.0137

                        z_modell = c0 + c1 * X + c2 * Y
                        Z = Z - (z_modell)

                    with self.lock:
                        self.ball_pos = (X, Y, Z, processing_time_ms)
                        self.new_data = True

            except Exception as e:
                print(f"error in WORKER-THREAD: {e}")
                time.sleep(0.1)

    # =========================================================
    # Ball detection & Zeitbasiertes Stroboskop
    # =========================================================
    def detect_ball(self, frame: np.ndarray):
        x = y = radius = 0
        current_time = time.time()
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2HSV)

        # light threshold
        lower_orange = np.array([8, 130, 50])
        upper_orange = np.array([18, 255, 170])   

        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # --- TIMER-LOGIK START ---
        if not self.is_recording:
            self.is_recording = True
            self.recording_start_time = current_time
            self.last_stamp_time = current_time
            # Weißes Canvas vorbereiten
            self.composite_canvas = np.full_like(frame_rgb, 255)
            print(f"Starte neue Aufzeichnung für {self.RECORDING_DURATION} Sekunden...")

        if contours:
            largest = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest)

            if radius > 5:
                center = (int(x), int(y))
                
                # --- INTERVALL-PRÜFUNG ---
                time_since_last_stamp_ms = (current_time - self.last_stamp_time) * 1000
                
                if time_since_last_stamp_ms >= self.STAMP_INTERVAL_MS:
                    self.last_stamp_time = current_time
                    
                    # --- SAUBERE KREIS-MASKE ERSTELLEN ---
                    circle_mask = np.zeros(frame_rgb.shape[:2], dtype=np.uint8)
                    
                    # Wir machen den Radius minimal kleiner (90%), 
                    # um wirklich nur den Ball und keinen Hintergrund am Rand zu stempeln
                    draw_radius = int(radius * 0.9) 
                    
                    cv2.circle(circle_mask, center, draw_radius, 255, -1)
                    
                    # Kopiere die Pixel durch die perfekte Kreis-Maske auf das Canvas
                    self.composite_canvas[circle_mask > 0] = frame_rgb[circle_mask > 0]

        # --- TIMER-LOGIK ENDE (BILD SPEICHERN) ---
        if (current_time - self.recording_start_time) >= self.RECORDING_DURATION:
            import os
            
            # 1. Definiere den Zielordner (hier: ein Unterordner namens 'aufnahmen')
            base_dir = os.path.dirname(os.path.abspath(__file__))
            save_dir = os.path.join(base_dir, "aufnahmen")
            
            # 2. Erstelle den Ordner, falls er noch nicht existiert
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            # 3. Dateiname zusammenbauen
            filename = f"stroboskop_{int(self.RECORDING_DURATION)}s_{time.strftime('%Y%m%d-%H%M%S')}.jpg"
            full_path = os.path.join(save_dir, filename)
            
            try:
                success = cv2.imwrite(full_path, self.composite_canvas)
                if success:
                    print(f"!!! ERFOLG: Bild gespeichert unter: {full_path}")
                else:
                    print("!!! FEHLER: cv2.imwrite hat False zurückgegeben!")
            except Exception as e:
                print(f"!!! FEHLER beim Speichern: {e}")
            
            # Reset
            self.is_recording = False

        # --- BILD FÜR DEN WEBSTREAM VORBEREITEN ---
        # Zeige immer das aktuelle Stroboskop-Canvas im Web-Stream an
        if self.composite_canvas is not None:
            display_frame = self.composite_canvas.copy()
        else:
            display_frame = frame_rgb.copy()

        # Save frame for website
        if ENABLE_WEB_STREAM:
            with self.web_lock:
                self.web_frame = display_frame
            self.new_frame_event.set()
            
        return x, y, radius

    # =========================================================
    # Public interface
    # =========================================================
    def start(self):
        config = self.picam2.create_video_configuration(
            main={"size": (1456, 1088)},
            controls={
                "FrameDurationLimits": (20000, 20000),
                "AeEnable": False,
                "ExposureTime": 14000,
                "AnalogueGain": 1.0
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
        with self.lock:
            if self.new_data:
                pos = self.ball_pos
                self.new_data = False
                return pos
            else:
                return None


# =========================================================
# Web Server Setup
# =========================================================
if ENABLE_WEB_STREAM:
    app = Flask(__name__)

    camera_instance = None 

    @app.route('/')
    def index():
        return """
        <html>
            <head>
                <title>Stroboskop Stream</title>
                <style>
                    body { background-color: #111; color: white; text-align: center; font-family: sans-serif; }
                    img { max-width: 100%; border: 2px solid #444; }
                </style>
            </head>
            <body>
                <h1>Live Stroboskop Rendering (Zeitbasiert)</h1>
                <img src="/video_feed">
            </body>
        </html>
        """

    def generate():
            global camera_instance
            while True:
                if camera_instance is None:
                    time.sleep(0.1)
                    continue

                if not camera_instance.new_frame_event.wait(timeout=1.0):
                    continue
                
                camera_instance.new_frame_event.clear()

                with camera_instance.web_lock:
                    frame = camera_instance.web_frame

                if frame is None:
                    continue

                ret, jpeg = cv2.imencode('.jpg', frame)
                if not ret:
                    continue

                try:
                    yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' +
                            jpeg.tobytes() + b'\r\n')
                except Exception as e:
                    print(f"Web-Client getrennt oder Fehler: {e}")
                    break

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