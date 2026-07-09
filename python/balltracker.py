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
    import threading

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
            

        # --- Frame buffer ---
        self.frame_q = queue.Queue(maxsize=1)

        # --- Shared state for main ---
        self.lock = threading.Lock()
        self.ball_pos = (0.0, 0.0, 0.0, 0.0)
        self.new_data = False

        # --- Worker thread ---
        self.running = False
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)

        # --- Camera calibration parameters ---
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


        BALANCE = 1.0  # 0.0=less FOV, 1.0=max FOV
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

            try:
                # Startzeit für die Performancemessung
                start_time = time.time()

                # --- METHODE: Zwei-Punkte-Entzerrung ---
                
                frame_process = frame
                x_distorted, y_distorted, r_dist = self.detect_ball(frame_process)
                # processing_time_ms = (time.time() - start_time) * 1000
                
                if r_dist > 5:

                    # 1. Optisches Zentrum des verzerrten Bildes
                    cx_dist = self.K[0, 2]
                    cy_dist = self.K[1, 2]

                    # 2. Vektor vom Bildzentrum zum Ball
                    dx = x_distorted - cx_dist
                    dy = y_distorted - cy_dist
                    dist_center = math.hypot(dx, dy)

                    if dist_center < 1.0:
                        # Ball ist exakt in der Mitte, Richtung ist egal
                        x_edge_dist = x_distorted + r_dist
                        y_edge_dist = y_distorted
                    else:
                        # 3. Tangentialvektor berechnen (90 Grad gedreht zum Radiusvektor)
                        # Normieren auf Länge 1
                        tan_x = -dy / dist_center
                        tan_y = dx / dist_center
                        
                        # 4. Randpunkt berechnen asu Ballmittelpunkt + (Tangentialvektor (normiert) * Radius)
                        x_edge_dist = x_distorted + (tan_x * r_dist)
                        y_edge_dist = y_distorted + (tan_y * r_dist)

                    # 5. Punkte für die Entzerrung übergeben
                    pts_distorted = np.array([[[x_distorted, y_distorted], 
                                            [x_edge_dist, y_edge_dist]]], dtype=np.float64)

                    
                    # Beide Punkte gleichzeitig entzerren!
                    pts_undistorted = cv2.fisheye.undistortPoints(
                        pts_distorted, self.K, self.D, P=self.new_K
                    )
                    
                    # Entzerrter Mittelpunkt
                    x_px = pts_undistorted[0][0][0]
                    y_px = pts_undistorted[0][0][1]
                    # print(f"undistorted center: ({x_px:.1f}, {y_px:.1f})")
                    
                    # Entzerrter Randpunkt
                    x_edge = pts_undistorted[0][1][0]
                    y_edge = pts_undistorted[0][1][1]

                    # Parameter der Kameramatrix    m_trajectory.setCircle(50.0f, 0.5f);
                    fx = self.new_K[0, 0]
                    fy = self.new_K[1, 1]
                    cx = self.new_K[0, 2]
                    cy = self.new_K[1, 2]

                    R_real = 20.0  # mm

                    # 1. 3D-Richtungsvektoren (Z = 1.0 Ebene)
                    # Vektor zum Zentrum des Balls
                    v_center = np.array([(x_px - cx) / fx, (y_px - cy) / fy, 1.0])
                    # Vektor zur Außenkante des Balls
                    v_edge = np.array([(x_edge - cx) / fx, (y_edge - cy) / fy, 1.0])

                    # 2. Vektoren normieren
                    norm_center = np.linalg.norm(v_center)
                    norm_edge = np.linalg.norm(v_edge)
                    v_center_norm = v_center / norm_center
                    v_edge_norm = v_edge / norm_edge

                    # 3. 3D-Winkel zwischen center und rand-vektor berechnen
                    cos_alpha = np.clip(np.dot(v_center_norm, v_edge_norm), -1.0, 1.0)
                    alpha = math.acos(cos_alpha)

                    if alpha > 0:
                        # 4. Die direkte Luftlinie (D) berechnen
                        # Geometrie der Kugel: sin(Sichtwinkel) = Radius / Distanz
                        D = R_real / math.sin(alpha)

                        # 5. X, Y, Z berechnen
                        # v_center_norm zeigt in die Richtung des Balls, D gibt an, wie weit weg er ist.
                        X = D * v_center_norm[0]
                        Y = D * v_center_norm[1]
                        Z = D * v_center_norm[2]

                        # 6. Master-Offset: Optisches Zentrum direkt zum mechanischen Zentrum
                        # Plattenmitte (681, 505) - Abstand zum optischen Nullpunkt der Kamera-Matrix.
                            
                        Zref = 185.7  # mm, Referenzhöhe für die Offset-Kompensation (z.B. Höhe der Platte)
                        X -= (669 - self.new_K[0, 2]) * Zref / fx
                        Y -= (509 - self.new_K[1, 2]) * Zref / fy

                        # --- Start der Z-Kompensation (Entzerrung) ---
                        # 1-3 auskommentierung if entzerrung nicht gewunscht
                        # 1. Koeffizienten aus dem MATLAB-Fit
                        c0 = 185.7
                        c1 = -0.0025
                        c2 = 0.0137

                        # 2. Berechne die erwartete Wölbung (das Modell) an der aktuellen X, Y Position
                        # Achtung: Python nutzt ** für Potenzen
                        z_modell = c0 + c1 * X + c2 * Y


                        # 3. Z korrigieren: Gemessener Wert minus die "Beule" plus Referenzhöhe (c0)
                        Z = Z - (z_modell)

                        processing_time_ms = (time.time() - start_time) * 1000
                        
                        with self.lock:
                            self.ball_pos = (X, Y, Z, processing_time_ms)
                            self.new_data = True
                        
                
        
            except Exception as e:
                print(f"error in WORKER-THREAD: {e}")
                time.sleep(0.1) # Kurze Pause, um Log-Spam zu verhindern


    # =========================================================
    # Ball detection
    # =========================================================
    def detect_ball(self, frame: np.ndarray):
        x = y = radius = 0
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # TE 616 (neutral)
        lower_orange = np.array([10, 150, 100])
        upper_orange = np.array([20, 255, 255])

        # TS 0.12 (hell)
        # lower_orange = np.array([2, 150, 50])
        # upper_orange = np.array([12, 255, 200])

        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)

            # circle detection
            ((x, y), radius) = cv2.minEnclosingCircle(largest)


            if radius > 5:
                center = (int(x), int(y))
                cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), 3)

            # elipse detection
            # if len(largest) >= 5:
            #     ellipse = cv2.fitEllipse(largest)
            #     # ellipse liefert: Zentrum(x,y), Achsen(breite, höhe), Rotationswinkel
            #     (x, y), (width, height), angle = ellipse
                
            #     # Der "stabile" Radius ist die Hälfte der kürzeren Achse (Minor Axis)
            #     radius = min(width, height) / 2.0
                
            #     if radius > 5:
            #         center = (int(x), int(y))
            #         # Zeichnet die exakte Ellipse (sieht in der Präsentation super aus!)
            #         cv2.ellipse(frame, ellipse, (0, 255, 0), 2) 
            #         cv2.circle(frame, center, 2, (0, 0, 255), 3) # Mittelpunkt
        
        # Höhe und Breite des Frames abfragen, Zentrum berechnen
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        # print(f"Breite: {w}, cx: {cx}")
        # print(f"Höhe: {h}, cy: {cy}")

        new_cx = int(self.new_K[0, 2])
        new_cy = int(self.new_K[1, 2])

        # Rotes Kreuz (+) im Bildzentrum einzeichnen
        # cv2.drawMarker(frame, (new_cx, new_cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        # cv2.drawMarker(frame, (cx, cy), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        # cv2.drawMarker(frame, (683, 505), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # Save frame for website (with drawings already on it)
        if ENABLE_WEB_STREAM:
            with self.web_lock:
                self.web_frame = frame
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
                "ExposureTime": 8000,
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
                    time.sleep(0.1)
                    continue

                # wait for new frame
                if not camera_instance.new_frame_event.wait(timeout=1.0):
                    continue
                
                # reset flag
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