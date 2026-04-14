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
            

        # --- Frame buffer (latest only) ---
        self.frame_q = queue.Queue(maxsize=1)

        # --- Shared state for main ---
        self.lock = threading.Lock()
        self.ball_pos = (0.0, 0.0, 0.0, 0.0)
        self.new_data = False

        # --- Worker thread ---
        self.running = False
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)

        # camera distortion
        # only used if K was made with other frame sizes   
        # def scale_intrinsics(K, old_size, new_size):
        #     old_w, old_h = old_size
        #     new_w, new_h = new_size
        #     sx = new_w / old_w
        #     sy = new_h / old_h
        #     K_scaled = K.copy()
        #     K_scaled[0, 0] *= sx  # fx
        #     K_scaled[1, 1] *= sy  # fy
        #     K_scaled[0, 2] *= sx  # cx
        #     K_scaled[1, 2] *= sy  # cy
        #     return K_scaled

        # # --- Fisheye calibration (your working values) ---
        # K_old = np.array([
        #     [410.17747674, 0.0, 299.96826545],
        #     [0.0, 409.32732313, 219.99535070],
        #     [0.0, 0.0, 1.0]
        # ], dtype=np.float64)

        # D = np.array([
        #     [0.01534284],
        #     [-0.01886187],
        #     [0.01338572],
        #     [0.02682248]
        # ], dtype=np.float64)

        # K = scale_intrinsics(K_old, (640, 480), (1456, 1088))

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

        # self.K = np.array([
        #     [937.29215827, 0.0, 680.93444222],
        #     [0.0, 937.69688378, 506.66778474],
        #     [0.0, 0.0, 1.0]
        # ], dtype=np.float64)

        # self.D = np.array([
        #     [0.02105106],
        #     [-0.09963138],
        #     [0.46593145],
        #     [-0.76069309]
        # ], dtype=np.float64)


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
        
        # --- SCHALTER ZUM TESTEN ---
        # True  = Ganzes Bild entzerren (Hohe Präzision, langsam)
        # False = Nur den Punkt entzerren (Schnell, am Rand ungenau für Z)
        FULL_FRAME_UNDISTORT = False

        while self.running:
            try:
                frame = self.frame_q.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                # Startzeit für die Performancemessung
                start_time = time.time()

                if FULL_FRAME_UNDISTORT:
                    # --- METHODE 1: Vollbild-Entzerrung ---
                    # Das ganze Bild geradeziehen, bevor der Ball gesucht wird
                    frame_process = cv2.remap(frame, self.map1, self.map2, 
                                            interpolation=cv2.INTER_LINEAR, 
                                            borderMode=cv2.BORDER_CONSTANT)
                    
                    # Ball im bereits perfekten Bild suchen
                    x_px, y_px, r = self.detect_ball(frame_process)
                    f_avg = (self.new_K[0, 0] + self.new_K[1, 1]) / 2.0


                else:
                    # --- METHODE 2: Zwei-Punkte-Entzerrung ---
                    frame_process = frame
                    x_distorted, y_distorted, r_dist = self.detect_ball(frame_process)
                    
                    if r_dist > 5:
                        # Wir definieren zwei Punkte im VERZERRTEN Bild:
                        # 1. Den Mittelpunkt
                        # 2. Einen Punkt am rechten Rand des Balls
                        # pts_distorted = np.array([[[x_distorted, y_distorted], 
                        #                            [x_distorted + r_dist, y_distorted]]], dtype=np.float64)

                        # test
                        # 1. Optisches Zentrum des verzerrten Bildes (aus deiner K-Matrix)
                        cx_dist = self.K[0, 2]
                        cy_dist = self.K[1, 2]

                        # 2. Vektor vom Bildzentrum zum Ball berechnen
                        dx = x_distorted - cx_dist
                        dy = y_distorted - cy_dist
                        dist_center = math.hypot(dx, dy)

                        if dist_center < 1.0:
                            # Ball ist exakt in der Mitte, Richtung ist egal
                            x_edge_dist = x_distorted + r_dist
                            y_edge_dist = y_distorted
                        else:
                            # 3. Tangentialvektor berechnen (90 Grad gedreht zum Radiusvektor)
                            # Normieren auf Länge 1, dann mit r_dist multiplizieren
                            tan_x = -dy / dist_center
                            tan_y = dx / dist_center
                            
                            # 4. Den korrekten Randpunkt setzen
                            x_edge_dist = x_distorted + (tan_x * r_dist)
                            y_edge_dist = y_distorted + (tan_y * r_dist)

                        # 5. Punkte für die Entzerrung übergeben
                        pts_distorted = np.array([[[x_distorted, y_distorted], 
                                                [x_edge_dist, y_edge_dist]]], dtype=np.float64)
                        # test
                        
                        # Beide Punkte gleichzeitig entzerren!
                        pts_undistorted = cv2.fisheye.undistortPoints(
                            pts_distorted, self.K, self.D, P=self.new_K
                        )
                        
                        # Entzerrter Mittelpunkt
                        x_px = pts_undistorted[0][0][0]
                        y_px = pts_undistorted[0][0][1]
                        
                        # Entzerrter Randpunkt
                        x_edge = pts_undistorted[0][1][0]
                        y_edge = pts_undistorted[0][1][1]
                        

                # --- BERECHNUNG ---
                if r_dist > 5:
                    # Parameter der NEUEN Kameramatrix
                    fx = self.new_K[0, 0]
                    fy = self.new_K[1, 1]
                    cx = self.new_K[0, 2]
                    cy = self.new_K[1, 2]

                    R_real = 20.0  # mm

                    # 1. Wir bauen zwei 3D-Richtungsvektoren (Z = 1.0 Ebene)
                    # Vektor zum Zentrum des Balls
                    v_center = np.array([(x_px - cx) / fx, (y_px - cy) / fy, 1.0])
                    # Vektor zur Außenkante des Balls
                    v_edge = np.array([(x_edge - cx) / fx, (y_edge - cy) / fy, 1.0])

                    # 2. Vektoren normieren (Länge auf exakt 1.0 setzen)
                    norm_center = np.linalg.norm(v_center)
                    norm_edge = np.linalg.norm(v_edge)
                    v_center_norm = v_center / norm_center
                    v_edge_norm = v_edge / norm_edge

                    # 3. Den echten 3D-Winkel zwischen Zentrum und Kante berechnen
                    # Das ist der mathematisch perfekte Sichtwinkel, unbeeindruckt von Pixel-Streckungen!
                    # np.clip sichert uns gegen Float-Rundungsfehler ab
                    cos_alpha = np.clip(np.dot(v_center_norm, v_edge_norm), -1.0, 1.0)
                    alpha = math.acos(cos_alpha)

                    if alpha > 0:
                        # 4. Die direkte Luftlinie (D) berechnen
                        # Geometrie der Kugel: sin(Sichtwinkel) = Radius / Distanz
                        D = R_real / math.sin(alpha)

                        # 5. X, Y, Z berechnen
                        # Da unser Vektor 'v_center_norm' die Richtung vorgibt und die Länge 1 hat,
                        # müssen wir ihn nur noch mit der Distanz D "langziehen".
                        X = D * v_center_norm[0]
                        Y = D * v_center_norm[1]
                        Z = D * v_center_norm[2]  # Das ist jetzt ein absolut stabiles Z!


                    
                # else:
                #     # --- METHODE 2: Punkt-Entzerrung ---
                #     frame_process = frame
                #     x_distorted, y_distorted, r = self.detect_ball(frame_process)
                    
                #     if r > 5:
                #         pt = np.array([[[x_distorted, y_distorted]]], dtype=np.float64)
                #         undistorted_pt = cv2.fisheye.undistortPoints(
                #             pt, self.K, self.D, P=self.new_K
                #         )
                #         x_px = undistorted_pt[0][0][0]
                #         y_px = undistorted_pt[0][0][1]
                #         f_avg = (self.K[0, 0] + self.K[1, 1]) / 2.0


                # --- BERECHNUNG (für beide Methoden gleich) ---
                # if r > 5:  # valid detection
                #     # Parameter aus der NEUEN Kameramatrix auslesen
                #     fx = self.new_K[0, 0]
                #     fy = self.new_K[1, 1]
                #     cx = self.new_K[0, 2]
                #     cy = self.new_K[1, 2]

                #     R_real = 20.0  # mm
                    

                #     # 3D-Position berechnen
                #     Z = (R_real * f_avg) / r 
                #     X = (x_px - cx) * Z / fx
                #     Y = (y_px - cy) * Z / fy

                    # Zeitmessung abschließen
                    processing_time_ms = (time.time() - start_time) * 1000

                    # print(f"Modus: {'Vollbild' if FULL_FRAME_UNDISTORT else 'Punkt'} | "
                    #     f"Zeit: {processing_time_ms:.1f} ms | "
                    #     f"Radius: {r_dist:.2f} px | Distanz Z: {Z:.1f} mm")

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

        # range for pingpong ball
        # lower_orange = np.array([5, 150, 150])
        # upper_orange = np.array([25, 255, 255])

        # schablone
        # lower_orange = np.array([9, 220, 70])
        # upper_orange = np.array([15, 255, 120])

        # lower_orange = np.array([8, 170, 50])
        # upper_orange = np.array([18, 255, 170])

        # good values
        # lower_orange = np.array([10, 150, 100])
        # upper_orange = np.array([20, 255, 255])

        lower_orange = np.array([8, 130, 80])
        upper_orange = np.array([18, 255, 170])      

        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest)
            if radius > 5:
                center = (int(x), int(y))
                cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                cv2.circle(frame, center, 2, (0, 0, 255), 3)
        
        # Höhe und Breite des Frames abfragen, Zentrum berechnen
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        new_cx = int(self.new_K[0, 2])
        new_cy = int(self.new_K[1, 2])

        # Rotes Kreuz (+) im Bildzentrum einzeichnen
        cv2.drawMarker(frame, (new_cx, new_cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

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