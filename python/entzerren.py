import argparse
import os
import threading
import time
from http import server
from socketserver import ThreadingMixIn

import cv2
import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None


# ============================================================
# FISHEYE-KALIBRIERDATEN
# ============================================================

# Diese Auflösung muss der Auflösung deiner Kalibrierbilder entsprechen.
# Ich nehme hier 640x480 an, weil cx~300 und cy~220 dazu gut passen.
# Falls deine Chessboard-Bilder eine andere Auflösung hatten:
# -> NUR diese Zeile anpassen.
CALIB_DIM = (1456, 1088)  # (width, height)

# Fisheye-Kameramatrix
K = np.array(
    [
        [914.91763, 0.0, 663.41604981],
        [0.0, 917.4751116, 526.47839392],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

# Fisheye-Distortion-Koeffizienten: [k1, k2, k3, k4]
D = np.array(
    [
        [0.01584966],
        [0.01778682],
        [-0.14639213],
        [0.24211901],
    ],
    dtype=np.float64,
)

# balance:
# 0.0 = weniger Sichtfeld, meist natürlicher / weniger Stretching
# 0.3 = etwas mehr Sichtfeld
# 1.0 = maximales Sichtfeld, aber oft stark verzogen / gestreckt
BALANCE = 1.0


def validate_calibration_fisheye(k_matrix: np.ndarray, dist_coeffs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Validiert und normalisiert K und D fuer OpenCV-Fisheye."""
    k = np.asarray(k_matrix, dtype=np.float64)
    if k.shape != (3, 3):
        raise ValueError(f"K muss die Form (3, 3) haben, ist aber {k.shape}")

    dvec = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1)
    if dvec.shape != (4, 1):
        raise ValueError(f"D muss die Form (4,1) bzw. 4 Koeffizienten haben, ist aber {dvec.shape}")

    return k, dvec


def scale_fisheye_intrinsics(k_matrix: np.ndarray, calib_dim: tuple[int, int], frame_dim: tuple[int, int]) -> np.ndarray:
    """
    Skaliert die Intrinsics von der Kalibrier-Aufloesung auf die aktuelle Frame-Aufloesung.
    D bleibt unveraendert.
    """
    calib_w, calib_h = calib_dim
    frame_w, frame_h = frame_dim

    sx = frame_w / calib_w
    sy = frame_h / calib_h

    k_scaled = k_matrix.copy()
    k_scaled[0, 0] *= sx  # fx
    k_scaled[1, 1] *= sy  # fy
    k_scaled[0, 2] *= sx  # cx
    k_scaled[1, 2] *= sy  # cy
    return k_scaled


def open_camera_opencv(index: int, width: int, height: int) -> cv2.VideoCapture:
    """Oeffnet einen OpenCV-Stream (USB/CSI je nach System)."""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Kamera mit Index {index} konnte nicht geoeffnet werden")

    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    return cap


def open_camera_picam2(width: int, height: int):
    """Oeffnet die Raspberry Pi Kamera ueber Picamera2."""
    if Picamera2 is None:
        raise RuntimeError(
            "Picamera2 ist nicht installiert. Installiere es z. B. mit: sudo apt install -y python3-picamera2"
        )

    picam2 = Picamera2()
    if width > 0 and height > 0:
        config = picam2.create_preview_configuration(main={"size": (width, height)})
    else:
        config = picam2.create_preview_configuration()

    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)
    return picam2


def picam2_to_bgr(frame: np.ndarray) -> np.ndarray:
    """Normalisiert Picamera2-Frames auf 3-Kanal-BGR fuer OpenCV."""
    if frame.ndim == 3 and frame.shape[2] == 3:
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if frame.ndim == 3 and frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    return frame


def supports_opencv_window() -> bool:
    """Prueft, ob OpenCV-Fenster in der aktuellen Umgebung verfuegbar sind."""
    # In Headless-Setups (z. B. SSH ohne X/Wayland) darf kein Fenster-Probing
    # versucht werden, da manche OpenCV/Qt-Builds den Prozess hart abbrechen.
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False

    try:
        cv2.namedWindow("_probe_", cv2.WINDOW_NORMAL)
        cv2.imshow("_probe_", np.zeros((8, 8, 3), dtype=np.uint8))
        cv2.waitKey(1)
        cv2.destroyWindow("_probe_")
        return True
    except cv2.error:
        return False


class _ThreadedHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True


class MJPEGServer:
    """Einfacher MJPEG-Server fuer Livebild im Browser."""

    def __init__(self, host: str, port: int):
        self._lock = threading.Lock()
        self._latest_jpeg = None

        state = self

        class Handler(server.BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body = (
                        "<html><head><title>Entzerrung Live</title></head>"
                        "<body style='background:#101010;color:#f2f2f2;font-family:monospace'>"
                        "<h2>Live-Stream: Original | Entzerrt</h2>"
                        "<img src='/stream.mjpg' style='max-width:100%;height:auto;border:1px solid #555'/>"
                        "</body></html>"
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path != "/stream.mjpg":
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                try:
                    while True:
                        with state._lock:
                            jpeg = state._latest_jpeg

                        if jpeg is None:
                            time.sleep(0.03)
                            continue

                        self.wfile.write(b"--frame\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.02)
                except (BrokenPipeError, ConnectionResetError):
                    return

            def log_message(self, _format: str, *_args):
                return

        self._httpd = _ThreadedHTTPServer((host, port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._httpd.shutdown()
        self._httpd.server_close()

    def set_frame(self, frame: np.ndarray):
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return
        with self._lock:
            self._latest_jpeg = encoded.tobytes()


def compute_valid_roi_from_maps(map1: np.ndarray, map2: np.ndarray, frame_size: tuple[int, int]) -> tuple[int, int, int, int]:
    """
    Berechnet eine gueltige ROI fuer das entzerrte Bild.
    Da fisheye kein ROI wie getOptimalNewCameraMatrix liefert,
    bestimmen wir die gueltige Flaeche ueber eine remappte Maske.
    """
    w, h = frame_size
    mask = np.full((h, w), 255, dtype=np.uint8)

    valid = cv2.remap(
        mask,
        map1,
        map2,
        interpolation=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )

    pts = cv2.findNonZero(valid)
    if pts is None:
        return 0, 0, w, h

    x, y, rw, rh = cv2.boundingRect(pts)
    return x, y, rw, rh


def build_fisheye_maps(frame_size: tuple[int, int], calib_dim: tuple[int, int], k: np.ndarray, d: np.ndarray):
    """
    Erstellt Remap-Tabellen fuer die Fisheye-Entzerrung.
    """
    frame_w, frame_h = frame_size
    k_scaled = scale_fisheye_intrinsics(k, calib_dim, frame_size)

    new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        k_scaled,
        d,
        (frame_w, frame_h),
        np.eye(3),
        balance=BALANCE,
    )

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        k_scaled,
        d,
        np.eye(3),
        new_k,
        (frame_w, frame_h),
        cv2.CV_16SC2,
    )

    roi = compute_valid_roi_from_maps(map1, map2, (frame_w, frame_h))
    return map1, map2, roi


def live_undistort(
    camera_index: int,
    width: int,
    height: int,
    crop: bool,
    backend: str,
    display: str,
    web_host: str,
    web_port: int,
) -> None:
    """Zeigt Livebild mit Fisheye-Entzerrung; ESC oder q beendet."""
    k, dvec = validate_calibration_fisheye(K, D)
    web = None

    if backend == "picam2":
        cam = open_camera_picam2(width, height)

        def read_frame():
            return picam2_to_bgr(cam.capture_array())

        release = cam.stop
    else:
        cam = open_camera_opencv(camera_index, width, height)

        def read_frame():
            ret, frame = cam.read()
            return frame if ret else None

        release = cam.release

    print("Live-Fisheye-Undistortion gestartet")

    calib_w, calib_h = CALIB_DIM
    if width > 0 and height > 0 and (width, height) != CALIB_DIM:
        print(
            f"Hinweis: Kalibrierung war vermutlich bei {CALIB_DIM}, "
            f"Live-Stream laeuft aber mit {(width, height)}."
        )
        print("Das funktioniert, aber am besten ist dieselbe Aufloesung wie bei der Kalibrierung.")

    window_mode = display
    if display == "auto":
        window_mode = "window" if supports_opencv_window() else "web"
    elif display == "window" and not supports_opencv_window():
        print("Hinweis: Kein Display-Server gefunden, wechsle auf Web-Stream.")
        window_mode = "web"

    if window_mode == "window":
        print("Anzeige: OpenCV-Fenster | Tasten: q oder ESC = beenden")
    else:
        web = MJPEGServer(web_host, web_port)
        web.start()
        print(f"Anzeige: Browser-Stream auf http://{web_host}:{web_port}")
        print("Beenden mit Ctrl+C im Terminal")

    last_frame_size = None
    map1 = None
    map2 = None
    roi = None

    try:
        while True:
            frame = read_frame()
            if frame is None:
                print("Warnung: Kein Frame gelesen, neuer Versuch...")
                continue

            h, w = frame.shape[:2]
            frame_size = (w, h)

            if frame_size != last_frame_size:
                map1, map2, roi = build_fisheye_maps(frame_size, CALIB_DIM, k, dvec)
                last_frame_size = frame_size
                print(f"Remap aktualisiert fuer Aufloesung {frame_size}, ROI={roi}, balance={BALANCE}")

            undistorted = cv2.remap(
                frame,
                map1,
                map2,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )

            if crop and roi is not None:
                x, y, rw, rh = roi
                if rw > 0 and rh > 0:
                    undistorted = undistorted[y:y + rh, x:x + rw]

            if undistorted.shape[:2] != frame.shape[:2]:
                undistorted = cv2.resize(undistorted, (frame.shape[1], frame.shape[0]))

            combined = np.hstack((frame, undistorted))

            cv2.putText(
                combined,
                "Original",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                combined,
                "Entzerrt (fisheye)",
                (frame.shape[1] + 20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            if window_mode == "window":
                cv2.imshow("Kamera Kalibrierung Live - Fisheye", combined)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
            else:
                web.set_frame(combined)

    finally:
        if web is not None:
            web.stop()
        release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live-Fisheye-Entzerrung mit vorgegebener K-Matrix und D-Vektor")
    parser.add_argument("--camera", type=int, default=0, help="Kamera-Index (default: 0)")
    parser.add_argument("--width", type=int, default=1456, help="Frame-Breite (default: 640)")
    parser.add_argument("--height", type=int, default=1088, help="Frame-Hoehe (default: 480)")
    parser.add_argument(
        "--backend",
        choices=["picam2", "opencv"],
        default="picam2",
        help="Kamera-Backend: picam2 (Raspberry Pi Cam) oder opencv",
    )
    parser.add_argument(
        "--display",
        choices=["auto", "window", "web"],
        default="auto",
        help="Ausgabeart: OpenCV-Fenster oder Browser-Stream",
    )
    parser.add_argument("--web-host", default="0.0.0.0", help="Host fuer MJPEG-Webserver (default: 0.0.0.0)")
    parser.add_argument("--web-port", type=int, default=8080, help="Port fuer MJPEG-Webserver (default: 8080)")
    parser.add_argument(
        "--no-crop",
        action="store_true",
        help="Kein Zuschneiden auf gueltige ROI nach Entzerrung",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        live_undistort(
            camera_index=args.camera,
            width=args.width,
            height=args.height,
            crop=not args.no_crop,
            backend=args.backend,
            display=args.display,
            web_host=args.web_host,
            web_port=args.web_port,
        )
    except KeyboardInterrupt:
        print("Beendet.")
        return 0
    except Exception as exc:
        print(f"Fehler: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())