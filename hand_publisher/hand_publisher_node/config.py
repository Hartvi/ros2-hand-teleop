import numpy as np
from dataclasses import dataclass, field

USE_DROID = False
IMAGE_SIZE = (1280, 720)
CAMS = ()


@dataclass
class CamInfo:
    w: float = IMAGE_SIZE[0]
    h: float = IMAGE_SIZE[1]
    x0: float = -1
    y0: float = -1
    f: float = -1
    R: np.ndarray = field(default_factory=lambda: np.eye(3, dtype=float))
    t: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

    def __post_init__(self):
        if self.x0 < 0:
            self.x0 = self.w / 2.0
        if self.y0 < 0:
            self.y0 = self.h / 2.0


def init_caps():
    global CAP_IDS, DROID_IDS, USE_DROID, PUB_METADATA, SUB_METADATA, NUM_SRCS, SRCS, CAMS
    import cv2
    import threading

    def get_caps_ids() -> list[int]:
        ids = []
        idx = 0
        max_probe = 10  # don't probe beyond /dev/video9
        while idx < max_probe:
            cap_ready = False

            def try_open_camera():
                nonlocal cap_ready
                try:
                    cap = cv2.VideoCapture(idx)
                    # Try to read with a very low timeout
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    ok, _ = cap.read()
                    cap.release()
                    cap_ready = ok
                except Exception:
                    cap_ready = False

            # Probe with timeout to avoid hanging on locked cameras
            thread = threading.Thread(target=try_open_camera, daemon=True)
            thread.start()
            thread.join(timeout=2.0)  # 2 second timeout per camera

            if cap_ready:
                ids.append(idx)
            idx += 1

        if ids:
            print(f"Detected cameras at indices: {ids}")
        else:
            print("Warning: no cameras detected (using droid or simulation)")
        return ids

    def droid_src(use_usb: bool) -> str:
        return (
            f"http://127.0.0.1:4747/video?{IMAGE_SIZE[0]}x{IMAGE_SIZE[1]}"
            if use_usb
            else f"http://192.168.0.95:4747/video?{IMAGE_SIZE[0]}x{IMAGE_SIZE[1]}"
        )

    CAP_IDS = get_caps_ids()
    DROID_IDS = [droid_src(True)] if USE_DROID else []
    SRCS = CAP_IDS + DROID_IDS
    NUM_SRCS = max(len(SRCS), len(DROID_IDS) + 1)  # at least one camera must be present
    CAMS = tuple(CamInfo() for _ in range(NUM_SRCS))

    print("NUM SRCS", NUM_SRCS)
