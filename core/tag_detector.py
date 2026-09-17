import cv2
import numpy as np

TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36H11
TAG_ID = 0


class TagDetector:

    def __init__(self):
        self.dict = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

        self.found = False
        self.corners = None
        self.center_x = -1
        self.frame_w = 640
        self.scale = 1.0

    def detect(self, frame):
        self.found = False
        self.corners = None
        self.scale = 1.0

        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ok = self._try_detect(gray, w)
        if not ok:
            small = cv2.resize(gray, (w // 2, h // 2))
            ok = self._try_detect(small, w)
            self.scale = 2.0
        if not ok:
            tiny = cv2.resize(gray, (w // 4, h // 4))
            ok = self._try_detect(tiny, w)
            self.scale = 4.0

        return ok

    def _try_detect(self, gray, orig_w):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            for i, tag_id in enumerate(ids):
                if tag_id[0] == TAG_ID:
                    self.found = True
                    self.corners = corners[i][0] * self.scale
                    cx = np.mean(self.corners[:, 0])
                    self.center_x = int(cx)
                    self.frame_w = orig_w
                    return True
        return False

    def get_offset_from_center(self):
        if not self.found or self.frame_w == 0:
            return 0.0
        return (self.center_x - self.frame_w / 2) / (self.frame_w / 2)

    def draw(self, frame):
        if self.found and self.corners is not None:
            pts = np.array([self.corners], dtype=np.int32)
            cv2.polylines(frame, pts, True, (0, 255, 0), 2)
            cv2.circle(frame, (self.center_x, frame.shape[0] // 2),
                       5, (0, 255, 0), -1)
        return frame
