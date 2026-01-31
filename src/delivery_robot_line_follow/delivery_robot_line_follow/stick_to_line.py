import time
import cv2
import numpy as np

class LineFollower:
    def __init__(self,
                 width=640, height=480,
                 linePos_1=380, linePos_2=430,
                 invert_for_black_line=True,
                 erode_iterations=6,
                 jpeg_quality=80):

        self.width = width
        self.height = height

        self.linePos_1 = linePos_1
        self.linePos_2 = linePos_2

        self.lineColorSet = 255

        self.invert_for_black_line = invert_for_black_line

        self.erode_iterations = erode_iterations

        self.jpeg_quality = jpeg_quality

        self.prev_time = time.time()
        self.fps = 0.0

        self.last_center = None

    def process_jpeg(self, jpg_bytes):
        np_arr = np.frombuffer(jpg_bytes, np.uint8)
        frame_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame_image is None:
            return jpg_bytes, {"ok": False, "reason": "decode_failed"}

        frame_image = cv2.resize(frame_image, (self.width, self.height))

        out_frame, center, error = self.process_frame(frame_image)

        ok, encoded = cv2.imencode(
            ".jpg",
            out_frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            return jpg_bytes, {"ok": False, "reason": "encode_failed"}

        metrics = {
            "ok": True,
            "fps": self.fps,
            "center_x": center,
            "error_px": error
        }
        return encoded.tobytes(), metrics

    def process_frame(self, frame_image):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        if dt > 0:
            self.fps = 1.0 / dt

        frame_gray = cv2.cvtColor(frame_image, cv2.COLOR_BGR2GRAY)

        if self.invert_for_black_line:
            _, frame_findline = cv2.threshold(frame_gray, 0, 255,
                                              cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        else:
            _, frame_findline = cv2.threshold(frame_gray, 0, 255,
                                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        frame_findline = cv2.erode(frame_findline, None, iterations=self.erode_iterations)

        colorPos_1 = frame_findline[self.linePos_1]
        colorPos_2 = frame_findline[self.linePos_2]

        lineIndex_Pos1 = np.where(colorPos_1 == self.lineColorSet)[0]
        lineIndex_Pos2 = np.where(colorPos_2 == self.lineColorSet)[0]

        left_Pos1 = right_Pos1 = center_Pos1 = None
        left_Pos2 = right_Pos2 = center_Pos2 = None

        try:
            if len(lineIndex_Pos1) > 0:
                left_Pos1 = int(lineIndex_Pos1[0])
                right_Pos1 = int(lineIndex_Pos1[-1])
                center_Pos1 = int((left_Pos1 + right_Pos1) / 2)
        except:
            pass

        try:
            if len(lineIndex_Pos2) > 0:
                left_Pos2 = int(lineIndex_Pos2[0])
                right_Pos2 = int(lineIndex_Pos2[-1])
                center_Pos2 = int((left_Pos2 + right_Pos2) / 2)
        except:
            pass

        center = None
        if center_Pos1 is not None and center_Pos2 is not None:
            center = int((center_Pos1 + center_Pos2) / 2)
        elif center_Pos1 is not None:
            center = center_Pos1
        elif center_Pos2 is not None:
            center = center_Pos2

        if center is None and self.last_center is not None:
            center = self.last_center

        if center is not None:
            self.last_center = center

        error = None
        if center is not None:
            error = int(center - self.width / 2)

        out = cv2.cvtColor(frame_findline, cv2.COLOR_GRAY2BGR)

        cv2.line(out, (0, self.linePos_1), (self.width, self.linePos_1), (150, 150, 150), 1)
        cv2.line(out, (0, self.linePos_2), (self.width, self.linePos_2), (150, 150, 150), 1)

        if left_Pos1 is not None:
            cv2.line(out, (left_Pos1, self.linePos_1 - 30), (left_Pos1, self.linePos_1 + 30), (255, 255, 255), 1)
        if right_Pos1 is not None:
            cv2.line(out, (right_Pos1, self.linePos_1 - 30), (right_Pos1, self.linePos_1 + 30), (255, 255, 255), 1)

        if left_Pos2 is not None:
            cv2.line(out, (left_Pos2, self.linePos_2 - 30), (left_Pos2, self.linePos_2 + 30), (255, 255, 255), 1)
        if right_Pos2 is not None:
            cv2.line(out, (right_Pos2, self.linePos_2 - 30), (right_Pos2, self.linePos_2 + 30), (255, 255, 255), 1)

        if center is not None:
            cy = int((self.linePos_1 + self.linePos_2) / 2)
            cv2.line(out, (center - 20, cy), (center + 20, cy), (0, 0, 0), 2)
            cv2.line(out, (center, cy - 20), (center, cy + 20), (0, 0, 0), 2)

        frame_center = int(self.width / 2)
        cv2.line(out, (frame_center, 0), (frame_center, self.height), (80, 80, 80), 1)

        cv2.putText(out, f"FPS: {self.fps:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if error is not None:
            cv2.putText(out, f"Error: {error}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(out, "Line lost", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return out, center, error
