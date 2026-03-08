import pyrealsense2 as rs
import cv2
import numpy as np

class CharucoCalibration:
    def __init__(self, x, y, square_size, marker_size, aruco_dict = cv2.aruco.DICT_5X5_100, min_corners = 6):
        self.min_corners = min_corners
        self.board = cv2.aruco.CharucoBoard(
            (x, y),
            square_size,
            marker_size,
            cv2.aruco.getPredefinedDictionary(aruco_dict)
        )
        self.detector = cv2.aruco.CharucoDetector(self.board, cv2.aruco.CharucoParameters(), cv2.aruco.DetectorParameters())

        # Everything detected
        self.cornersR = []
        self.cornersL = []
        self.idsR = []
        self.idsL = []

        # The pairs used for calibration
        self.obj_pairs = []
        self.l_pairs = []
        self.r_pairs = []

        self.detections = 0
        self.pairs = 0
    def feed(self, imgR, imgL): # A frame is valid so long as both cameras detect the minimum points        
        cornersR, idsR, _, _ = self.detector.detectBoard(imgR)
        cornersL, idsL, _, _ = self.detector.detectBoard(imgL)

        if(cornersR is None or len(cornersR) <= self.min_corners):
            return False
        if(cornersL is None or len(cornersL) <= self.min_corners):
            return False
        
        self.cornersR.append(cornersR)
        self.cornersL.append(cornersL)
        self.idsR.append(idsR)
        self.idsL.append(idsL)

        self.detections += 1

        return True
    def generatePairs(self): # Pairs exist where both cameras detected shared points
        board_corners = self.board.getChessboardCorners()

        for cornersR, idsR, cornersL, idsL in zip(self.cornersR, self.idsR, self.cornersL, self.idsL):
            flatR = idsR.flatten()
            flatL = idsL.flatten()

            common_ids, iR, iL = np.intersect1d(flatR, flatL, assume_unique=True, return_indices=True)

            if len(common_ids) < self.min_corners:
                continue
            
            self.obj_pairs.append(board_corners[common_ids].astype(np.float32))
            self.r_pairs.append(cornersR[iR].astype(np.float32))
            self.l_pairs.append(cornersL[iL].astype(np.float32))
            self.pairs += 1
        
        return self.pairs
    def calibrate(self, width, height, mtxR, distR, mtxL, distL):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        flags = cv2.CALIB_FIX_INTRINSIC

        ret, cmR, distR, cmL, distL, R, T, E, F = cv2.stereoCalibrate(
            self.obj_pairs,
            self.r_pairs,
            self.l_pairs,
            mtxR, distR,
            mtxL, distL,
            (width, height),
            critera=critera,
            flags=flags
        )

        return ret, cmR, distR, cmL, distL, R, T, E, F

def get_intr(rs_stream):
    intr = rs_stream.as_video_stream_profile().get_intrinsics()
    mtx = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0 , 0, 1]
    ], dtype=np.float)
    dist = np.array(intr.coeffs, dtype=np.float64)

    return mtx, dist

RESOLUTION = (1280, 720)
FPS = 15

ctx = rs.context()
devices = ctx.query_devices()

pipelines = []
serials = []

for device in devices[:2]:
    pipeline = rs.pipeline()
    config = rs.config()

    serial = device.get_info(rs.camera_info.serial_number)
    serials.append(serial)

    config.enable_device(serial)
    config.enable_stream(
        rs.stream.color,
        RESOLUTION[0], RESOLUTION[1],
        rs.format.bgr8,
        FPS
    )

    pipeline.start(config)
    pipelines.append(pipeline)

print(f"Serials: {serials[0]} (cam1), {serials[1]} (cam2)")

# ChArUco
calib = CharucoCalibration(5, 7, 0.030, 0.015)

print("Warming up cameras...")
for _ in range(60):
    for p in pipelines:
        p.wait_for_frames()


print(f"Move the ChArUco board to {20} different positions/angles.")
print("Press 'q' to quit early.\n")

while calib.detections < 20:

    framesR = pipelines[0].wait_for_frames()
    framesL = pipelines[1].wait_for_frames()

    bgrR = np.asanyarray(framesR.get_color_frame().get_data())
    bgrL = np.asanyarray(framesL.get_color_frame().get_data())

    grayR = cv2.cvtColor(bgrR, cv2.COLOR_BGR2GRAY)
    grayL = cv2.cvtColor(bgrL, cv2.COLOR_BGR2GRAY)

    if(calib.feed(grayR, grayL)):
        print(f"Captured ({calib.detections}) calibration-valid frames.")

print(f"{calib.generatePairs()} pairs found.")

ret, cmR, distR, cmL, distL, R, T, E, F = calib.calibrate()

print(f"{ret} error.")