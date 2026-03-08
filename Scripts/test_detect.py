import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_5X5_250
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_SIZE = 0.030
MARKER_SIZE = 0.015

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_SIZE, MARKER_SIZE, aruco_dict)
detector_params = cv2.aruco.DetectorParameters()
charuco_detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), detector_params)
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

img = cv2.imread("./board_photo_Color.png")
if img is None:
    raise FileNotFoundError("Could not load board_photo_Color.png")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
display = img.copy()

# --- Layer 1: raw ArUco ---
aruco_corners, aruco_ids, rejected = aruco_detector.detectMarkers(gray)
n_aruco = len(aruco_ids) if aruco_ids is not None else 0
print(f"Raw ArUco markers detected: {n_aruco}")
if aruco_ids is not None:
    print(f"  IDs found: {sorted(aruco_ids.flatten().tolist())}")
    cv2.aruco.drawDetectedMarkers(display, aruco_corners, aruco_ids)

print(f"Rejected candidates: {len(rejected)}")

# --- Layer 2: ChArUco ---
charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
n_charuco = len(charuco_corners) if charuco_corners is not None else 0
print(f"\nChArUco corners detected: {n_charuco}")

if charuco_corners is not None:
    print(f"  Corner IDs: {sorted(charuco_ids.flatten().tolist())}")
    cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids, (0, 255, 0))
else:
    print("  ChArUco interpolation failed")

# --- Layer 3: pose estimation (if enough corners) ---
if n_charuco >= 6:
    # Rough camera matrix for a 1280x720 image — good enough for pose vis
    h, w = img.shape[:2]
    focal = w  
    camera_matrix = np.array([
        [focal,     0, w / 2],
        [    0, focal, h / 2],
        [    0,     0,     1]
    ], dtype=np.float64)
    dist_coeffs = np.zeros(5)
    
    obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids)
    if obj_pts is not None and len(obj_pts) >= 4:
        valid, rvec, tvec, _ = cv2.solvePnPRansac(
            obj_pts, img_pts, camera_matrix, dist_coeffs
        )
        if valid:
            cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, SQUARE_SIZE * 2)
            dist_m = np.linalg.norm(tvec)
            print(f"\nPose estimated successfully")
            print(f"  Distance to board: {dist_m*100:.1f} cm")
            print(f"  Translation (m): {tvec.flatten().round(4).tolist()}")
            print(f"  Rotation (deg):  {(np.degrees(rvec)).flatten().round(2).tolist()}")
        else:
            print("\nPose estimation failed (solvePnP returned false)")
    else:
        print("\nPose estimation failed — matchImagePoints returned no points")

else:
    print(f"\nSkipping pose estimation — need 6+ corners, got {n_charuco}")

# --- Status summary ---
print("\n--- Summary ---")
if n_aruco == 0:
    print("FAIL: No ArUco markers found — wrong dictionary or image quality issue")
elif n_charuco == 0:
    print("FAIL: ArUco ok but ChArUco failed — likely SQUARES_X/Y mismatch or MARKER_SIZE too large")
elif n_charuco < 6:
    print(f"PARTIAL: Only {n_charuco} ChArUco corners — board may be partially out of frame")
else:
    print(f"OK: {n_aruco} ArUco markers, {n_charuco} ChArUco corners")

# --- Display ---
scale = min(1280 / display.shape[1], 720 / display.shape[0])
preview = cv2.resize(display, (int(display.shape[1]*scale), int(display.shape[0]*scale)))
cv2.imshow("ChArUco Detection", preview)
cv2.waitKey(0)
cv2.destroyAllWindows()