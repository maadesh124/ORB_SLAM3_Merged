import numpy as np
import cv2 as cv
import glob
import os
import sys

def calibrate_camera_from_folder(image_folder):
    # Constants
    board_size = (7, 6)         # (columns, rows) of inner corners
    square_size = 1.0           # Unit length of each square (can be cm, meters etc.)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points for a single image
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []  # 3D real-world points
    imgpoints = []  # 2D image points

    image_paths = sorted(glob.glob(os.path.join(image_folder, "*.jpg")))
    if not image_paths:
        print(f"❌ No .jpg images found in folder: {image_folder}")
        return

    print(f"📸 Found {len(image_paths)} images in {image_folder}. Starting calibration...")

    for fname in image_paths:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, corners = cv.findChessboardCorners(gray, board_size, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            # Optional display
            cv.drawChessboardCorners(img, board_size, corners2, ret)
            cv.imshow('Corners', img)
            cv.waitKey(200)
        else:
            print(f"⚠️ Chessboard not detected in: {fname}")

    cv.destroyAllWindows()

    if not objpoints:
        print("❌ Calibration failed. No valid chessboard patterns were found.")
        return

    # Perform calibration
    ret, K, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)



    # Compute re-projection error
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
        total_error += error
    print("\n🎯 Calibration Results (Formatted):")
    print(f"Camera.fx: {K[0,0]:.1f}")
    print(f"Camera.fy: {K[1,1]:.1f}")
    print(f"Camera.cx: {K[0,2]:.1f}")
    print(f"Camera.cy: {K[1,2]:.1f}\n")

    print(f"Camera.k1: {dist[0,0]:.5f}")
    print(f"Camera.k2: {dist[0,1]:.5f}")
    print(f"Camera.p1: {dist[0,2]:.5f}")
    print(f"Camera.p2: {dist[0,3]:.5f}\n")

    print(f"Camera.width: {gray.shape[1]}")
    print(f"Camera.height: {gray.shape[0]}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 calibrate_camera.py <image_folder>")
        sys.exit(1)

    folder_path = sys.argv[1]
    calibrate_camera_from_folder(folder_path)

