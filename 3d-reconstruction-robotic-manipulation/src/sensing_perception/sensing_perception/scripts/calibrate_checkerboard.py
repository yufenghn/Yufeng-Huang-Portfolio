import glob, sys, os, yaml
import numpy as np
import cv2 as cv

# Config
IMAGES_GLOB = sys.argv[1] if len(sys.argv) > 1 else ""
ROWS = int(sys.argv[2]) if len(sys.argv) > 2 else 7   # inner corners vertically
COLS = int(sys.argv[3]) if len(sys.argv) > 3 else 10  # inner corners horizontally
SQUARE = float(sys.argv[4]) if len(sys.argv) > 4 else 0.025  # meters per square
OUT_YAML = sys.argv[5] if len(sys.argv) > 5 else "calib.yaml"

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

# Prepare object points
objp = np.zeros((ROWS * COLS, 3), np.float32)
objp[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2)
objp *= SQUARE

objpoints = []
imgpoints = []

images = sorted(glob.glob(IMAGES_GLOB))
if not images:
    print("No images matched:", IMAGES_GLOB)
    sys.exit(1)

img_size = None
used = 0

for fn in images:
    img = cv.imread(fn)
    if img is None:
        continue
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    if img_size is None:
        img_size = (gray.shape[1], gray.shape[0])

    ret, corners = cv.findChessboardCorners(gray, (COLS, ROWS), None)
    if ret:
        corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        used += 1

if used < 8:
    print("Not enough usable images:", used)
    sys.exit(2)

rms, K, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)

K_list = [
    float(K[0, 0]), float(K[0, 1]), float(K[0, 2]),
    float(K[1, 0]), float(K[1, 1]), float(K[1, 2]),
    float(K[2, 0]), float(K[2, 1]), float(K[2, 2])
]

data = {
    "image_width": img_size[0],
    "image_height": img_size[1],
    "K": K_list,
    "dist_coeffs": dist.flatten().tolist(),
    "rms_reprojection_error": float(rms),
}

with open(OUT_YAML, "w") as f:
    yaml.dump(data, f, sort_keys=False)

print("Wrote", OUT_YAML)
print("K =", K_list)
print("RMS error =", rms)
