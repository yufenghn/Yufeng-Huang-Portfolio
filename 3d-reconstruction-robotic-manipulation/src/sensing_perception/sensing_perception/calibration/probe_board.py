import glob, os, cv2
import numpy as np

IMAGES = "/home/yufeng/7CCEMSAP_ws/src/sensing_perception/sensing_perception/data/calib/*.jpg"
OUTDIR = os.path.abspath(os.path.join(__file__, "..", "..", "output", "debug"))
os.makedirs(OUTDIR, exist_ok=True)

def try_detect(img, gray, size):
    cols, rows = size
    # 1) modern SB detector (best)
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=0)
    method = "SB"
    if not ok:
        # 2) legacy fallback with robust flags
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags=flags)
        method = "LEGACY"
        if ok:
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                   (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    return ok, corners, method

imgs = sorted(glob.glob(IMAGES))[:12]  # check first 12
print(f"Probing {len(imgs)} images")
sizes = [(9,6),(6,9)]  # try both orientations

for i, f in enumerate(imgs):
    img = cv2.imread(f); 
    if img is None: 
        print("Cannot read", f); 
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found = False
    for size in sizes:
        ok, corners, method = try_detect(img, gray, size)
        if ok:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, size, corners, ok)
            base = os.path.join(OUTDIR, f"probe_{i:02d}_{size[0]}x{size[1]}_{method}.jpg")
            cv2.imwrite(base, vis)
            print(f"[OK] {os.path.basename(f)} with size {size} via {method}")
            found = True
            break
    if not found:
        print(f"[FAIL] {os.path.basename(f)} (no pattern)")
print(f"Debug images → {OUTDIR}")
