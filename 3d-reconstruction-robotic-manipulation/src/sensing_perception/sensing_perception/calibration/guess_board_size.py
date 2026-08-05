import glob, os, cv2, numpy as np
IMAGES = "/home/yufeng/7CCEMSAP_ws/src/sensing_perception/sensing_perception/data/calib/*.jpg"
OUTDIR = os.path.abspath(os.path.join(__file__, "..", "..", "output", "debug_guess"))
os.makedirs(OUTDIR, exist_ok=True)

def detect(gray, size):
    cols, rows = size
    # Try robust modern detector first
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=0)
    if ok: return ok, corners, "SB"
    # Fallback: legacy with strong flags + subpixel
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags=flags)
    if ok:
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
            (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    return ok, corners, "LEGACY" if ok else "NONE"

imgs = sorted(glob.glob(IMAGES))
print(f"Scanning {len(imgs)} images")
# Try a reasonable grid of inner-corner sizes (columns x rows)
cands = [(c,r) for c in range(4,13) for r in range(4,13)]
scores = {s:0 for s in cands}
examples = {}

for idx, f in enumerate(imgs[:40]):  # first 40 frames are enough to guess
    img = cv2.imread(f)
    if img is None: continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    best = None
    # Try both orientations for each candidate
    for (c,r) in cands:
        for size in [(c,r),(r,c)]:
            ok, corners, how = detect(gray, size)
            if ok:
                scores[size] += 1
                if size not in examples:
                    vis = img.copy()
                    cv2.drawChessboardCorners(vis, size, corners, True)
                    examples[size] = (vis, os.path.basename(f), how)

# Sort by detections
ranked = sorted(scores.items(), key=lambda kv: kv[1], reverse=True)
print("\nTop candidates (inner corners):")
for (size, cnt) in ranked[:8]:
    if cnt>0:
        print(f"  {size[0]}x{size[1]}  -> {cnt} hits")

if ranked[0][1] == 0:
    print("\nNo size worked. Check that your pattern is a true chessboard (NOT ChArUco/circles),"
          " the whole board is visible, and images are sharp/contrasty.")
else:
    size = ranked[0][0]
    print(f"\nMost likely inner-corner size → {size[0]}x{size[1]}")
    # Save a couple of visual examples
    os.makedirs(OUTDIR, exist_ok=True)
    saved = 0
    for k,(vis, name, how) in examples.items():
        if k == size and saved < 3:
            outp = os.path.join(OUTDIR, f"guess_{size[0]}x{size[1]}_{how}_{saved}.jpg")
            cv2.imwrite(outp, vis); saved += 1
    print(f"Debug images (with drawn corners) → {OUTDIR}")
