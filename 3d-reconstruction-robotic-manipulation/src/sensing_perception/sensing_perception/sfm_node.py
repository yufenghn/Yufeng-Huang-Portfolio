import os
import glob
import json
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node


# Helpers

def reprojection_filter(K, R, t, pts1_px, pts2_px, X_cam1, max_err_px=1.5):
    """
    Filter points based on reprojection error in both cameras.
    X_cam1: Nx3 points in cam1 coordinates
    """
    X1 = X_cam1  # Nx3

    # project to cam1
    x1 = (K @ X1.T).T
    x1 = x1[:, :2] / np.clip(x1[:, 2:3], 1e-12, None)

    # project to cam2
    X2 = (R @ X1.T + t).T
    x2 = (K @ X2.T).T
    x2 = x2[:, :2] / np.clip(x2[:, 2:3], 1e-12, None)

    e1 = np.linalg.norm(x1 - pts1_px, axis=1)
    e2 = np.linalg.norm(x2 - pts2_px, axis=1)
    return (e1 < max_err_px) & (e2 < max_err_px)


def depth_percentile_filter(X_cam1, lo=10, hi=90):
    """
    Keep points whose depth (Z) lies between lo and hi percentiles.
    Helps remove very near / very far outliers.
    """
    z = X_cam1[:, 2]
    if len(z) < 10:
        return np.ones(len(z), dtype=bool)
    lo_v, hi_v = np.percentile(z, [lo, hi])
    return (z > max(1e-6, lo_v)) & (z < hi_v)


def write_ply(path, pts, colors=None):
    """
    Save a colored point cloud to PLY (ASCII).
    """
    pts = np.asarray(pts, np.float32)
    if colors is None:
        colors = np.full_like(pts, 180, dtype=np.uint8)

    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for p, c in zip(pts, colors):
            f.write(f"{p[0]} {p[1]} {p[2]} {c[2]} {c[1]} {c[0]}\n")


def load_calib(path):
    """
    Load calibration YAML with keys:
      K: 3x3 or flat 9
      dist: list of 5 (optional)
    """
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    K = np.array(data["K"], dtype=np.float64)
    K = K.reshape(3, 3)

    dist = np.array(data.get("dist", [0, 0, 0, 0, 0]), dtype=np.float64)
    return K, dist


def detector(use_sift=True):
    """
    Create a SIFT or ORB detector + descriptor.
    Returns (detector, name, use_hamming)
    """
    if use_sift and hasattr(cv2, "SIFT_create"):
        return cv2.SIFT_create(), "SIFT", False
    return cv2.ORB_create(6000), "ORB", True


def match_des(d1, d2, use_hamming):
    """
    Descriptor matching with Lowe's ratio test.
    """
    if d1 is None or d2 is None:
        return []

    if use_hamming:
        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        m = bf.knnMatch(d1, d2, k=2)
    else:
        flann = cv2.FlannBasedMatcher(
            dict(algorithm=1, trees=5),
            dict(checks=64)
        )
        m = flann.knnMatch(d1.astype(np.float32), d2.astype(np.float32), k=2)

    good = []
    for pair in m:
        if len(pair) < 2:
            continue
        m1, m2 = pair
        if m1.distance < 0.75 * m2.distance:
            good.append(m1)
    return good


# SfM Node

class SfMNode(Node):
    def __init__(self):
        super().__init__("sfm_node")

        # parameters
        self.declare_parameter("images_path", "")
        self.declare_parameter("calib_yaml", "")
        self.declare_parameter("use_sift", True)
        self.declare_parameter("min_pair_inliers", 80)
        self.declare_parameter("max_reproj_err", 1.2)

        # read params
        self.images_glob = self.get_parameter("images_path").value
        self.calib_yaml = self.get_parameter("calib_yaml").value
        self.use_sift = self.get_parameter("use_sift").value
        self.min_inl = self.get_parameter("min_pair_inliers").value
        self.max_rep = self.get_parameter("max_reproj_err").value
        self.substep = max(1, int(self.get_parameter("subsample_step").value))

        if not self.images_glob or not self.calib_yaml:
            self.get_logger().fatal("Missing parameters images_path or calib_yaml")
            raise SystemExit(1)

        self.run_sfm()

    def run_sfm(self):

        # Load images
        paths = sorted(glob.glob(self.images_glob))
        paths = paths[::self.substep]

        if len(paths) < 2:
            self.get_logger().fatal("Not enough image files found.")
            raise SystemExit(2)

        K, dist = load_calib(self.calib_yaml)
        self.get_logger().info(f"Loaded calibration from {self.calib_yaml}")

        imgs_c, imgs_g = [], []

        for p in paths:
            img = cv2.imread(p)
            if img is None:
                self.get_logger().warn(f"Failed to read image: {p}")
                continue

            und = cv2.undistort(img, K, dist)

            imgs_c.append(und)
            imgs_g.append(cv2.cvtColor(und, cv2.COLOR_BGR2GRAY))

        if len(imgs_g) < 2:
            self.get_logger().fatal("Not enough images after loading/undistort.")
            raise SystemExit(3)

        det, det_name, use_hamming = detector(self.use_sift)
        self.get_logger().info(f"Using feature detector: {det_name}")


        # SfM main loop
        T_w_c = [np.eye(4)]
        map_points = []
        map_colors = []

        for i in range(len(imgs_g) - 1):
            g1 = imgs_g[i]
            g2 = imgs_g[i + 1]
            c2 = imgs_c[i + 1]

            # Detect + describe
            k1, d1 = det.detectAndCompute(g1, None)
            k2, d2 = det.detectAndCompute(g2, None)

            if d1 is None or d2 is None or len(k1) < 20 or len(k2) < 20:
                self.get_logger().warn(f"[{i}-{i+1}] not enough keypoints, copying previous pose.")
                T_w_c.append(T_w_c[-1].copy())
                continue

            matches = match_des(d1, d2, use_hamming)

            if len(matches) < self.min_inl:
                self.get_logger().warn(f"[{i}-{i+1}] low matches: {len(matches)}")

            if len(matches) < 8:
                self.get_logger().warn(f"[{i}-{i+1}] too few matches AFTER ratio test.")
                T_w_c.append(T_w_c[-1].copy())
                continue

            pts1 = np.float32([k1[m.queryIdx].pt for m in matches])
            pts2 = np.float32([k2[m.trainIdx].pt for m in matches])

            # Essential matrix with RANSAC
            E, inliers = cv2.findEssentialMat(
                pts1, pts2, K,
                method=cv2.RANSAC, prob=0.999, threshold=1.0
            )

            if E is None:
                self.get_logger().warn(f"[{i}-{i+1}] Essential matrix estimation failed.")
                T_w_c.append(T_w_c[-1].copy())
                continue

            inliers = inliers.ravel().astype(bool)
            pts1i = pts1[inliers]
            pts2i = pts2[inliers]

            if len(pts1i) < 8:
                self.get_logger().warn(f"[{i}-{i+1}] too few inliers after RANSAC.")
                T_w_c.append(T_w_c[-1].copy())
                continue

            # Recover relative pose R, t mapping cam1 to cam2
            _, R, t, _ = cv2.recoverPose(E, pts1i, pts2i, K)

            Twc_prev = T_w_c[-1]
            Rwc_prev = Twc_prev[:3, :3]
            twc_prev = Twc_prev[:3, 3:4]

            Rwc_now = Rwc_prev @ R.T
            twc_now = twc_prev - Rwc_now @ t

            Twc_now = np.eye(4)
            Twc_now[:3, :3] = Rwc_now
            Twc_now[:3, 3] = twc_now.ravel()
            T_w_c.append(Twc_now)

            # Triangulation between cam1 and cam2
            P1 = K @ np.hstack([np.eye(3), np.zeros((3, 1))])
            P2 = K @ np.hstack([R, t])

            X4 = cv2.triangulatePoints(P1, P2, pts1i.T, pts2i.T)
            X = (X4[:3] / (X4[3] + 1e-9)).T  # Nx3 in cam1 coordinates

            X2 = (R @ X.T + t).T
            good = (X[:, 2] > 0) & (X2[:, 2] > 0)
            X = X[good]
            pts1i = pts1i[good]
            pts2i = pts2i[good]

            if len(X) == 0:
                continue

            # Reprojection filtering
            ok = reprojection_filter(K, R, t, pts1i, pts2i, X,
                                     max_err_px=self.max_rep)
            X = X[ok]
            pts2i = pts2i[ok]

            if len(X) == 0:
                continue

            # Depth percentile filtering
            okd = depth_percentile_filter(X)
            X = X[okd]
            pts2i = pts2i[okd]

            if len(X) == 0:
                continue

            # Colors sampled from image 2
            for (u, v) in pts2i.astype(int):
                u = np.clip(u, 0, c2.shape[1] - 1)
                v = np.clip(v, 0, c2.shape[0] - 1)
                map_colors.append(c2[v, u])

            # Transform from cam1 to world using previous Twc_prev
            Xw = (Rwc_prev @ X.T + twc_prev).T
            map_points.append(Xw)

        # Save RAW and CLEAN clouds
        if len(map_points) == 0:
            self.get_logger().fatal("No points reconstructed.")
            raise SystemExit(5)

        pts = np.vstack(map_points).astype(np.float32)
        cols = np.vstack(map_colors).astype(np.uint8)

        R_flip = np.diag([-1.0, 1.0, 1.0]).astype(np.float32)
        pts = (R_flip @ pts.T).T

        for i, T in enumerate(T_w_c):
            R = T[:3, :3]
            t = T[:3, 3]
            R_new = R_flip @ R
            t_new = R_flip @ t
            T_new = T.copy()
            T_new[:3, :3] = R_new
            T_new[:3, 3] = t_new
            T_w_c[i] = T_new
        # --------------------------------------------------------

        out_dir = os.path.abspath(os.path.join(__file__, "..", "..", "output"))
        os.makedirs(out_dir, exist_ok=True)

        raw_path = os.path.join(out_dir, "sparse_cloud_raw.ply")
        write_ply(raw_path, pts, cols)
        self.get_logger().info(f"RAW cloud saved: {raw_path}")

        d = np.linalg.norm(pts - np.median(pts, axis=0), axis=1)
        thr = np.percentile(d, 85)
        mask = d < thr
        pc = pts[mask]
        cc = cols[mask]

        # recenter + scale
        center = np.median(pc, axis=0)
        pc -= center
        scale = np.percentile(np.linalg.norm(pc, axis=1), 95)
        if scale > 1e-6:
            pc /= scale

        clean_path = os.path.join(out_dir, "sparse_cloud_clean.ply")
        write_ply(clean_path, pc, cc)
        self.get_logger().info(f"CLEAN cloud saved: {clean_path}")

        # save poses
        pose_path = os.path.join(out_dir, "poses.json")
        with open(pose_path, "w") as f:
            json.dump({"poses_Twc": [T.tolist() for T in T_w_c]}, f)

        self.get_logger().info("SfM COMPLETE.")



# Main

def main(args=None):
    rclpy.init(args=args)
    node = SfMNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
