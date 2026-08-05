import os
import glob
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

class CalibrationNode(Node):
    """
    Robust chessboard calibration for still images.
    Parameters (ROS):
      - images_path (str): glob to images. e.g. /abs/path/*.jpg
      - board_size  (int[2]): [cols, rows] INNER corners (e.g. [9,6])
      - square_size (float): size of one square in meters (e.g. 0.025)
      - min_detections (int): min number of successful boards to run solve (default 10)
      - write_debug (bool): save a few debug images (default True)
      - undistort_samples (int): how many undistorted previews to save (default 3)
    Outputs:
      - output/calib.yaml (K, dist, rms, image_size, board_size, square_size, n_images)
      - output/debug/ (optional drawn corners + undistorted samples)
    """
    def __init__(self):
        super().__init__('calibration_node')

        # ---------- Parameters ----------
        self.declare_parameter('images_path', '')
        self.declare_parameter('board_size', [9, 6])      # [cols, rows] inner corners
        self.declare_parameter('square_size', 0.025)       # meters
        self.declare_parameter('min_detections', 10)
        self.declare_parameter('write_debug', True)
        self.declare_parameter('undistort_samples', 3)

        self.images_path     = self.get_parameter('images_path').get_parameter_value().string_value
        self.board_size_list = self.get_parameter('board_size').get_parameter_value().integer_array_value
        self.square_size     = float(self.get_parameter('square_size').value)
        self.min_detections  = int(self.get_parameter('min_detections').value)
        self.write_debug     = bool(self.get_parameter('write_debug').value)
        self.undistort_n     = int(self.get_parameter('undistort_samples').value)

        # Convert to plain tuple
        if len(self.board_size_list) != 2:
            self.get_logger().fatal("board_size must be exactly two integers, e.g. [9,6].")
            raise SystemExit(1)
        self.board_size = (int(self.board_size_list[0]), int(self.board_size_list[1]))

        # ---------- Run ----------
        self.run()

    def run(self):
        images = sorted(glob.glob(self.images_path))
        self.get_logger().info(f'Found {len(images)} images matching: {self.images_path}')
        if not images:
            self.get_logger().fatal('No images found. Pass an absolute images_path (wildcards allowed).')
            raise SystemExit(2)

        # Prepare 3D points (0,0,0), (1*s,0,0) ... in chessboard plane
        cols, rows = self.board_size
        objp = np.zeros((cols*rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)  # (x,y) grid
        objp *= self.square_size

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

        objpoints = []  # 3D points in world coords
        imgpoints = []  # 2D points in image plane
        img_size = None

        # Debug dir
        out_dir = os.path.join(os.path.dirname(__file__), '..', 'output')
        out_dir = os.path.abspath(out_dir)
        dbg_dir = os.path.join(out_dir, 'debug')
        if self.write_debug:
            os.makedirs(dbg_dir, exist_ok=True)
        os.makedirs(out_dir, exist_ok=True)

        detections = 0
        for i, fname in enumerate(images):
            img = cv2.imread(fname, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn(f'Could not read {fname}, skipping.')
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_size = gray.shape[::-1]  # (w,h)

            ok, corners = cv2.findChessboardCorners(gray, self.board_size,
                                                    flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ok:
                continue

            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints.append(corners)
            detections += 1

            if self.write_debug and detections <= 5:
                vis = img.copy()
                cv2.drawChessboardCorners(vis, self.board_size, corners, ok)
                cv2.imwrite(os.path.join(dbg_dir, f'detected_{i:03d}.jpg'), vis)

        self.get_logger().info(f'Successful detections: {detections}')
        if detections < self.min_detections:
            self.get_logger().fatal(f'Not enough detections ({detections}) — need at least {self.min_detections}.')
            raise SystemExit(3)

        # Calibrate
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objectPoints=objpoints,
            imagePoints=imgpoints,
            imageSize=img_size,
            cameraMatrix=None,
            distCoeffs=None
        )

        self.get_logger().info(f'RMS reprojection error: {rms:.4f}')
        self.get_logger().info(f'K:\n{K}\nDist:\n{dist.ravel()}')

        # Save YAML
        calib_path = os.path.join(out_dir, 'calib.yaml')
        data = {
            'K': K.tolist(),
            'dist': dist.tolist(),
            'rms': float(rms),
            'image_size': {'width': int(img_size[0]), 'height': int(img_size[1])},
            'board_size': {'cols': cols, 'rows': rows},  # inner corners
            'square_size_m': float(self.square_size),
            'n_images_used': int(detections),
        }
        with open(calib_path, 'w') as f:
            yaml.safe_dump(data, f)
        self.get_logger().info(f'Saved intrinsics → {calib_path}')

        # Optional undistorted previews
        if self.write_debug:
            n = min(self.undistort_n, len(images))
            self.get_logger().info(f'Writing {n} undistorted preview(s) to {dbg_dir}')
            for j, fname in enumerate(images[:n]):
                img = cv2.imread(fname, cv2.IMREAD_COLOR)
                if img is None:
                    continue
                und = cv2.undistort(img, K, dist)
                cv2.imwrite(os.path.join(dbg_dir, f'undistorted_{j:02d}.jpg'), und)

        self.get_logger().info('Calibration complete.')

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    # No spin needed; node runs once and exits.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
