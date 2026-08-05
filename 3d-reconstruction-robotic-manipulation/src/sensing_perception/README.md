# README – Structure-from-Motion (SfM) Pipeline  
7CCEMSAP – Sensing & Perception  
Author: Yufeng  
Package: sensing_perception

## 1. Overview
This ROS2 package implements a complete Structure-from-Motion (SfM) pipeline for still images. The system supports:
- Camera calibration using a chessboard target
- Feature detection and matching (SIFT)
- Essential matrix estimation
- Two-view and multi-view pose recovery
- Triangulation and sparse 3D reconstruction
- Visualisation and saving of reconstruction results

The pipeline is designed to be fully reproducible using ROS2 parameters.

## 2. Package Structure
```
sensing_perception/
│
├── sensing_perception/
│   ├── sfm_node.py
│   ├── __init__.py
│
├── scripts/
│   ├── calibrate_checkerboard.py
│
├── data/
│   ├── calib/
│   └── object/
│
├── output/
│   ├── calib.yaml
│   └── sfm_object/
│
├── package.xml
├── setup.py
└── README.md
```

## 3. Building the Workspace
```
cd ~/7CCEMSAP_ws
colcon build
source install/setup.bash
```

## 4. Camera Calibration

### 4.1 Calibration Images
Place all calibration images into:
```
src/sensing_perception/sensing_perception/data/calib/
```
Calibration board specification:
- Inner corners: 10 × 7
- Square size: 0.025 m

### 4.2 Running Calibration
```
cd ~/7CCEMSAP_ws/src/sensing_perception/sensing_perception/scripts

python3 calibrate_checkerboard.py "/home/yufeng/7CCEMSAP_ws/src/sensing_perception/sensing_perception/data/calib/*.jpg" 7 10 0.025 "/home/yufeng/7CCEMSAP_ws/src/sensing_perception/sensing_perception/output/calib.yaml"
```

## 5. Running Structure-from-Motion

### 5.1 Object Image Dataset
```
src/sensing_perception/sensing_perception/data/object/
```

### 5.2 Running the SfM Node
```
ros2 run sensing_perception sfm   --ros-args   -p images_path:="/home/yufeng/7CCEMSAP_ws/src/sensing_perception/sensing_perception/data/object/*.jpg"   -p calib_yaml:="/home/yufeng/7CCEMSAP_ws/src/sensing_perception/sensing_perception/output/calib.yaml"   -p use_sift:=true   -p min_pair_inliers:=60   -p output_dir:="/home/yufeng/7CCEMSAP_ws/src/sensing_perception/sensing_perception/output/sfm_object"
```

## 6. Output Files
Results are saved to:
```
output/sfm_object/
```
Typical outputs:
- matches_*.png
- pose_*.txt
- cloud.npy
- cloud.ply
- log.txt

## 7. SfM Parameters
| Parameter | Type | Description |
|----------|------|-------------|
| images_path | string | Glob path for input images |
| calib_yaml | string | Path to calibration YAML |
| use_sift | bool | Enable SIFT extraction |
| min_pair_inliers | int | RANSAC inlier threshold |
| output_dir | string | Output directory |

## 8. Reproducible Commands
### Build
```
colcon build
source install/setup.bash
```

### Calibrate
```
python3 calibrate_chessboard.py ".../calib/*.jpg" 7 10 0.025 ".../output/calib.yaml"
```

### Run SfM
```
ros2 run sensing_perception sfm --ros-args -p images_path:=... -p calib_yaml:=...
```

## 9. Troubleshooting
- Ensure wildcard paths are quoted.
- Reduce min_pair_inliers for sparse matches.
- Use images with sufficient parallax.

## 10. License
MIT License
