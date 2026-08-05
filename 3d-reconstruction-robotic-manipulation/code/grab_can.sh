#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 0) Open the gripper
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.15, max_effort: 10.0}}"

# 1) Move arm from RViz pose downwards
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory:
    joint_names:
      - link1_to_link2
      - link2_to_link3
      - link3_to_link4
      - link4_to_link5
      - link5_to_link6
      - link6_to_link6_flange
    points:
      - positions: [0.05313196837586918,
                    -2.089075292803585,
                    1.4114916117764384,
                    -0.10316444137369946,
                    -0.0411372648191398,
                    0.0016142337585524542]
        time_from_start:
          sec: 0
          nanosec: 0
      - positions: [0.05313196837586918,
                    -2.25,
                    1.60,
                    -0.20,
                    -0.0411372648191398,
                    0.0016142337585524542]
        time_from_start:
          sec: 3
          nanosec: 0"

# 2) Close the gripper to grab the can
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: -0.7, max_effort: 20.0}}"

# 3) Move arm (with can) to a new place position
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "trajectory:
    joint_names:
      - link1_to_link2
      - link2_to_link3
      - link3_to_link4
      - link4_to_link5
      - link5_to_link6
      - link6_to_link6_flange
    points:
      - positions: [0.05313196837586918,
                    -2.25,
                    1.60,
                    -0.20,
                    -0.0411372648191398,
                    0.0016142337585524542]
        time_from_start:
          sec: 0
          nanosec: 0
      - positions: [0.7145706256776138,
                    -0.6969414725326833,
                    -2.1008752276363154,
                    2.0348249351141043,
                    -0.5599176487506811,
                    0.4699257659124782]
        time_from_start:
          sec: 4
          nanosec: 0"

# 4) Open the gripper to drop the can
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.15, max_effort: 10.0}}"
