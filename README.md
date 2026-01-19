# nav2-spiral-search

Vision-guided spiral search and cone approach behavior built on ROS 2 Nav2.

## Overview

This repository implements an autonomous target discovery and approach system
using ROS 2 Nav2. A spiral search pattern is executed by sending sequential Nav2
waypoint goals, allowing the robot to explore the environment safely while
leveraging Nav2 planning and obstacle avoidance.

Once a cone is detected, the system transitions to an approach behavior that
converts the estimated cone pose into a Nav2 navigation goal.

## System Architecture

Camera → Cone Detection → Target Pose Estimation → Behavior Control → Nav2

High-level behavior is managed by a finite state machine, while all robot motion
is executed through Nav2 goal-based navigation.

## Core Components

- **spiral_search_node**  
  Generates a sequence of Nav2 waypoint goals forming an expanding spiral
  pattern for environment exploration.

- **cone_detection_pose_node**  
  Detects cones and estimates their pose relative to the robot base frame.

- **cone_approach_node**  
  Sends Nav2 goals to approach the detected cone while respecting obstacle
  avoidance and controller constraints.

## Behavior Logic

The system operates using three primary states:
- **SEARCH**: Execute spiral waypoint navigation
- **APPROACH**: Navigate to detected cone pose
- **STOP**: Halt after successful approach

State transitions are driven by perception feedback and Nav2 task status.

## Requirements

- ROS 2 Humble
- Nav2
- OpenCV

## Running

```bash
colcon build
source install/setup.bash
ros2 launch nav2_spiral_search nav2_spiral_search.launch.py
