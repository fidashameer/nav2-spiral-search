# System Architecture

This package implements a vision-guided spiral search and cone approach behavior
as a modular layer on top of ROS 2 Nav2.

## High-Level Data Flow

Camera → Cone Detection → Target Pose Estimation → Behavior Control → Nav2

## Core Nodes

### spiral_search_node
Executes a spiral motion pattern to explore the environment when no target
is detected. The spiral radius increases over time to ensure area coverage.

### cone_detection_pose_node
Processes camera input to detect cones and estimates the target pose
relative to the robot base frame.

### cone_approach_node
Consumes the detected target pose and commands Nav2 to approach the cone
while respecting obstacle avoidance and controller constraints.

## Behavior Logic

The system transitions between search and approach modes based on perception
feedback and Nav2 task execution status.
