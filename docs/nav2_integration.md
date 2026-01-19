# Nav2 Integration

This package builds directly on ROS 2 Nav2 for both spiral search and target
approach behaviors. All robot motion is executed through Nav2 goal-based
navigation rather than direct velocity control.

## Why Nav2

Nav2 provides a robust, modular navigation framework with global planning,
local control, and obstacle avoidance. By expressing both search and approach
as navigation goals, the system maintains consistent safety and behavior
across different operating modes.

Key benefits:
- Unified motion execution through Nav2
- Pluggable planners and controllers
- Built-in recovery behaviors and failure handling

## Spiral Search via Nav2 Goals

The spiral search behavior is implemented by generating a sequence of waypoint
goals that form an expanding spiral around the robot’s start position.

These waypoints are sent to Nav2 sequentially using a waypoint follower
interface. Nav2 is responsible for:
- Path planning between waypoints
- Obstacle avoidance
- Velocity and acceleration limits

This approach allows the search pattern to remain planner- and controller-
agnostic.

## Cone Approach via Nav2 Goals

Once a cone is detected, the estimated cone pose is converted into a Nav2 goal.
Nav2 executes the approach while handling obstacle avoidance and goal tolerance
checking.

The same planners and controllers are reused for both search and approach,
ensuring consistent behavior.

## Planner and Controller Configuration

Typical configurations include:
- Global planner: NavFn or Smac
- Local controller: DWB or MPPI

Planner and controller selection is handled purely through configuration files,
allowing behavior logic to remain unchanged.

## Failure Handling and Recovery

Nav2 task status is monitored to detect:
- Goal completion
- Aborted or failed navigation
- Timeout or excessive deviation

On failure or loss of target, the system transitions back to the spiral search
state and continues exploration.

## Design Rationale

Treating Nav2 as the sole motion execution backend simplifies the system
architecture, improves safety, and reduces duplicated control logic. High-level
behavior is expressed entirely through goal sequencing rather than low-level
motion commands.
