# Behavior State Machine

The system is governed by a simple finite state machine (FSM) that coordinates
search and approach behaviors based on perception feedback.

## States

### SEARCH
The robot executes a spiral motion pattern to explore the environment when no
cone is detected. The spiral radius increases over time to improve area coverage.

**Entry condition**
- No valid cone detection available

**Exit condition**
- Cone detected with a valid pose estimate

---

### APPROACH
Once a cone is detected, the robot switches to approach mode and commands Nav2
to move toward the target pose while respecting obstacle avoidance constraints.

**Entry condition**
- Valid cone pose available

**Exit conditions**
- Target reached
- Cone lost (returns to SEARCH)

---

### STOP
The robot stops once the target is reached or when an approach task completes
successfully.

**Entry condition**
- Target reached within tolerance

---


## Design Rationale

This FSM keeps the behavior logic simple and modular while allowing Nav2 to
handle motion planning and obstacle avoidance. Separating search and approach
states improves robustness and makes the system easier to extend.
