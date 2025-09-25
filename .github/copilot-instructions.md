# Copilot Instructions for RoboBehaviors-and-Finite-State-Machines-Project

## Project Overview

This project implements a modular ROS 2 (Humble) robotics stack for the Neato
platform, focused on composing multiple robot behaviors (e.g., teleop, shape
driving, people following) into a finite-state machine (FSM) controller. The FSM
supervises subprocesses for each behavior and manages transitions based on
sensor input and timers.

## Architecture & Key Components

- **FSM Supervisor:**

  - `ros_behaviors_fsm/ros_behaviors_fsm/finite_state_controller.py`
    - Main ROS 2 node (`BehaviorFSM`) that launches and kills subprocesses for
      each behavior (e.g., `draw_pentagon`, `person_follower`, `spin_360`).
    - FSM states: PENTAGON, FOLLOW, SPIN, IDLE.
    - Transitions are triggered by events (e.g., bump, lost target, estop).
    - Publishes current FSM state to `/fsm/state` (`std_msgs/String`).

- **Behaviors:**

  - Each behavior is a standalone ROS 2 node, launched as a subprocess by the
    FSM.
  - Example behaviors: `draw_pentagon`, `spin_360`, `person_follower`.
  - Entry points are defined in `setup.py` as console scripts.

- **Launch Files:**

  - Located in `launch/` (e.g., `fsm.launch.py`).
  - Launch the FSM and optionally other nodes/tools.
  - All launch files must be installed via `setup.py` (`data_files`).

- **Parameters:**

  - Configurable via YAML (`config/params.yaml`) and/or launch files.
  - Key parameters: thresholds, timeouts, topic names.

- **State Diagrams:**
  - Maintained in both Mermaid format (README) and QRT comments in code for
    `qrt_graph` visualization.

## Developer Workflows

- **Build:**

  ```bash
  colcon build --packages-select ros_behaviors_fsm
  source install/setup.bash
  ```

- **Run FSM:**

  ```bash
  ros2 launch ros_behaviors_fsm fsm.launch.py use_sim_time:=false
  ```

- **Run Individual Behaviors:**

  ```bash
  ros2 run ros_behaviors_fsm draw_pentagon
  ros2 run ros_behaviors_fsm spin_360
  ros2 run ros_behaviors_fsm person_follower
  ```

- **Monitor FSM State:**

  ```bash
  ros2 topic echo /fsm/state
  ```

- **Visualize FSM Structure:**
  - Use `qrt_graph` on `finite_state_controller.py` (requires QRT comments).

## Project-Specific Patterns & Conventions

- **FSM transitions and states are documented with `# QRT:` comments** for
  compatibility with `qrt_graph`.
- **Behavior switching is done via subprocess management** (not ROS 2 lifecycle
  or composition).
- **All ROS 2 nodes and behaviors are started via launch files or console
  scripts.**
- **Topic and parameter names are consistent and set via launch/YAML.**
- **No custom message types; uses standard ROS 2 messages.**
- **Debugging and visualization are done with RViz and `ros2 topic echo`.**

## Integration & External Dependencies

- **Depends on ROS 2 Humble, geometry_msgs, sensor_msgs, std_msgs,
  gazebo_msgs.**
- **Requires `qrt_graph` for FSM diagram generation (QRT comments).**
- **Behavior nodes must be present and registered as console scripts in
  `setup.py`.**

## Key Files & Directories

- `ros_behaviors_fsm/ros_behaviors_fsm/finite_state_controller.py` — FSM logic
  and state publishing
- `ros_behaviors_fsm/launch/` — Launch files for FSM and behaviors
- `ros_behaviors_fsm/config/params.yaml` — Parameter configuration
- `README.md` — Project overview, state diagram, and usage instructions

---

# Project Completeness Checklist

Based on your codebase, README, and assignment requirements, here are the files
and functionality you should have for a complete submission:

## Required Files/Structure

- `ros_behaviors_fsm/ros_behaviors_fsm/finite_state_controller.py`
  - FSM logic (present)
- `ros_behaviors_fsm/ros_behaviors_fsm/teleop.py`
  - Teleop node (should exist if you implemented teleop)
- `ros_behaviors_fsm/ros_behaviors_fsm/drive_square.py`
  - Drive square node (should exist)
- `ros_behaviors_fsm/ros_behaviors_fsm/person_follower.py`
  - Person following node (should exist)
- `ros_behaviors_fsm/ros_behaviors_fsm/spin_360.py`
  - 360 spin node (should exist)
- `ros_behaviors_fsm/ros_behaviors_fsm/draw_pentagon.py`
  - Pentagon node (should exist if referenced)
- `ros_behaviors_fsm/ros_behaviors_fsm/wall_follower.py`
  - Wall following node (if implemented)
- `ros_behaviors_fsm/launch/fsm.launch.py`
  - Main launch file (should exist)
- `ros_behaviors_fsm/config/params.yaml`
  - Parameter file (should exist)
- `ros_behaviors_fsm/setup.py` and `package.xml`
  - Package setup and dependencies (should exist)
- `README.md`
  - Project documentation (present)
- `bags/`
  - Bag files for demo runs (should exist and contain at least one bag)

## Functionality

- **FSM launches behaviors as subprocesses** (present in
  `finite_state_controller.py`)
- **Behaviors are implemented as ROS 2 nodes** (check that each referenced node
  exists)
- **FSM state is published to `/fsm/state`** (present)
- **QRT comments for FSM structure** (present)
- **Launch files install correctly via `setup.py`** (check `data_files` in
  `setup.py`)
- **All behaviors referenced in FSM exist as scripts and are registered in
  `setup.py`**
- **README includes state diagram, build/run instructions, and behavior
  descriptions**
- **Bag files for at least one demo run are present in `bags/`**
- **Visualization topics for RViz are documented and used**

## Common Omissions to Check

- Missing or unregistered behavior scripts (e.g., `draw_pentagon.py`,
  `spin_360.py`)
- Launch files not installed (missing from `setup.py` `data_files`)
- `params.yaml` missing or not referenced in launch files
- Bag files not pushed to repo
- Incomplete README (missing diagrams, explanations, or assignment link)

---

**Action:**

- Double-check that all referenced behavior scripts and launch files exist and
  are installed.
- Ensure all behaviors are registered as console scripts in `setup.py`.
- Make sure your `bags/` directory contains at least one demo bag file.
- Complete all placeholder sections in your README.

If you want a script to check for missing files or want to see a sample
`setup.py` or launch file, let me know!
