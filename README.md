# Goat Butler Robot

A ROS 2 based restaurant delivery robot built for the Goat Robotics ROS Developer assessment. The robot autonomously handles food delivery for a café with 3 tables, replacing a human butler for order collection and delivery.

## Problem Statement

The robot starts at a home position. When an order is received for a given table, it:
1. Navigates to the kitchen to collect the food
2. Navigates to the customer's table to deliver it
3. Returns to the home position

The system must generalize across multiple tables, handle confirmation waits, timeouts, and order cancellations, without hardcoding logic per table.

## Architecture

The robot's behavior is modeled as a **finite state machine** using `smach`/`smach_ros`, driven by real navigation through **Nav2** in a TurtleBot3 Gazebo simulation.

```
[HOME] --order received--> [GO_TO_KITCHEN] --arrived--> [GO_TO_TABLE] --arrived--> [RETURN_HOME] --> [HOME]
                                  |                              |
                                  +---- failed ----> [ORDER_FAILED]
```

**Key design decision — genericity over hardcoding:** every navigation call (to kitchen, to any table, back home) goes through a single reusable `NavHelper.go_to(waypoint_name)` method that sends a `NavigateToPose` action goal to Nav2 and blocks until success/failure. Table identity is passed as **state machine userdata** (`table_id`), not branched on in code — so `table1`, `table2`, `table3` are just different waypoint keys, and adding a `table4` requires zero code changes, only a new entry in `waypoints.py`.

## Tech Stack

- **ROS 2 Humble**
- **SMACH / smach_ros** — state machine framework
- **Nav2** — path planning and navigation
- **TurtleBot3** in **Gazebo** — simulated robot and environment
- **RViz2** — visualization, localization (AMCL), and manual goal testing

## Environment Map

The following map represents the restaurant environment used for the TurtleBot3 simulation. It defines the fixed navigation waypoints for the **home position, kitchen, and three customer tables**.

<img width="403" height="375" alt="Screenshot from 2026-09-26 00-58-35" src="https://github.com/user-attachments/assets/ca47ec63-5390-4aef-89c4-0bb727f246b2" />

### Navigation Waypoints

| Location | Purpose |
|---|---|
| 🏠 Home | Robot starting and return position |
| 🍳 Kitchen | Food collection point |
| Table 1 | Customer delivery point |
| Table 2 | Customer delivery point |
| Table 3 | Customer delivery point |

The robot uses these locations as named navigation waypoints. The corresponding coordinates are stored in `waypoints.py`, allowing the navigation logic to remain generic and independent of individual table IDs.

## Repository Structure

```
goat_butler_robot/
├── goat_butler_robot/
│   ├── __init__.py
│   ├── waypoints.py           # named locations -> (x, y, yaw) coordinates
│   └── butler_state_machine.py # SMACH states, NavHelper, main entry point
├── package.xml
├── setup.py
└── README.md
```

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble installed
- TurtleBot3 and Nav2 packages:
  ```bash
  sudo apt install ros-humble-turtlebot3* ros-humble-turtlebot3-simulations ros-humble-navigation2 ros-humble-nav2-bringup
  export TURTLEBOT3_MODEL=waffle_pi
  echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
  ```
- SMACH:
  ```bash
  sudo apt install ros-humble-smach ros-humble-smach-ros
  ```

## Setup & Running

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select goat_butler_robot
source install/setup.bash

# Terminal 1: launch simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: launch navigation
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=<map.yaml>
# Then set the 2D Pose Estimate in RViz to localize the robot

# Terminal 3: run the butler state machine for a given table
ros2 run goat_butler_robot butler_state_machine table1
```

## Milestones

| # | Description | Status |
|---|---|---|
| 1 | Single order: home → kitchen → table → home, no confirmation | ✅ Complete |
| 2 | Wait for confirmation at kitchen/table, timeout → return home | 🚧 In progress |
| 3 | Confirmation handling at both kitchen and table with fallback routing | ⬜ Planned |
| 4 | Order cancellation mid-route | ⬜ Planned |
| 5 | Multiple simultaneous orders across tables | ⬜ Planned |
| 6 | Multi-order with no confirmation at one table (skip and continue) | ⬜ Planned |
| 7 | Multi-order with cancellation of one table mid-route | ⬜ Planned |

## Demo

The following video demonstrates Milestone 1 of the Goat Butler Robot, including autonomous navigation from the home position to the kitchen, delivery to the selected table, and return to home.

[▶️ Watch Milestone 1 Demo Video]([https://drive.google.com/drive/folders/1XvIMe_udGJ3wtdEZ_UHHXtWO3Cyu0a_A?usp=sharing](https://drive.google.com/file/d/1jyGbYD_MXxP78RoHZrJIueGyyNOyKEY1/view?usp=drive_link))

## Notes

This was developed and tested in simulation (TurtleBot3 + Gazebo) rather than on physical hardware, as the assessment focuses on ROS architecture and state-machine design rather than a specific robot platform. The design generalizes directly to real navigation stacks or other differential-drive robots with minimal changes.
