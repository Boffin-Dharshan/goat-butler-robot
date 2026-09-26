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

<!-- TODO: replace this note with the exported flowchart image once generated in Napkin AI,
     using the same github.com/user-attachments upload method as the Environment Map image below.
     ![State Machine Flowchart](https://github.com/user-attachments/assets/PASTE_ID_HERE) -->

**State machine overview:** HOME → GO_TO_KITCHEN → WAIT_KITCHEN_CONFIRM → (confirmed) → NEXT_TABLE → GO_TO_TABLE → WAIT_TABLE_CONFIRM → (confirmed) → NEXT_TABLE (loop until queue empty) → RETURN_HOME. Any timeout or cancellation routes through RETURN_VIA_KITCHEN before RETURN_HOME. Any navigation failure routes to ORDER_FAILED.

**Key design decision — genericity over hardcoding:** every navigation call (to kitchen, to any table, back home) goes through a single reusable `NavHelper.go_to(waypoint_name)` method that sends a `NavigateToPose` action goal to Nav2 and blocks until success/failure. Table identity is passed as **state machine userdata** (`table_id`/`current_table`), not branched on in code — so `table1`, `table2`, `table3` are just different waypoint keys, and adding a `table4` requires zero code changes, only a new entry in `waypoints.py`.

Confirmation waiting follows the same principle: a single reusable `ConfirmListener` class subscribes to any given topic and blocks (with a timeout) until it receives a positive confirmation. It's parameterized by topic name — `WaitKitchenConfirm` uses a fixed `/kitchen/confirm` topic, while `WaitTableConfirm` dynamically builds `/{table_id}/confirm` at runtime, so table1/table2/table3 confirmation all reuse the same class and state logic with zero duplication.

**Fallback routing:** a timeout at the kitchen returns the robot straight home. A timeout at the table routes the robot back through the kitchen first (`RETURN_VIA_KITCHEN`), then home — matching the assessment's specified behavior for each case.

**Cancellation:** a `CancelListener` (same reusable pattern as `ConfirmListener`) watches `/order/cancel` for the entire order lifecycle. `NavHelper.go_to()` polls Nav2's action result instead of blocking on it, so a cancellation can interrupt an in-progress navigation leg and cleanly cancel the Nav2 goal, rather than only being checked between legs. A cancellation while en route to the kitchen sends the robot straight home; a cancellation while en route to a table routes it back through the kitchen first (reusing `RETURN_VIA_KITCHEN` from the confirmation-timeout logic), then home — again matching the assessment's specified behavior.

**Arrival visibility:** after each successful navigation leg, the robot holds position for 2 seconds (`ARRIVAL_PAUSE_SEC` in `NavHelper.go_to()`) before proceeding. This makes each arrival unambiguous when watching or recording the simulation — without it, the robot could appear to glide continuously through waypoints with no clear indication of having reached one.

**Localization:** AMCL's `set_initial_pose` and `initial_pose` are preset in `nav2_params.yaml` to match the robot's fixed spawn point in the Gazebo world (the same coordinates as the `home` waypoint). This means the robot localizes automatically on launch, with no manual "2D Pose Estimate" click required in RViz — one less manual step for anyone running or reviewing the demo.

**Orientation handling:** only the `home` waypoint enforces a fixed final orientation (so the robot looks consistently "docked" between orders). Kitchen and table waypoints don't force a specific heading — combined with `use_final_approach_orientation: true` in the Nav2 controller params, the robot settles smoothly into whatever direction it was already driving on arrival, instead of stopping and snapping to an arbitrary fixed direction.

**Multi-table delivery (Milestone 5):** a `NextTable` state pops table IDs one at a time from a `table_queue` (populated from CLI args) and loops through `GO_TO_TABLE` until the queue is empty, then routes home. This is the multi-table version of the base delivery workflow and, per the assessment spec for this milestone, does not include confirmation waiting — that returns in Milestone 6, layered on top of this same queue-processing structure without needing to rebuild it.

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
│   ├── waypoints.py            # named locations -> (x, y, yaw) coordinates
│   └── butler_state_machine.py # SMACH states, NavHelper, ConfirmListener, CancelListener, main entry point
├── maps/
│   ├── map.yaml                # bundled map metadata
│   └── map.pgm                 # bundled map image
├── config/
│   └── nav2_params.yaml        # Nav2 params (Humble waffle_pi base + use_final_approach_orientation)
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

# Terminal 2: launch navigation with the bundled map and custom params
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=$(ros2 pkg prefix goat_butler_robot)/share/goat_butler_robot/maps/map.yaml \
  params_file:=$(ros2 pkg prefix goat_butler_robot)/share/goat_butler_robot/config/nav2_params.yaml
# AMCL's initial pose is preset to match the robot's Gazebo spawn point (see Localization
# note below), so it localizes automatically — no manual 2D Pose Estimate click required.

# Terminal 3: run the butler state machine for a given table
ros2 run goat_butler_robot butler_state_machine table1
```

> The launch command above resolves the map and params paths through the installed package share directory, so it works on any machine after `colcon build` — no hardcoded local paths required.

### Simulating confirmations (Milestones 2 & 3)

After the robot reaches the kitchen, it waits up to 15 seconds for confirmation:
```bash
ros2 topic pub --once /kitchen/confirm std_msgs/msg/Bool "{data: true}"
```
If no confirmation is published within 15 seconds, the robot skips the table and returns directly home.

After delivering to a table, it waits up to 15 seconds for confirmation there too:
```bash
ros2 topic pub --once /table1/confirm std_msgs/msg/Bool "{data: true}"
```
(replace `table1` with whichever table was ordered for). If no confirmation is published, the robot returns to the kitchen first, then home.

### Simulating order cancellation (Milestone 4)

At any point while the robot is actively driving to the kitchen or a table, publish:
```bash
ros2 topic pub --once /order/cancel std_msgs/msg/Bool "{data: true}"
```
- Cancelled en route to the kitchen → robot returns straight home
- Cancelled en route to a table → robot returns to the kitchen first, then home

### Running a multi-table order (Milestone 5)

Pass multiple table names as CLI arguments — the robot visits each in order, then returns home once all are delivered, with no confirmation waiting:
```bash
ros2 run goat_butler_robot butler_state_machine table1 table2 table3
```

## Milestones

| # | Description | Status |
|---|---|---|
| 1 | Single order: home → kitchen → table → home, no confirmation | ✅ Complete |
| 2 | Wait for confirmation at kitchen, timeout → return home | ✅ Complete |
| 3 | Confirmation handling at both kitchen and table with fallback routing | ✅ Complete |
| 4 | Order cancellation mid-route | ✅ Complete |
| 5 | Multiple simultaneous orders across tables | ✅ Complete |
| 6 | Multi-order with no confirmation at one table (skip and continue) | ⬜ Planned |
| 7 | Multi-order with cancellation of one table mid-route | ⬜ Planned |

## Demo

The following videos demonstrate each completed milestone of the Goat Butler Robot.

[▶️ Milestone 1 Demo Video](https://drive.google.com/file/d/1-HTyKQ74O6ooTu3HuX1iJB4o0SWeFfJf/view?usp=drive_link)

[▶️ Milestone 2 Demo Video](https://drive.google.com/file/d/1i2lGJgXutsLFVUqwEhDK2KUCTCJjOjZk/view?usp=drive_link)

[▶️ Milestone 3 Demo Video](https://drive.google.com/file/d/1COn4YoiQpL7wc4BLA1u53EOtKGrdKqSu/view?usp=drive_link)

[▶️ Milestone 4 Demo Video](https://drive.google.com/file/d/10Y7ooquQ3_nhR97tYfji5vdCAN6F5i5R/view?usp=drive_link)

[▶️ Milestone 5 Demo Video](https://drive.google.com/file/d/1D6VxWmsI3y3cJDqeCJnEEYfqIkRh1dai/view?usp=drive_link)

## Notes

This was developed and tested in simulation (TurtleBot3 + Gazebo) rather than on physical hardware, as the assessment focuses on ROS architecture and state-machine design rather than a specific robot platform. The design generalizes directly to real navigation stacks or other differential-drive robots with minimal changes.
