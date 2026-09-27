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

<img width="1472" height="1052" alt="image" src="https://github.com/user-attachments/assets/d7180ba0-9d1a-42a4-9fd2-2d3dde2a5e52" />


**State machine overview:** HOME → GO_TO_KITCHEN → WAIT_KITCHEN_CONFIRM → (confirmed) → NEXT_TABLE → GO_TO_TABLE → WAIT_TABLE_CONFIRM → (confirmed) → NEXT_TABLE (loop until queue empty) → RETURN_HOME. Any timeout or cancellation routes through RETURN_VIA_KITCHEN before RETURN_HOME. Any navigation failure routes to ORDER_FAILED.

**Key design decision — genericity over hardcoding:** every navigation call (to kitchen, to any table, back home) goes through a single reusable `NavHelper.go_to(waypoint_name)` method that sends a `NavigateToPose` action goal to Nav2 and blocks until success/failure. Table identity is passed as **state machine userdata** (`table_id`/`current_table`), not branched on in code — so `table1`, `table2`, `table3` are just different waypoint keys, and adding a `table4` requires zero code changes, only a new entry in `waypoints.py`.

Confirmation waiting follows the same principle: a single reusable `ConfirmListener` class subscribes to any given topic and blocks (with a timeout) until it receives a positive confirmation. It's parameterized by topic name — `WaitKitchenConfirm` uses a fixed `/kitchen/confirm` topic, while `WaitTableConfirm` dynamically builds `/{table_id}/confirm` at runtime, so table1/table2/table3 confirmation all reuse the same class and state logic with zero duplication.

**Fallback routing:** a timeout at the kitchen returns the robot straight home. A timeout at the table routes the robot back through the kitchen first (`RETURN_VIA_KITCHEN`), then home — matching the assessment's specified behavior for each case.

**Cancellation:** a `CancelListener` (same reusable pattern as `ConfirmListener`) watches `/order/cancel` for the entire order lifecycle. `NavHelper.go_to()` polls Nav2's action result instead of blocking on it, so a cancellation can interrupt an in-progress navigation leg and cleanly cancel the Nav2 goal, rather than only being checked between legs. A cancellation while en route to the kitchen sends the robot straight home; a cancellation while en route to a table routes it back through the kitchen first (reusing `RETURN_VIA_KITCHEN` from the confirmation-timeout logic), then home — again matching the assessment's specified behavior.

**Arrival visibility:** after each successful navigation leg, the robot holds position for 2 seconds (`ARRIVAL_PAUSE_SEC` in `NavHelper.go_to()`) before proceeding. This makes each arrival unambiguous when watching or recording the simulation — without it, the robot could appear to glide continuously through waypoints with no clear indication of having reached one.

**Localization:** AMCL's `set_initial_pose` and `initial_pose` are preset in `nav2_params.yaml` to match the robot's fixed spawn point in the Gazebo world (the same coordinates as the `home` waypoint). This means the robot localizes automatically on launch, with no manual "2D Pose Estimate" click required in RViz — one less manual step for anyone running or reviewing the demo.

**Orientation handling:** only the `home` waypoint enforces a fixed final orientation (so the robot looks consistently "docked" between orders). Kitchen and table waypoints don't force a specific heading — combined with `use_final_approach_orientation: true` in the Nav2 controller params, the robot settles smoothly into whatever direction it was already driving on arrival, instead of stopping and snapping to an arbitrary fixed direction.

**Skip-and-continue with mandatory kitchen return (Milestone 6):** `WaitTableConfirm` waits up to 15 seconds for confirmation at each table, but unlike the single-table flow, both a confirmation and a timeout lead to the same outcome — the delivery loop simply continues to the next table. A timeout is treated as "skip this table," not a failure. Once all tables in the queue have been attempted, `NEXT_TABLE` always routes through `RETURN_VIA_KITCHEN` before `RETURN_HOME`, regardless of how each table's confirmation went — this matches the assessment spec precisely, which (unlike Milestone 5) requires a kitchen stop at the end of every multi-table run. Note that, per the spec, no confirmation step is required at the kitchen for the multi-table milestones (5, 6, 7) — only at each table.

**Cancel-then-return timing fix:** cancelling navigation and immediately sending the next goal (e.g. cancel en route to a table, then navigate to the kitchen) could cause Nav2 to instantly abort the new goal, because its action server hadn't finished releasing the cancelled one yet. Fixed with a short settle delay (`CANCEL_SETTLE_SEC`) after a cancellation is confirmed, before the state machine's next navigation call.

**Per-table cancellation (Milestone 7):** a `TableCancelRegistry` subscribes to `/{table}/cancel` for every table in the run's queue, tracking cancellation independently per table — distinct from `CancelListener`, which aborts the entire order. `NextTable` skips any table already cancelled before departure; `GoToTable` also aborts mid-flight if that specific table's order is cancelled while en route, then continues the loop to the next table rather than aborting the whole run.

**Design decision — conditional kitchen routing at the end of a multi-table run:** the assessment spec states, for Milestones 6 and 7, that "after finishing the delivery of the final table, the robot goes to the kitchen before going to the home position." Every example given for these milestones happens to involve a skipped or cancelled table, so the spec doesn't explicitly define behavior for a fully clean multi-table run (every table confirmed, nothing skipped). We interpreted the kitchen stop as conditional on at least one skip or cancellation having occurred during the run — reasoning that the stop plausibly exists so the robot can return any uncollected/unused item to the kitchen, which wouldn't apply to a run where every delivery succeeded cleanly. A clean run therefore returns straight home, matching Milestone 5's behavior; a run with any skip or cancellation routes via the kitchen first, matching the literal M6/M7 text. This is a documented judgment call on an ambiguous case, not a deviation from an explicit requirement — the alternative (always via kitchen, regardless of outcome) is one line to restore if a strictly literal reading is preferred.

**Nav2 orientation tuning:** the default TurtleBot3 Nav2 params include a `RotateToGoal` trajectory critic and a `yaw_goal_tolerance` on the goal checker, both of which force the robot to rotate in place to match a specific final heading before a goal is considered reached — even at waypoints where no specific orientation matters. Since only the `home` waypoint needs a consistent final orientation, `RotateToGoal` was removed from the active critics list, `GoalAlign.scale` was set to `0.0`, and `general_goal_checker.yaw_goal_tolerance` was loosened, so kitchen/table arrivals complete as soon as the robot reaches position, without an unnecessary spin-in-place.

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

### Running a multi-table order (Milestones 5–7)

Pass multiple table names as CLI arguments — the robot visits each in order:
```bash
ros2 run goat_butler_robot butler_state_machine table1 table2 table3
```

For Milestone 6 behavior, confirm or ignore each table as it's reached, to test skip-and-continue:
```bash
ros2 topic pub --once /table1/confirm std_msgs/msg/Bool "{data: true}"
# leave table2 unconfirmed — the robot waits 15s, then skips it and moves on
ros2 topic pub --once /table3/confirm std_msgs/msg/Bool "{data: true}"
```
The robot will always route through the kitchen once all tables have been attempted, then return home.

### Simulating per-table cancellation (Milestone 7)

To cancel one specific table's order — either before the robot departs for it, or while it's en route — publish to that table's own cancel topic:
```bash
ros2 topic pub --once /table2/cancel std_msgs/msg/Bool "{data: true}"
```
The robot skips only that table and continues delivering to the rest of the queue. If any table was skipped or cancelled during the run, the robot routes via the kitchen before returning home; a fully clean run (every table confirmed) returns straight home.

## Milestones

| # | Description | Status |
|---|---|---|
| 1 | Single order: home → kitchen → table → home, no confirmation | ✅ Complete |
| 2 | Wait for confirmation at kitchen, timeout → return home | ✅ Complete |
| 3 | Confirmation handling at both kitchen and table with fallback routing | ✅ Complete |
| 4 | Order cancellation mid-route | ✅ Complete |
| 5 | Multiple simultaneous orders across tables | ✅ Complete |
| 6 | Multi-order with no confirmation at one table (skip and continue) | ✅ Complete |
| 7 | Multi-order with cancellation of one table mid-route | ✅ Complete |

## Demo

The following videos demonstrate each completed milestone of the Goat Butler Robot.

[▶️ Milestone 1 Demo Video](https://drive.google.com/file/d/1-HTyKQ74O6ooTu3HuX1iJB4o0SWeFfJf/view?usp=drive_link)

[▶️ Milestone 2 Demo Video](https://drive.google.com/file/d/1i2lGJgXutsLFVUqwEhDK2KUCTCJjOjZk/view?usp=drive_link)

[▶️ Milestone 3 Demo Video](https://drive.google.com/file/d/1COn4YoiQpL7wc4BLA1u53EOtKGrdKqSu/view?usp=drive_link)

[▶️ Milestone 4 Demo Video](https://drive.google.com/file/d/10Y7ooquQ3_nhR97tYfji5vdCAN6F5i5R/view?usp=drive_link)

[▶️ Milestone 5 Demo Video](https://drive.google.com/file/d/1D6VxWmsI3y3cJDqeCJnEEYfqIkRh1dai/view?usp=drive_link)

[▶️ Milestone 6 Demo Video](https://drive.google.com/file/d/1xj8oyPa79WeCi5jYk0X7QB-Tgt693122/view?usp=drive_link)

[▶️ Milestone 7 Demo Video](https://drive.google.com/file/d/1PWxdecU_l3_9OCfTMLPUm3lgKuny9CTc/view?usp=drive_link)

## Notes

This was developed and tested in simulation (TurtleBot3 + Gazebo) rather than on physical hardware, as the assessment focuses on ROS architecture and state-machine design rather than a specific robot platform. The design generalizes directly to real navigation stacks or other differential-drive robots with minimal changes.
