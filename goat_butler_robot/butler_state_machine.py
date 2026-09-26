import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus
import smach
import sys
import time
import math

from goat_butler_robot.waypoints import WAYPOINTS

ARRIVAL_PAUSE_SEC = 2.0  # brief pause after each successful arrival so it's visually clear in Gazebo/demo


def yaw_to_quaternion(yaw: float):
    """Converts a yaw angle (radians) to a quaternion's z, w components (planar rotation only)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CancelListener:
    """Watches a cancel topic for the whole order lifecycle. Reusable across every nav leg."""
    def __init__(self, node: Node, topic: str = '/order/cancel'):
        self.node = node
        self.cancelled = False
        self.sub = node.create_subscription(Bool, topic, self._callback, 10)

    def _callback(self, msg: Bool):
        if msg.data:
            self.cancelled = True

    def reset(self):
        self.cancelled = False


class NavHelper:
    """Single reusable nav function — every state calls this, nothing hardcoded per-location."""
    def __init__(self, node: Node, cancel_listener: 'CancelListener'):
        self.node = node
        self.cancel_listener = cancel_listener
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def go_to(self, waypoint_name: str) -> str:
        """Returns 'success', 'failed', or 'cancelled'."""
        self.cancel_listener.reset()
        x, y, yaw = WAYPOINTS[waypoint_name]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        if waypoint_name == 'home':
            # Only home enforces a fixed "parked" orientation, using its stored yaw.
            qz, qw = yaw_to_quaternion(yaw)
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw
        else:
            # Kitchen/tables: no fixed final heading forced here. Combined with
            # use_final_approach_orientation:true in nav2_params.yaml, the robot
            # simply stops facing however it approached, instead of wasting time
            # spinning in place to match an arbitrary orientation.
            goal.pose.pose.orientation.z = 0.0
            goal.pose.pose.orientation.w = 1.0

        self.node.get_logger().info(f'Navigating to {waypoint_name} ({x}, {y})')
        self.client.wait_for_server()
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().warn(f'Goal to {waypoint_name} REJECTED')
            return 'failed'

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.cancel_listener.cancelled:
                self.node.get_logger().warn(f'Cancellation received while navigating to {waypoint_name} — aborting goal')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, cancel_future)
                return 'cancelled'

        status = result_future.result().status
        success = (status == GoalStatus.STATUS_SUCCEEDED)
        self.node.get_logger().info(f'Arrived at {waypoint_name}: {"OK" if success else f"FAILED (status={status})"}')
        if success:
            self.node.get_logger().info(f'Holding at {waypoint_name} for {ARRIVAL_PAUSE_SEC}s...')
            time.sleep(ARRIVAL_PAUSE_SEC)
        return 'success' if success else 'failed'


class GoToKitchen(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['arrived', 'failed', 'cancelled'])
        self.nav = nav

    def execute(self, userdata):
        return {'success': 'arrived', 'failed': 'failed', 'cancelled': 'cancelled'}[self.nav.go_to('kitchen')]


class NextTable(smach.State):
    """Pops the next table off the queue. If queue is empty, all deliveries are done -> go home."""
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['next', 'queue_empty'],
            input_keys=['table_queue'],
            output_keys=['table_queue', 'current_table']
        )

    def execute(self, userdata):
        if not userdata.table_queue:
            return 'queue_empty'
        userdata.current_table = userdata.table_queue.pop(0)
        userdata.table_queue = userdata.table_queue  # SMACH needs explicit reassignment to propagate
        return 'next'


class GoToTable(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['arrived', 'failed', 'cancelled'], input_keys=['current_table'])
        self.nav = nav

    def execute(self, userdata):
        return {'success': 'arrived', 'failed': 'failed', 'cancelled': 'cancelled'}[self.nav.go_to(userdata.current_table)]


class ReturnHome(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.nav = nav

    def execute(self, userdata):
        result = self.nav.go_to('home')
        return 'done' if result == 'success' else 'failed'


class ReturnViaKitchen(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['at_kitchen', 'failed'])
        self.nav = nav

    def execute(self, userdata):
        result = self.nav.go_to('kitchen')
        return 'at_kitchen' if result == 'success' else 'failed'


def build_state_machine(nav: NavHelper, table_queue: list):
    sm = smach.StateMachine(outcomes=['order_complete', 'order_failed'])
    sm.userdata.table_queue = list(table_queue)  # copy, so caller's list isn't mutated
    sm.userdata.current_table = None

    with sm:
        smach.StateMachine.add(
            'GO_TO_KITCHEN', GoToKitchen(nav),
            transitions={'arrived': 'NEXT_TABLE', 'failed': 'order_failed', 'cancelled': 'RETURN_HOME'}
        )
        smach.StateMachine.add(
            'NEXT_TABLE', NextTable(),
            transitions={'next': 'GO_TO_TABLE', 'queue_empty': 'RETURN_HOME'}
        )
        smach.StateMachine.add(
            'GO_TO_TABLE', GoToTable(nav),
            transitions={'arrived': 'NEXT_TABLE', 'failed': 'order_failed', 'cancelled': 'RETURN_VIA_KITCHEN'}
        )
        smach.StateMachine.add(
            'RETURN_VIA_KITCHEN', ReturnViaKitchen(nav),
            transitions={'at_kitchen': 'RETURN_HOME', 'failed': 'order_failed'}
        )
        smach.StateMachine.add(
            'RETURN_HOME', ReturnHome(nav),
            transitions={'done': 'order_complete', 'failed': 'order_failed'}
        )
    return sm


def main():
    rclpy.init()
    node = rclpy.create_node('butler_state_machine')
    cancel_listener = CancelListener(node, '/order/cancel')
    nav = NavHelper(node, cancel_listener)

    # Accept one or more table names as CLI args, e.g.: table1 table2 table3
    table_queue = sys.argv[1:] if len(sys.argv) > 1 else ['table1']

    sm = build_state_machine(nav, table_queue)
    outcome = sm.execute()

    node.get_logger().info(f'State machine finished: {outcome}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()