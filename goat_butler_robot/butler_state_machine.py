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

ARRIVAL_PAUSE_SEC = 2.0
CANCEL_SETTLE_SEC = 1.0
CONFIRM_TIMEOUT_SEC = 15.0


def yaw_to_quaternion(yaw: float):
    """Converts a yaw angle (radians) to a quaternion's z, w components (planar rotation only)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CancelListener:
    """Watches a single global cancel topic for the whole order. Reusable across every nav leg."""
    def __init__(self, node: Node, topic: str = '/order/cancel'):
        self.node = node
        self.cancelled = False
        self.sub = node.create_subscription(Bool, topic, self._callback, 10)

    def _callback(self, msg: Bool):
        if msg.data:
            self.cancelled = True

    def reset(self):
        self.cancelled = False


class TableCancelRegistry:
    """Subscribes to /{table}/cancel for every table in the initial queue, tracking cancellation
    per table. Used for Milestone 7 (cancel ONE table's order, skip it, continue the others) —
    distinct from CancelListener, which aborts the whole run."""
    def __init__(self, node: Node, table_names: list):
        self.node = node
        self.cancelled = {t: False for t in table_names}
        self._subs = []
        for t in table_names:
            self._subs.append(
                node.create_subscription(Bool, f'/{t}/cancel', self._make_callback(t), 10)
            )

    def _make_callback(self, table: str):
        def callback(msg: Bool):
            if msg.data:
                self.cancelled[table] = True
        return callback

    def is_cancelled(self, table: str) -> bool:
        return self.cancelled.get(table, False)


class ConfirmListener:
    """Reusable confirmation waiter — subscribes to any given topic (e.g. /table1/confirm)."""
    def __init__(self, node: Node, topic: str):
        self.node = node
        self.topic = topic
        self.confirmed = False
        self.sub = node.create_subscription(Bool, topic, self._callback, 10)

    def _callback(self, msg: Bool):
        if msg.data:
            self.confirmed = True

    def wait(self, timeout_sec: float) -> bool:
        self.confirmed = False
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if self.confirmed:
                self.node.get_logger().info(f'Confirmation received on {self.topic}')
                return True
        self.node.get_logger().warn(f'Timeout waiting for confirmation on {self.topic}')
        return False


class NavHelper:
    """Single reusable nav function — every state calls this, nothing hardcoded per-location."""
    def __init__(self, node: Node, cancel_listener: 'CancelListener'):
        self.node = node
        self.cancel_listener = cancel_listener
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def go_to(self, waypoint_name: str, extra_cancel_check=None) -> str:
        """Returns 'success', 'failed', 'cancelled' (global order cancel), or
        'table_cancelled' (this one table's order was cancelled, per extra_cancel_check)."""
        self.cancel_listener.reset()
        x, y, yaw = WAYPOINTS[waypoint_name]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        if waypoint_name == 'home':
            qz, qw = yaw_to_quaternion(yaw)
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw
        else:
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
                self.node.get_logger().warn(f'Global cancellation received while navigating to {waypoint_name} — aborting goal')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, cancel_future)
                time.sleep(CANCEL_SETTLE_SEC)
                return 'cancelled'

            if extra_cancel_check and extra_cancel_check():
                self.node.get_logger().warn(f'Order for {waypoint_name} was cancelled — skipping this table')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, cancel_future)
                time.sleep(CANCEL_SETTLE_SEC)
                return 'table_cancelled'

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
    """Pops the next table off the queue, skipping any table whose order was cancelled
    (Milestone 7). If the queue empties out, the ending depends on whether ANY table was
    skipped or cancelled this run: a clean run (every table confirmed) goes straight home
    (matching Milestone 5's behavior); a run with at least one skip/cancel routes via kitchen
    first (per the M6/M7 spec text), since the robot is assumed to be carrying dishes back
    from a skipped attempt. This is a documented interpretation — see README design notes."""
    def __init__(self, table_cancel_registry: 'TableCancelRegistry'):
        smach.State.__init__(
            self,
            outcomes=['next', 'queue_empty_clean', 'queue_empty_skipped'],
            input_keys=['table_queue', 'any_skipped'],
            output_keys=['table_queue', 'current_table']
        )
        self.registry = table_cancel_registry

    def execute(self, userdata):
        queue = userdata.table_queue
        while queue:
            candidate = queue.pop(0)
            if self.registry.is_cancelled(candidate):
                self.registry.node.get_logger().warn(
                    f'Order for {candidate} was already cancelled before departure — skipping'
                )
                userdata.any_skipped = True
                continue
            userdata.current_table = candidate
            userdata.table_queue = queue
            return 'next'
        userdata.table_queue = queue
        return 'queue_empty_skipped' if userdata.any_skipped else 'queue_empty_clean'


class GoToTable(smach.State):
    def __init__(self, nav: NavHelper, table_cancel_registry: 'TableCancelRegistry'):
        smach.State.__init__(
            self,
            outcomes=['arrived', 'failed', 'cancelled', 'table_cancelled'],
            input_keys=['current_table'],
            output_keys=['any_skipped']
        )
        self.nav = nav
        self.registry = table_cancel_registry

    def execute(self, userdata):
        table = userdata.current_table
        check = lambda: self.registry.is_cancelled(table)
        result = self.nav.go_to(table, extra_cancel_check=check)
        if result == 'table_cancelled':
            userdata.any_skipped = True
        return {
            'success': 'arrived',
            'failed': 'failed',
            'cancelled': 'cancelled',
            'table_cancelled': 'table_cancelled',
        }[result]


class WaitTableConfirm(smach.State):
    """Waits for confirmation at the current table. Whether confirmed or timed out, the
    delivery loop moves on to the next table — a timeout means 'skip this table', not fail.
    A timeout also marks any_skipped, affecting the final routing decision in NextTable."""
    def __init__(self, node: Node):
        smach.State.__init__(
            self,
            outcomes=['next'],
            input_keys=['current_table'],
            output_keys=['any_skipped']
        )
        self.node = node

    def execute(self, userdata):
        listener = ConfirmListener(self.node, f'/{userdata.current_table}/confirm')
        confirmed = listener.wait(CONFIRM_TIMEOUT_SEC)
        if not confirmed:
            self.node.get_logger().warn(f'No confirmation at {userdata.current_table} — skipping, continuing to next table')
            userdata.any_skipped = True
        return 'next'


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


def build_state_machine(nav: NavHelper, node: Node, table_queue: list, table_cancel_registry: 'TableCancelRegistry'):
    sm = smach.StateMachine(outcomes=['order_complete', 'order_failed'])
    sm.userdata.table_queue = list(table_queue)
    sm.userdata.current_table = None
    sm.userdata.any_skipped = False  # tracks whether any table was skipped/cancelled/unconfirmed this run

    with sm:
        smach.StateMachine.add(
            'GO_TO_KITCHEN', GoToKitchen(nav),
            transitions={'arrived': 'NEXT_TABLE', 'failed': 'order_failed', 'cancelled': 'RETURN_HOME'}
        )
        smach.StateMachine.add(
            'NEXT_TABLE', NextTable(table_cancel_registry),
            transitions={
                'next': 'GO_TO_TABLE',
                'queue_empty_clean': 'RETURN_HOME',        # no skips this run -> straight home
                'queue_empty_skipped': 'RETURN_VIA_KITCHEN', # at least one skip/cancel -> via kitchen first
            }
        )
        smach.StateMachine.add(
            'GO_TO_TABLE', GoToTable(nav, table_cancel_registry),
            transitions={
                'arrived': 'WAIT_TABLE_CONFIRM',
                'failed': 'order_failed',
                'cancelled': 'RETURN_VIA_KITCHEN',       # global /order/cancel -> abort whole run
                'table_cancelled': 'NEXT_TABLE',          # this table's order cancelled -> skip, keep going
            }
        )
        smach.StateMachine.add(
            'WAIT_TABLE_CONFIRM', WaitTableConfirm(node),
            transitions={'next': 'NEXT_TABLE'}
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

    table_queue = sys.argv[1:] if len(sys.argv) > 1 else ['table1']
    table_cancel_registry = TableCancelRegistry(node, table_queue)

    sm = build_state_machine(nav, node, table_queue, table_cancel_registry)
    outcome = sm.execute()

    node.get_logger().info(f'State machine finished: {outcome}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()