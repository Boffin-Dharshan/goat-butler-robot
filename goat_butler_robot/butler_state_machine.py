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

from goat_butler_robot.waypoints import WAYPOINTS

CONFIRM_TIMEOUT_SEC = 15.0


class CancelListener:
    """Reusable — watches a cancel topic in the background for the whole order lifecycle."""
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
    def __init__(self, node: Node, cancel_listener: 'CancelListener'):
        self.node = node
        self.cancel_listener = cancel_listener
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def go_to(self, waypoint_name: str) -> str:
        """Returns 'success', 'failed', or 'cancelled'."""
        self.cancel_listener.reset()   # <-- ADD THIS LINE: fresh cancel state for this leg

        x, y, yaw = WAYPOINTS[waypoint_name]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
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
        return 'success' if success else 'failed'


class ConfirmListener:
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


class GoToKitchen(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['arrived', 'failed', 'cancelled'])
        self.nav = nav

    def execute(self, userdata):
        return {'success': 'arrived', 'failed': 'failed', 'cancelled': 'cancelled'}[self.nav.go_to('kitchen')]


class WaitKitchenConfirm(smach.State):
    def __init__(self, listener: ConfirmListener):
        smach.State.__init__(self, outcomes=['confirmed', 'timeout'])
        self.listener = listener

    def execute(self, userdata):
        return 'confirmed' if self.listener.wait(CONFIRM_TIMEOUT_SEC) else 'timeout'


class GoToTable(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['arrived', 'failed', 'cancelled'], input_keys=['table_id'])
        self.nav = nav

    def execute(self, userdata):
        return {'success': 'arrived', 'failed': 'failed', 'cancelled': 'cancelled'}[self.nav.go_to(userdata.table_id)]


class WaitTableConfirm(smach.State):
    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=['confirmed', 'timeout'], input_keys=['table_id'])
        self.node = node

    def execute(self, userdata):
        listener = ConfirmListener(self.node, f'/{userdata.table_id}/confirm')
        return 'confirmed' if listener.wait(CONFIRM_TIMEOUT_SEC) else 'timeout'


class ReturnHome(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.nav = nav

    def execute(self, userdata):
        # Home leg is never itself cancellable mid-flight per spec — treat cancelled same as failed here
        result = self.nav.go_to('home')
        return 'done' if result == 'success' else 'failed'


class ReturnViaKitchen(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['at_kitchen', 'failed'])
        self.nav = nav

    def execute(self, userdata):
        result = self.nav.go_to('kitchen')
        return 'at_kitchen' if result == 'success' else 'failed'


def build_state_machine(nav: NavHelper, kitchen_listener: ConfirmListener, node: Node, table_id: str):
    sm = smach.StateMachine(outcomes=['order_complete', 'order_failed'])
    sm.userdata.table_id = table_id
    with sm:
        smach.StateMachine.add(
            'GO_TO_KITCHEN', GoToKitchen(nav),
            transitions={
                'arrived': 'WAIT_KITCHEN_CONFIRM',
                'failed': 'order_failed',
                'cancelled': 'RETURN_HOME',       # cancelled en route to kitchen -> straight home
            }
        )
        smach.StateMachine.add(
            'WAIT_KITCHEN_CONFIRM', WaitKitchenConfirm(kitchen_listener),
            transitions={'confirmed': 'GO_TO_TABLE', 'timeout': 'RETURN_HOME'}
        )
        smach.StateMachine.add(
            'GO_TO_TABLE', GoToTable(nav),
            transitions={
                'arrived': 'WAIT_TABLE_CONFIRM',
                'failed': 'order_failed',
                'cancelled': 'RETURN_VIA_KITCHEN', # cancelled en route to table -> kitchen first, then home
            }
        )
        smach.StateMachine.add(
            'WAIT_TABLE_CONFIRM', WaitTableConfirm(node),
            transitions={'confirmed': 'RETURN_HOME', 'timeout': 'RETURN_VIA_KITCHEN'}
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
    kitchen_listener = ConfirmListener(node, '/kitchen/confirm')

    table_id = sys.argv[1] if len(sys.argv) > 1 else 'table1'
    sm = build_state_machine(nav, kitchen_listener, node, table_id)
    outcome = sm.execute()

    node.get_logger().info(f'State machine finished: {outcome}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()