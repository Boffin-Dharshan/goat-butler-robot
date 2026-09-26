import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import smach
import sys
import time

from goat_butler_robot.waypoints import WAYPOINTS

CONFIRM_TIMEOUT_SEC = 15.0


class NavHelper:
    def __init__(self, node: Node):
        self.node = node
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def go_to(self, waypoint_name: str) -> bool:
        x, y, yaw = WAYPOINTS[waypoint_name]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.node.get_logger().info(f'Navigating to {waypoint_name} ({x}, {y})')
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().warn(f'Goal to {waypoint_name} REJECTED')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        status = result_future.result().status
        success = (status == 4)
        self.node.get_logger().info(f'Arrived at {waypoint_name}: {"OK" if success else f"FAILED (status={status})"}')
        return success


class ConfirmListener:
    """Reusable for any location's confirm topic — kitchen, table1, table2, table3."""
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
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.nav = nav

    def execute(self, userdata):
        return 'arrived' if self.nav.go_to('kitchen') else 'failed'


class WaitKitchenConfirm(smach.State):
    def __init__(self, listener: ConfirmListener):
        smach.State.__init__(self, outcomes=['confirmed', 'timeout'])
        self.listener = listener

    def execute(self, userdata):
        return 'confirmed' if self.listener.wait(CONFIRM_TIMEOUT_SEC) else 'timeout'


class GoToTable(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['arrived', 'failed'], input_keys=['table_id'])
        self.nav = nav

    def execute(self, userdata):
        return 'arrived' if self.nav.go_to(userdata.table_id) else 'failed'


class WaitTableConfirm(smach.State):
    """Creates a fresh ConfirmListener scoped to THIS table_id, so table1/2/3 don't need separate states."""
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
        return 'done' if self.nav.go_to('home') else 'failed'


class ReturnViaKitchen(smach.State):
    """Table-side timeout: go back to kitchen first, THEN home — per milestone 3b."""
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['at_kitchen', 'failed'])
        self.nav = nav

    def execute(self, userdata):
        return 'at_kitchen' if self.nav.go_to('kitchen') else 'failed'


def build_state_machine(nav: NavHelper, kitchen_listener: ConfirmListener, node: Node, table_id: str):
    sm = smach.StateMachine(outcomes=['order_complete', 'order_failed'])
    sm.userdata.table_id = table_id
    with sm:
        smach.StateMachine.add(
            'GO_TO_KITCHEN', GoToKitchen(nav),
            transitions={'arrived': 'WAIT_KITCHEN_CONFIRM', 'failed': 'order_failed'}
        )
        smach.StateMachine.add(
            'WAIT_KITCHEN_CONFIRM', WaitKitchenConfirm(kitchen_listener),
            transitions={'confirmed': 'GO_TO_TABLE', 'timeout': 'RETURN_HOME'}
        )
        smach.StateMachine.add(
            'GO_TO_TABLE', GoToTable(nav),
            transitions={'arrived': 'WAIT_TABLE_CONFIRM', 'failed': 'order_failed'}
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
    nav = NavHelper(node)
    kitchen_listener = ConfirmListener(node, '/kitchen/confirm')

    table_id = sys.argv[1] if len(sys.argv) > 1 else 'table1'
    sm = build_state_machine(nav, kitchen_listener, node, table_id)
    outcome = sm.execute()

    node.get_logger().info(f'State machine finished: {outcome}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()