import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import smach
import sys

from goat_butler_robot.waypoints import WAYPOINTS


class NavHelper:
    """Single reusable nav function — every state calls this, nothing hardcoded per-location."""
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
        goal.pose.pose.orientation.z = yaw  # simplify for now; use quaternion conversion for real yaw
        goal.pose.pose.orientation.w = 1.0

        self.node.get_logger().info(f'Navigating to {waypoint_name} ({x}, {y})')
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        return True  # refine with actual result.status check later


class GoToKitchen(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['arrived'])
        self.nav = nav

    def execute(self, userdata):
        self.nav.go_to('kitchen')
        return 'arrived'


class GoToTable(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['arrived'], input_keys=['table_id'])
        self.nav = nav

    def execute(self, userdata):
        self.nav.go_to(userdata.table_id)
        return 'arrived'


class ReturnHome(smach.State):
    def __init__(self, nav: NavHelper):
        smach.State.__init__(self, outcomes=['done'])
        self.nav = nav

    def execute(self, userdata):
        self.nav.go_to('home')
        return 'done'


def build_state_machine(nav: NavHelper, table_id: str):
    sm = smach.StateMachine(outcomes=['order_complete'])
    sm.userdata.table_id = table_id
    with sm:
        smach.StateMachine.add('GO_TO_KITCHEN', GoToKitchen(nav), transitions={'arrived': 'GO_TO_TABLE'})
        smach.StateMachine.add('GO_TO_TABLE', GoToTable(nav), transitions={'arrived': 'RETURN_HOME'})
        smach.StateMachine.add('RETURN_HOME', ReturnHome(nav), transitions={'done': 'order_complete'})
    return sm


def main():
    rclpy.init()
    node = rclpy.create_node('butler_state_machine')
    nav = NavHelper(node)

    table_id = sys.argv[1] if len(sys.argv) > 1 else 'table1'
    sm = build_state_machine(nav, table_id)
    outcome = sm.execute()

    node.get_logger().info(f'State machine finished: {outcome}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
