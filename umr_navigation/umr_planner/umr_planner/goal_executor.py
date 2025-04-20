import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from umr_interfaces.msg import Goals, MotionCommand, PhaseResult
from .motion_code_enums import MotionCode


class GoalExecutor(Node):
    def __init__(self):
        super().__init__('goal_executor')

        self.goal_queue = []
        self.executing = False
        self.motion_command = None
        self.awaiting_move = False

        self.cb_group = ReentrantCallbackGroup()

        self.goals_sub = self.create_subscription(
            Goals, "goals", self.goals_cb, 10, callback_group=self.cb_group
        )

        self.motion_sub = self.create_subscription(
            MotionCommand, "motion_comand", self.motion_cb, 10, callback_group=self.cb_group
        )

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group
        )

        self.result_pub = self.create_publisher(
            PhaseResult, "phase_result", 10
        )

        self.get_logger().info("goal_executor started and waiting for goals")

    async def goals_cb(self, msg: Goals):
        self.get_logger().info(f"Received {len(msg.goals)} goals")
        self.goal_queue.extend(msg.goals)
        
        if not self.executing:
            self.executing = True
            await self.send_next_goal()

    async def motion_cb(self, msg: MotionCommand):
        self.motion_command = msg.motion_code

        if self.motion_command == MotionCode.MOVE.value and self.awaiting_move:
            self.get_logger().info("MOVE command received while awaiting. Proceeding to next goal.")
            self.awaiting_move = False
            await self.send_next_goal()


    async def send_next_goal(self):
        while not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("compute_path_to_pose action server not available")
            return
        
        if not self.goal_queue:
            self.get_logger().info("All goals executed. Waiting for new goals...")
            self.executing = False

            # Send phase result
            result_msg = PhaseResult()
            result_msg.success = True
            result_msg.error_code = 0
            self.result_pub.publish(result_msg)
            return

        goal_pose = self.goal_queue.pop(0)

        # Send action goal and wait for result
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_handle = await self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(
            f"Sending goal: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}"
        )

        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            # await self.send_next_goal()
            return
        
        self.get_logger().info("Goal accepted by Nav2. Waiting for result...")

        # Receive result from Nav2
        result = await goal_handle.get_result_async()
        nav_result = result.result
        self.get_logger().info(f"Goal result received: {nav_result}")

        
        if self.motion_command == MotionCode.MOVE.value:
            self.get_logger().info("Received MOVE. Proceeding to goal_pose.")
            await self.send_next_goal()
        else:
            self.awaiting_move = True
            self.get_logger().info("Waiting for MOVE command...")


def main(args=None):
    rclpy.init(args=args)
    node = GoalExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("goal_executor stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()