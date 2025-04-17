import sys
import asyncio
import argparse
import uuid
import pprint
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.parameter import Parameter

from umr_interfaces.msg import DispatchTask, Phase, DispatchResult

"""

"""
class TaskDispatcher(Node):
    """Task dispatcher."""
    def __init__(self, argv=sys.argv):
        """Initialize a task dispatcher."""
        super().__init__("task_dispatcher")

        # Parse CLI arguments
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "-p",
            "--place",
            type=str,
            nargs="+",
            required=True,
            help="Place to go to",
        )
        parser.add_argument(
            "-r",
            "--requester",
            type=str,
            default="cli_user",
            help="Name of requester"
        )
        parser.add_argument(
            "--use_sim_time",
            action="store_true",
            help="Use sim time, default: false",
        )
        self.args = parser.parse_args(argv[1:])
        self.request_id = str(uuid.uuid4())

        # Future object to wait for result
        self.response = asyncio.Future()

        # QoS
        self.qos_profile = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        # Publisher
        self.pub = self.create_publisher(
            DispatchTask, "dispatch_task", self.qos_profile
        )

        # Subscription
        self.sub = self.create_subscription(
            DispatchResult, "dispatch_result", self.response_cb, 10
        )

        # Enable ROS2 sim time
        if self.args.use_sim_time:
            self.get_logger().info("Using Sim Time")
            param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
            self.set_parameters([param])
        
        # Create DispatchTask
        task_msg = DispatchTask()
        task_msg.request_id = self.request_id
        task_msg.category = "go_to_places"
        task_msg.requester = self.args.requester

        for place in self.args.place:
            phase = Phase()
            phase.category = "go_to_place"
            phase.place = place
            task_msg.phases.append(phase)
        
        self.pub.publish(task_msg)
        self.get_logger().info("Dispatch task sent:\n" + pprint.pformat([{
            "category": p.category,
            "place": p.place
        } for p in task_msg.phases]))

    def response_cb(self, msg: DispatchResult):
        if msg.request_id != self.request_id:
            return
        self.get_logger().info(f"Received response:\n - success: {msg.success}\n - message: {msg.message}")
        self.response.set_result(msg)

def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    task_dispatcher = TaskDispatcher(args_without_ros)
    rclpy.spin_until_future_complete(
        task_dispatcher, task_dispatcher.response, timeout_sec=5.0
    )
    if task_dispatcher.response.done():
        task_dispatcher.get_logger().info(f"Response: \n{task_dispatcher.response.result().success}")
    else:
        task_dispatcher.get_logger().info(f"Did not get a response")

    task_dispatcher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
     main(sys.argv)