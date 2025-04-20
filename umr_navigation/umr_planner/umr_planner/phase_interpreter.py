import os
import yaml
import math
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped
from umr_interfaces.msg import Phase, Goals
from nav2_msgs.action import ComputePathToPose

from .lib import phase_interpreter_func_lib as PIFL

"""

"""

class PhaseInterpreter(Node):
    def __init__(self):
        super().__init__("phase_interpreter")

        pkg_share_dir = get_package_share_directory("umr_planner")
        yaml_path = os.path.join(pkg_share_dir, "config", "waypoints.yaml")
        self.place_map = PIFL.load_place_map(yaml_path)

        self.cb_group = ReentrantCallbackGroup()

        self.phase_sub = self.create_subscription(
            Phase, "phase", self.phase_cb, 10, callback_group=self.cb_group
        )
        self.path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=self.cb_group
        )
        self.goals_pub = self.create_publisher(
            Goals, "goals", 10
        )
        
        self.get_logger().info("phase_interpreter started and waiting for Phase messages.")
    

    async def phase_cb(self, msg: Phase):
        self.get_logger().info(f"Received Phase:\n - category: {msg.category}\n - place: {msg.place}")
        
        if msg.place not in self.place_map:
            self.get_logger().error(f"Unknown place: {msg.place}")
            return
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = self.place_map[msg.place]
        self.get_logger().info(
            f"{msg.place}:\n - x: {goal_pose.pose.position.x:.2f}\n - y :{goal_pose.pose.position.y:.2f}"
        )
        
        self.get_logger().info("Searching for valid stop point around goal...")
        stop_pose = await self.find_valid_stop_point(goal_pose, stop_distance=1.0)

        goals_msg = Goals()
        goals_msg.goals = [stop_pose, goal_pose]
        self.goals_pub.publish(goals_msg)
        self.get_logger().info(
            f"{msg.category}:\n"
            f" - stop_pose: ({stop_pose.pose.position.x:.2f}, {stop_pose.pose.position.y:.2f})\n"
            f" - goal_pose: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})\n"
            f" - stop_pose_msg:\n {stop_pose}\n"
            f" - goal_pose_msg:\n {goal_pose}\n"

        )
    
    
    async def find_valid_stop_point(self, goal_pose: PoseStamped, stop_distance: float = 0.5) -> PoseStamped:
        while not self.path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Waiting for compute_path_to_pose action server...")
            return goal_pose
        
        # Send action goal and wait for result
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_handle = await self.path_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.get_logger().error("/compute_path_to_pose action goal rejected.")
            goal_handle.abort()
            return goal_pose
            
        self.get_logger().info("/compute_path_to_pose goal was accepted. Waiting for result...")
        result = await goal_handle.get_result_async()
        path_result = result.result
        self.get_logger().info(f"Got path with {len(path_result.path.poses)} poses.")
        
        if len(path_result.path.poses) == 0:
            self.get_logger().warn("Received empty path. Using goal_pose directly.")
            return goal_pose
        
        stop_pose = PoseStamped()
        stop_pose.header.frame_id = "map"
        stop_pose.header.stamp = self.get_clock().now().to_msg()
        stop_pose.pose = PIFL.get_stop_point(path_result.path, stop_distance)
        self.get_logger().info(
            f"Valid stop point:\n"
            f" x: {stop_pose.pose.position.x:.2f}\n"
            f" y: {stop_pose.pose.position.y:.2f}\n")
        
        return stop_pose



def main(args=None):
    rclpy.init(args=args)
    node = PhaseInterpreter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("phase_interpreter stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()