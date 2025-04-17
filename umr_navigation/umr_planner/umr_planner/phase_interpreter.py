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

"""

ros2 run nav2_planner planner_server --ros-args --params-file nav2_params.yaml

"""

class PhaseInterpreter(Node):
    def __init__(self):
        super().__init__("phase_interpreter")

        pkg_share_dir = get_package_share_directory("umr_planner")
        yaml_path = os.path.join(pkg_share_dir, "config", "waypoints.yaml")
        self.place_map = load_place_map(yaml_path)

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
        self.get_logger().info(f"Received Phase: category={msg.category}, place={msg.place}")
        
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
        stop_pose = await self.find_valid_stop_point(goal_pose)

        goals_msg = Goals()
        goals_msg.goals = [stop_pose, goal_pose]
        self.goals_pub.publish(goals_msg)
        self.get_logger().info(
            f"{msg.category}:\n"
            f" - stop_point: ({stop_pose.pose.position.x:.2f}, {stop_pose.pose.position.y:.2f})\n"
            f" - goal_point: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})\n"
        )

    
    async def find_valid_stop_point(self, goal_pose: PoseStamped, radius=0.5, steps=12):
        while not self.path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("compute_path_to_pose action server not available")
            return
        
        for i in range(steps):
            self.get_logger().info(f"step {i+1}")
            theta = 2 * math.pi * i / steps
            dx = radius * math.cos(theta)
            dy = radius * math.sin(theta)

            candidate = PoseStamped()
            candidate.header.frame_id = "map"
            candidate.header.stamp = self.get_clock().now().to_msg()
            candidate.pose.position.x = goal_pose.pose.position.x + dx
            candidate.pose.position.y = goal_pose.pose.position.y + dy
            candidate.pose.position.z = goal_pose.pose.position.z
            candidate.pose.orientation = goal_pose.pose.orientation

            # Send action goal and wait for result
            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal = candidate
            goal_handle = await self.path_client.send_goal_async(goal_msg)

            if not goal_handle.accepted:
                self.get_logger().error("/compute_path_to_pose action goal rejected.")
                goal_handle.abort()
                continue
            
            self.get_logger().info("/compute_path_to_pose foal was accepted. Waiting for result...")

            # Receive result from Nav2 planner_server
            result = await goal_handle.get_result_async()
            path_result = result.result
            self.get_logger().info(f"Got path with {len(path_result.path.poses)} poses.")
        
            # Check path.poses is not empty.
            if len(path_result.path.poses) > 0:
                self.get_logger().info(f"Valid stop point found at ({candidate.pose.position.x:.2f}, {candidate.pose.position.y:.2f})")
                return candidate
        
        self.get_logger().warn("No valid stop point found. Using goal_pose directly.")
        return goal_pose


def load_place_map(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    place_map = {}
    for name, pose_dict in data.items():
        pose = Pose()
        pose.position.x = pose_dict["position"]["x"]
        pose.position.y = pose_dict["position"]["y"]
        pose.position.z = pose_dict["position"]["z"]
        pose.orientation.x = pose_dict["orientation"]["x"]
        pose.orientation.y = pose_dict["orientation"]["y"]
        pose.orientation.z = pose_dict["orientation"]["z"]
        pose.orientation.w = pose_dict["orientation"]["w"]
        place_map[name] = pose
    
    return place_map


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