import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool, String
from umr_interfaces.msg import DetectionArray, Detection, MotionCommand

import math

"""
Description:
Receive obstacle info and request waypoints.

---
Topic Subscription:
    msg type: std_msgs/Float32
    topic name: /lidar_info

Topic Subscription:
    msg type: std_msgs/Bool
    topic name: /human_info

Topic pub:
    msg type: umr_interfaces/MotionCommand
    topic name: /motion_command

"""

class MotionPlanner(Node):
    def __init__(self):
        super().__init__("motion_planner")

        # Initialize variables
        self.lidar_info = None
        self.human_info = None
        self.box_info = None
        self.threshold = 1.0
        self.timer_period = 0.1

        # Subscription
        self.lidar_subscription = self.create_subscription(
            Float32, "lidar_info", self.lidar_callback, 10)
        self.human_subscription = self.create_subscription(
            Bool, "human_info", self.human_callback, 10)
        self.box_subscription = self.create_subscription(
            Bool, "box_info", self.box_callback, 10)
        
        # Publisher
        self.pub = self.create_pub(
            self, MotionCommand, "motion_command")

        # Timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def lidar_callback(self, msg: Float32):
        self.lidar_info = msg

    def human_callback(self, msg: Bool):
        self.human_info = msg

    def box_callback(self, msg: Bool):
        self.box_info = msg
    
    def timer_callback(self):
        cmd_msg = MotionCommand()

        if self.lidar_info is not None and self.lidar_info.data < self.threshold:
            self.get_logger().info(f"Front obstacle detected: {self.lidar_info.data} m")

            if self.human_info is not None and self.human_info.data is True:
                self.get_logger().info(f"Human detected: {self.human_info.data}")

                cmd_msg.motion_code = 0
                cmd_msg.motion_msg = "human detected"
                cmd_msg.human_detected = True
            
            elif self.box_info is not None and self.box_info is True:
                self.get_logger().info(f"Box detected: {self.box_info.data}")

                cmd_msg.motion_code = 0
                cmd_msg.motion_msg = "box blocking"
                cmd_msg.human_detected = False

            else:
                cmd_msg.motion_code = 0
                cmd_msg.motion_msg = "Obstacle detected"
                cmd_msg.human_detected = False
        else:
            self.get_logger().info("Move")
            cmd_msg.motion_code = 1
            cmd_msg.motion_msg = "path_clear"
            cmd_msg.motion_command = False
        
        self.get_logger().info(f"Publish motion command: {cmd_msg}")
        self.pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("motion_planner stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
