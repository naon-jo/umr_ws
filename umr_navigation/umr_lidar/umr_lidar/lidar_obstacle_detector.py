import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from umr_interfaces.msg import LidarInfo
from rclpy.qos import *

from .lib import lidar_perception_func_lib as LPFL

""" -------------------------------------------------
Description
라이다 센싱값으로 전방 장애물을 감지하는 노드
---
Subscription:
    type: "LaserScan"
    topic_name: "/scan"

Publisher:
    type: "LidarInfo"
    topic_name: "/lidar_info"
--------------------------------------------------"""

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__("lidar_detector_node")

        # params
        self.declare_parameter("front_angle_range_deg", 60)
        self.declare_parameter("distance_threshold", 2.0)
        
        # publisher & subscription
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(LidarInfo, "/lidar_info", 10)   #TODO: qos 수정하기

    def lidar_callback(self, msg):
        front_angle_range_deg = self.get_parameter("front_angle_range_deg").get_parameter_value().double_value
        distance_threshold = self.get_parameter("distance_threshold").get_parameter_value().double_value
        
        distance_min = LPFL.check_obstacle_front(
            msg = msg, 
            front_angle_range_deg = front_angle_range_deg)
        detected = distance_min < distance_threshold
        self.get_logger().info(f"detected: {detected}, dist: {distance_min:.2f} m")

        lidar_msg = LidarInfo()
        lidar_msg.detected = detected
        lidar_msg.distance_min = distance_min
        self.publisher.publish(lidar_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("lidar_obstacle_detector stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()