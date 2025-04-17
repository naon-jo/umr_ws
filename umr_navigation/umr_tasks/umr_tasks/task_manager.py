import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.parameter import Parameter

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from umr_interfaces.msg import DispatchTask, DispatchResult, Phase, PhaseResult

class TaskManager(Node):
    """Task manager"""
    def __init__(self):
        super().__init__("task_manager")

        self.phase_queue = []
        self.current_phase = None
        self.waiting_for_result = False

        # QoS setup
        self.qos_profile = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self.task_sub = self.create_subscription(
            DispatchTask, "dispatch_task", self.dispatch_cb, self.qos_profile
        )
        
        self.result_pub = self.create_publisher(
            DispatchResult, "dispatch_result", 10
        )
        
        self.phase_pub = self.create_publisher(
            Phase, "phase", self.qos_profile
        )

        self.result_sub = self.create_subscription(
            PhaseResult, "phase_result", self.result_cb, 10
        )

    
    def dispatch_cb(self, msg: DispatchTask):
        self.get_logger().info(f"New task received: {msg.category}, from: {msg.requester}")
    
        self.send_dispatch_result(msg)

        if msg.category == "go_to_places":
            self.phase_queue = list(msg.phases)
            self.send_next_phase()
        else:
            self.get_logger().info(f"Unsupported task category: {msg.category}")
            return
        
    def send_dispatch_result(self, msg:DispatchTask):
        result_msg = DispatchResult()
        result_msg.request_id = msg.request_id
        result_msg.success = True
        result_msg.message = "Task received successfully"
        self.result_pub.publish(result_msg)
    
    def send_next_phase(self):
        if self.waiting_for_result:
            return
        
        if not self.phase_queue:
            self.get_logger().info(f"Task completed. All phases finished.")
            return

        self.current_phase = self.phase_queue.pop(0)

        self.phase_pub.publish(self.current_phase)
        self.waiting_for_result = True
    
    def result_cb(self, msg: PhaseResult):
        if msg.success:
            self.send_next_phase()

        else:
            self.get_logger().warn("failed with error {msg.error_code}: {msg.error_msg}")
            self.waiting_for_result = False


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("task_server stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()