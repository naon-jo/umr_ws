import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.parameter import Parameter

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from umr_interfaces.msg import DispatchTask, Phase, DispatchResult

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

        # Subscriber
        self.task_sub = self.create_subscription(
            DispatchTask, "dispatch_task", self.dispatch_cb, self.qos_profile
        )
        # Publisher
        self.phase_pub = self.create_publisher(
            Phase, "phase", self.qos_profile
        )
        self.result_pub = self.create_publisher(
            DispatchResult, "dispatch_result", 10
        )
    
    def dispatch_cb(self, msg: DispatchTask):
        self.get_logger().info(f"New task received: {msg.category}, from: {msg.requester}")
    
        self.send_dispatch_result(msg)

        if msg.category == "go_to_places":
            self.phase_queue = list(msg.phases)
            self.send_phase()
        else:
            self.get_logger().info(f"Unsupported task category: {msg.category}")
            return
        
    def send_dispatch_result(self, msg:DispatchTask):
        result_msg = DispatchResult()
        result_msg.request_id = msg.request_id
        result_msg.success = True
        result_msg.message = "Task received successfully"
        self.result_pub.publish(result_msg)
    
    def send_phase(self):
        if self.waiting_for_result:
            return
        
        if not self.phase_queue:
            self.get_logger().info(f"Task completed. All phases finished.")
            #TODO: 피드백 구조
            return

        self.current_phase = self.phase_queue.pop(0)

        self.phase_pub.publish(self.current_phase)
        self.waiting_for_result = True
    
    def result_cb(self, msg:String):
        #TODO: error code & error_msg 업데이트
        result = msg.data
        
        if result == "success":
            self.get_logger().info(f"Phase finished: {result}")
            self.waiting_for_result = False
            self.send_phase()
        elif result == "failed":
            self.get_logger().warn(f"Phase finished: {result}")
            self.waiting_for_result = False
            #TODO: retry logic or skip
        else:
            self.get_logger().info(f"Unknown result code: {result}")
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