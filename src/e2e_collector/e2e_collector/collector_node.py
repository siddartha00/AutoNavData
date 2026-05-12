import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import json
import os
import time

class E2EDataCollector(Node):
    def __init__(self):
        super().__init__('e2e_data_collector')
        self.bridge = CvBridge()
        self.data_dir = '/ros2_ws/src/collected_data'
        self.active_task = False

        # Storage for current state
        self.current_img = None
        self.current_scan = None
        self.current_odom = None
        self.current_goal = None
        
        # Subscriptions
        self.create_subscription(Image, '/intel_realsense_r200/image_raw', self.img_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10) # The "Label"

    def img_callback(self, msg): self.current_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def scan_callback(self, msg): self.current_scan = msg.ranges
    def odom_callback(self, msg): self.current_odom = msg.pose.pose
    def goal_callback(self, msg): self.current_goal = msg.pose

    def on_configure(self, state):
        self.get_logger().info("Configuring: Checking for data directories and publishers...")
        os.makedirs(self.data_dir, exist_ok=True)
        # Initialize subscribers but don't process data yet
        self.img_sub = self.create_subscription(Image, '/intel_realsense_r200/image_raw', self.img_cb, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("Activating: Data collection started.")
        # Only in Active state will we actually write to disk
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info("Deactivating: Data collection paused.")
        return super().on_deactivate(state)

    def goal_cb(self, msg):
        # Logic to detect if a new goal is set
        self.active_task = True

    def cmd_callback(self, msg):
        # We trigger a save every time the "Expert" sends a command
        if self.current_img is not None:
            timestamp = time.time_ns()
            # Save Image
            img_path = f"{self.data_dir}/img_{timestamp}.jpg"
            cv2.imwrite(img_path, self.current_img)
            
            # Save Metadata (The JSON "Label")
            data = {
                "timestamp": timestamp,
                "img_path": img_path,
                "cmd_vel": {"linear": msg.linear.x, "angular": msg.angular.z},
                "odom": {"x": self.current_odom.position.x, "y": self.current_odom.position.y},
                "goal": {"x": self.current_goal.position.x, "y": self.current_goal.position.y} if self.current_goal else None,
                "lidar": list(self.current_scan) if self.current_scan else None
            }
            
            with open(f"{self.data_dir}/data_{timestamp}.json", 'w') as f:
                json.dump(data, f)

def main():
    rclpy.init()
    node = E2EDataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()