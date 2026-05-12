import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class FrontierNavigator(Node):
    def __init__(self):
        super().__init__('frontier_navigator')
        # Map subscription - reduced queue size to avoid processing stale maps
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.map_data = None
        self.exploring = False  # The State Lock
        self.get_logger().info("Frontier Navigator Initialized")

    def map_callback(self, msg):
        # Store latest map data regardless
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

        # BLOCK: If we are already navigating, ignore the map update for goal seeking
        if self.exploring:
            return

        # Only start if we aren't already busy
        self.start_exploration()

    def start_exploration(self):
        # Double check lock before processing heavy CV logic
        if self.exploring:
            return

        goal = self.find_frontiers()
        if goal:
            self.get_logger().info(f"New frontier found at {goal[0]:.2f}, {goal[1]:.2f}. Locking state.")
            self.exploring = True # LOCK
            self.send_goal(goal[0], goal[1])
        else:
            # We don't log this every frame to avoid spamming the console
            pass

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        self._nav_client.wait_for_server()
        self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2. Unlocking state.")
            self.exploring = False # UNLOCK on rejection
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Goal is finished (Success, Canceled, or Aborted)
        status = future.result().status
        self.get_logger().info(f"Navigation finished with status: {status}. Ready for next frontier.")
        
        # UNLOCK: Now the map_callback can trigger start_exploration() again
        self.exploring = False 

    # [find_frontiers and pixel_to_world remain the same as your previous logic]
    def find_frontiers(self):
        if self.map_data is None:
            return None
        free_mask = np.where(self.map_data == 0, 255, 0).astype(np.uint8)
        unknown_mask = np.where(self.map_data == -1, 255, 0).astype(np.uint8)
        kernel = np.ones((3,3), np.uint8)
        dilated_free = cv2.dilate(free_mask, kernel, iterations=1)
        frontiers = cv2.bitwise_and(dilated_free, unknown_mask)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontiers)
        if num_labels > 1:
            largest_idx = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1
            px, py = centroids[largest_idx]
            return self.pixel_to_world(px, py)
        return None

    def pixel_to_world(self, px, py):
        wx = self.map_info.origin.position.x + px * self.map_info.resolution
        wy = self.map_info.origin.position.y + py * self.map_info.resolution
        return wx, wy

def main(args=None):
    rclpy.init(args=args)
    node = FrontierNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()