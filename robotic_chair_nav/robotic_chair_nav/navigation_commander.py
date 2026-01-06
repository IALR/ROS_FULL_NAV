#!/usr/bin/env python3
import threading
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


class NavigationCommander(Node):
    def __init__(self):
        super().__init__('navigation_commander')

        # Map-frame destinations (from RViz /clicked_point)
        self.rooms = {
            'a': {'name': 'RED Room',    'x': 12.3871517,  'y': 23.0562725},
            'b': {'name': 'GREEN Room',  'x': 23.6751976,  'y': 22.8605423},
            'c': {'name': 'BLUE Room',   'x': 12.6005955,  'y': 11.3363991},
            'd': {'name': 'YELLOW Room', 'x': 23.6751575,  'y': 11.7718639},
            'e': {'name': 'Couloir',     'x': 18.3430271,  'y': 17.9256134},
        }

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        # Start non-blocking user input thread
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info(
            'Navigation Commander ready.\n'
            'Type a/b/c/d/e to send goal, or q to quit.'
        )

    def initialize_amcl_robust(self, x=0.0, y=0.0, yaw=0.0, attempts=5, delay=0.5):
        """Seed AMCL by publishing /initialpose multiple times."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        for _ in range(attempts):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            # Loose covariance to let AMCL converge
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = (math.radians(15)) ** 2
            self.initial_pose_pub.publish(msg)
            time.sleep(delay)
        self.get_logger().info(f'AMCL seeded at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad')

    def user_input_loop(self):
        prompt = (
            "\n====================================\n"
            " Select destination:\n"
            "   a) RED Room\n"
            "   b) GREEN Room\n"
            "   c) BLUE Room\n"
            "   d) YELLOW Room\n"
            "   e) COULOIR\n"
            "   q) Quit\n"
            "====================================\n"
            "Enter choice: "
        )
        while rclpy.ok():
            try:
                sys.stdout.write(prompt)
                sys.stdout.flush()
                choice = sys.stdin.readline().strip().lower()
            except Exception:
                break

            if choice == 'q':
                self.get_logger().info('Exiting commander; shutting down rclpy.')
                rclpy.shutdown()
                break

            if choice in self.rooms:
                self.send_goal(choice)
            else:
                self.get_logger().info('Invalid choice. Use a/b/c/d/e or q.')

    def send_goal(self, key):
        room = self.rooms[key]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = room['x']
        goal_msg.pose.pose.position.y = room['y']
        goal_msg.pose.pose.orientation.w = 1.0  # facing forward

        self.get_logger().info(
            f"Sending goal to {room['name']} at (x={room['x']:.2f}, y={room['y']:.2f})"
        )

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return

        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            return
        self.get_logger().info('Goal accepted; navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        # Uncomment for live distance updates:
        # self.get_logger().info(f"Distance remaining: {fb.distance_remaining:.2f} m")

    def result_callback(self, future):
        status = future.result().status
        if status == 0:
            self.get_logger().info('Reached goal successfully.')
        else:
            self.get_logger().warn(f'Goal failed with status code: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationCommander()

    # Optional: seed AMCL here if you know the spawn pose
    # node.initialize_amcl_robust(x=0.0, y=0.0, yaw=0.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()