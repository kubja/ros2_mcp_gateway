#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math
import asyncio

from my_package.msg import Status
from my_package.srv import GetStatus
from my_package.action import Navigate
from geometry_msgs.msg import Point


class MockNode(Node):

    def __init__(self):
        super().__init__('mock_node')
        self.get_logger().info('Mock Node has been started.')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.current_position = Point(x=0.0, y=0.0, z=0.0)

        # 1. Mock Publisher (Status messages)
        self.status_publisher = self.create_publisher(Status, 'my_package/status', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Mock Status Publisher created.')

        # 2. Mock Subscriber (Control Command - using a simple String for now)
        # For demonstration, let's assume a simple string message for commands.
        # In a real scenario, you'd define a custom message type for commands.
        from std_msgs.msg import String
        self.control_subscriber = self.create_subscription(
            String, 'my_package/control_command', self.control_command_callback, qos_profile)
        self.get_logger().info('Mock Control Command Subscriber created.')

        # 3. Mock Service Server (GetStatus)
        self.get_status_service = self.create_service(GetStatus, 'my_package/get_status', self.get_status_callback)
        self.get_logger().info('Mock GetStatus Service Server created.')

        # 4. Mock Action Server (Navigate)
        self.navigate_action_server = ActionServer(
            self,
            Navigate,
            'my_package/navigate',
            self.execute_navigate_callback)
        self.get_logger().info('Mock Navigate Action Server created.')

    def publish_status(self):
        msg = Status()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mock_frame'
        msg.current_pose.pose.position = self.current_position
        msg.battery_level = 95.5
        msg.is_charging = False
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Publishing Status: {msg.current_pose.pose.position.x}, {msg.current_pose.pose.position.y}, {msg.battery_level}')

    def control_command_callback(self, msg):
        self.get_logger().info(f'Received Control Command: "{msg.data}"')
        # In a real application, process the command here

    def get_status_callback(self, request, response):
        self.get_logger().info('Received GetStatus request.')
        response.status.header.stamp = self.get_clock().now().to_msg()
        response.status.header.frame_id = 'mock_frame'
        response.status.current_pose.pose.position = self.current_position
        response.status.battery_level = 80.0
        response.status.is_charging = True
        self.get_logger().info('Sending GetStatus response.')
        return response

    async def execute_navigate_callback(self, goal_handle):
        goal_position = goal_handle.request.goal_position
        self.get_logger().info(f'Executing Navigate goal to: {goal_position.x}, {goal_position.y}, {goal_position.z}')

        feedback_msg = Navigate.Feedback()
        
        start_time = self.get_clock().now().nanoseconds / 1e9
        duration = 10.0  # seconds
        
        initial_position = self.current_position
        
        while True:
            elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - start_time
            if elapsed_time >= duration:
                self.current_position = goal_position
                break

            # Linear interpolation for smooth movement
            alpha = elapsed_time / duration
            self.current_position.x = initial_position.x + alpha * (goal_position.x - initial_position.x)
            self.current_position.y = initial_position.y + alpha * (goal_position.y - initial_position.y)
            self.current_position.z = initial_position.z + alpha * (goal_position.z - initial_position.z)

            distance_to_goal = math.sqrt(
                (goal_position.x - self.current_position.x)**2 +
                (goal_position.y - self.current_position.y)**2 +
                (goal_position.z - self.current_position.z)**2
            )

            feedback_msg.current_position = self.current_position
            feedback_msg.distance_to_goal = float(distance_to_goal)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Publishing feedback: current_pos=({self.current_position.x:.2f}, {self.current_position.y:.2f}), dist_to_goal={distance_to_goal:.2f}')
            
            # Publish status with updated position
            self.publish_status()
            
            await self.sleep_async(1.0) # Simulate work in 1-second intervals

        goal_handle.succeed()

        result = Navigate.Result()
        result.success = True
        self.get_logger().info('Navigate goal succeeded.')
        return result

    async def sleep_async(self, seconds):
        # Helper to simulate async sleep
        await asyncio.sleep(seconds)

def main(args=None):
    rclpy.init(args=args)
    node = MockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()