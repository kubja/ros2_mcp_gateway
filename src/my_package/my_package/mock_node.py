#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from my_package.msg import Status
from my_package.srv import GetStatus
from my_package.action import Navigate


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
        msg.current_pose.pose.position.x = 1.0
        msg.current_pose.pose.position.y = 2.0
        msg.current_pose.pose.position.z = 0.0
        msg.battery_level = 95.5
        msg.is_charging = False
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Publishing Status: {{msg.current_pose.pose.position.x}}, {{msg.battery_level}}')

    def control_command_callback(self, msg):
        self.get_logger().info(f'Received Control Command: "{{msg.data}}"')
        # In a real application, process the command here

    def get_status_callback(self, request, response):
        self.get_logger().info(f'Received GetStatus request for robot_id: {{request.robot_id}}')
        response.status.header.stamp = self.get_clock().now().to_msg()
        response.status.header.frame_id = 'mock_frame'
        response.status.current_pose.pose.position.x = 10.0
        response.status.current_pose.pose.position.y = 20.0
        response.status.battery_level = 80.0
        response.status.is_charging = True
        self.get_logger().info('Sending GetStatus response.')
        return response

    async def execute_navigate_callback(self, goal_handle):
        self.get_logger().info(f'Executing Navigate goal: {{goal_handle.request.target_pose.pose.position.x}}, {{goal_handle.request.target_pose.pose.position.y}}')

        feedback_msg = Navigate.Feedback()
        # Simulate navigation progress
        for i in range(5):
            feedback_msg.current_pose.pose.position.x = goal_handle.request.target_pose.pose.position.x * (i + 1) / 5.0
            feedback_msg.current_pose.pose.position.y = goal_handle.request.target_pose.pose.position.y * (i + 1) / 5.0
            feedback_msg.distance_remaining = goal_handle.request.target_pose.pose.position.x * (5 - i) / 5.0 # Simplified
            self.get_logger().info(f'Publishing feedback: {{feedback_msg.current_pose.pose.position.x}}, {{feedback_msg.distance_remaining}}')
            goal_handle.publish_feedback(feedback_msg)
            await self.sleep_async(1.0) # Simulate work

        goal_handle.succeed()

        result = Navigate.Result()
        result.final_pose.pose.position.x = goal_handle.request.target_pose.pose.position.x
        result.final_pose.pose.position.y = goal_handle.request.target_pose.pose.position.y
        self.get_logger().info('Navigate goal succeeded.')
        return result

    async def sleep_async(self, seconds):
        # Helper to simulate async sleep in ROS2 context
        await rclpy.spin_until_future_complete(self, self.create_timer(seconds, lambda: None).timer_handle.future)

def main(args=None):
    rclpy.init(args=args)
    node = MockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()