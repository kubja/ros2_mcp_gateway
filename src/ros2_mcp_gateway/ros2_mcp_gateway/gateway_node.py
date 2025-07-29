import rclpy
from rclpy.node import Node
import asyncio
import threading
import json
from typing import Dict, Any
from fastmcp import FastMCP
import sys
import os

print(f"Python Executable: {sys.executable}")
print(f"sys.path: {sys.path}")
print(f"PYTHONPATH: {os.environ.get('PYTHONPATH')}")

mcp = FastMCP("ROS2 MCP Gateway")

class MCPGateway(Node):
    def __init__(self, name: str, service_configs: Dict[str, Dict[str, Any]], action_configs: Dict[str, Dict[str, Any]], topic_configs: Dict[str, Dict[str, Any]]):
        super().__init__(name)
        self.service_configs = service_configs
        self.action_configs = action_configs
        self.topic_configs = topic_configs
        self.mcp_services = {}
        self.mcp_actions = {}
        self.mcp_latest_msgs = {}
        self._create_clients()
        self._subscribe_topics()

    def _create_clients(self):
        from rosidl_runtime_py.utilities import get_service
        from rosidl_runtime_py.utilities import get_action

        for name, meta in self.service_configs.items():
            srv_type = get_service(meta['type'])
            client = self.create_client(srv_type, name)
            self.mcp_services[name] = client

        for name, meta in self.action_configs.items():
            action_type = get_action(meta['type'])
            from rclpy.action import ActionClient
            client = ActionClient(self, action_type, name)
            self.mcp_actions[name] = client

    def _subscribe_topics(self):
        from rosidl_runtime_py.utilities import get_message
        for topic_name, meta in self.topic_configs.items():
            msg_type = get_message(meta['type'])

            def callback(msg, topic_name=topic_name):
                self.mcp_latest_msgs[topic_name] = msg

            self.create_subscription(msg_type, topic_name, callback, 10)

    async def call_service(self, name: str, args: Dict[str, Any]) -> Dict:
        client = self.mcp_services[name]
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {name} not available, waiting...')

        req_type = type(client.srv_type)()
        for key, val in args.items():
            setattr(req_type, key, val)

        future = client.call_async(req_type)
        await asyncio.wrap_future(future)
        return future.result().__dict__

    async def run_action(self, name: str, args: Dict[str, Any], stream_queue: asyncio.Queue):
        from rclpy.action import GoalResponse, CancelResponse
        client = self.mcp_actions[name]
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Action {name} not available, waiting...')

        goal_msg = client.action_type.Goal()
        for key, val in args.items():
            setattr(goal_msg, key, val)

        future = client.send_goal_async(goal_msg, feedback_callback=lambda fb: stream_queue.put_nowait({"type": "progress", "feedback": fb.feedback.__dict__}))
        goal_handle = await future

        if not goal_handle.accepted:
            await stream_queue.put({"type": "error", "message": "Goal rejected"})
            return

        result_future = goal_handle.get_result_async()
        result = await result_future
        await stream_queue.put({"type": "result", "result": result.result.__dict__})

    async def get_latest_topic(self, name: str) -> Dict[str, Any]:
        msg = self.mcp_latest_msgs.get(name)
        if msg is None:
            return {"error": "No message received yet."}
        return msg.__dict__


# Global placeholder for ROS node
ros_node: MCPGateway = None

def main():
    import sys
    rclpy.init()

    global ros_node
    service_defs = {
        "get_status": {
            "type": "my_package/srv/GetStatus",
            "description": "Retrieve current robot system status."
        }
    }
    action_defs = {
        "navigate_to": {
            "type": "my_package/action/Navigate",
            "description": "Navigate the robot to a given target pose."
        }
    }
    topic_defs = {
        "status": {
            "type": "my_package/msg/Status",
            "description": "Get the latest system status message."
        }
    }
    ros_node = MCPGateway("ros2_mcp_gateway", service_defs, action_defs, topic_defs)

    def create_service_tool(name, meta):
        @mcp.tool(name=name, description=meta['description'])
        async def service_tool(args: Dict[str, Any]) -> Dict:
            return await ros_node.call_service(name, args)
        return service_tool

    for name, meta in service_defs.items():
        create_service_tool(name, meta)

    def create_action_tool(name, meta):
        @mcp.tool(name=name, description=meta['description'], exclude_args=['stream_queue'])
        async def action_tool(args: Dict[str, Any], stream_queue=None):
            await ros_node.run_action(name, args, stream_queue)
        return action_tool

    for name, meta in action_defs.items():
        create_action_tool(name, meta)

    def create_topic_tool(name, meta):
        @mcp.tool(name=name, description=meta['description'])
        async def topic_tool() -> Dict:
            return await ros_node.get_latest_topic(name)
        return topic_tool

    for name, meta in topic_defs.items():
        create_topic_tool(name, meta)

    loop = asyncio.get_event_loop()
    spin_thread = threading.Thread(target=ros_spin, args=(ros_node,))
    spin_thread.daemon = True
    spin_thread.start()
    mcp.run(transport="http")

def ros_spin(node):
    rclpy.spin(node)


if __name__ == '__main__':
    main()