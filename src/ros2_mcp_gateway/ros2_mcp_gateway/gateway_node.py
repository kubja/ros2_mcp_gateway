import rclpy
from rclpy.node import Node
import asyncio
import threading
import json
from typing import Dict, Any, List
from fastmcp import FastMCP
import os
import yaml
from geometry_msgs.msg import Point
from rosidl_runtime_py.utilities import get_message, get_service, get_action
from pydantic import create_model, BaseModel, Field
import inspect
from types import FunctionType

# ---------------- ROS Utilities ----------------

def ros_message_to_dict(msg):
    """Convert a ROS message to a dictionary."""
    if not hasattr(msg, 'get_fields_and_field_types'):
        return msg
    d = {}
    for field, field_type in msg.get_fields_and_field_types().items():
        value = getattr(msg, field)
        if hasattr(value, 'get_fields_and_field_types'):
            d[field] = ros_message_to_dict(value)
        elif isinstance(value, list):
            d[field] = [ros_message_to_dict(v) if hasattr(v, 'get_fields_and_field_types') else v for v in value]
        else:
            d[field] = value
    return d

def rclpy_future_to_concurrent_future(rclpy_future, loop):
    """Convert an rclpy Future to asyncio Future."""
    concurrent_future = asyncio.Future()
    def on_done(rclpy_future):
        try:
            result = rclpy_future.result()
            loop.call_soon_threadsafe(concurrent_future.set_result, result)
        except Exception as e:
            loop.call_soon_threadsafe(concurrent_future.set_exception, e)
    rclpy_future.add_done_callback(on_done)
    return concurrent_future

def convert_dict_to_ros_message(data: Dict, ros_message_type):
    """Convert dictionary/Pydantic model to ROS message recursively."""
    ros_msg = ros_message_type()
    for field_name, field_type_string in ros_message_type.get_fields_and_field_types().items():
        if field_name not in data:
            continue
        value = data[field_name]
        if isinstance(value, BaseModel):
            value = value.model_dump()
        is_array = field_type_string.endswith('[]')
        if is_array:
            field_type_string = field_type_string[:-2]
            nested_type = get_message(field_type_string)
            setattr(ros_msg, field_name, [
                convert_dict_to_ros_message(item, nested_type) if hasattr(nested_type, 'get_fields_and_field_types') else item
                for item in value
            ])
        elif hasattr(get_message(field_type_string), 'get_fields_and_field_types'):
            nested_type = get_message(field_type_string)
            setattr(ros_msg, field_name, convert_dict_to_ros_message(value, nested_type))
        else:
            setattr(ros_msg, field_name, value)
    return ros_msg

def ros_message_type_to_pydantic_model(ros_message_type):
    """Create Pydantic model from ROS message type."""
    fields = {}
    for field_name, field_type_string in ros_message_type.get_fields_and_field_types().items():
        is_array = field_type_string.endswith('[]')
        if is_array:
            field_type_string = field_type_string[:-2]
        py_type = Any
        if field_type_string == 'bool':
            py_type = bool
        elif field_type_string == 'string':
            py_type = str
        elif field_type_string in ['float32', 'float64']:
            py_type = float
        elif field_type_string in ['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']:
            py_type = int
        elif field_type_string in ['time', 'duration']:
            py_type = Dict[str, int]
        else:
            try:
                nested_type = get_message(field_type_string)
                py_type = ros_message_type_to_pydantic_model(nested_type)
            except Exception:
                pass
        if is_array:
            py_type = List[py_type]
        fields[field_name] = (py_type, Field(...))
    model_name = ros_message_type.__name__ if hasattr(ros_message_type, '__name__') else 'DynamicModel'
    return create_model(model_name, **fields)

# ---------------- MCP Gateway Node ----------------

mcp = FastMCP("ROS2 MCP Gateway")

class MCPGateway(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        if not config_file or not os.path.exists(config_file):
            self.get_logger().error(f"Config file not found: {config_file}")
            config_data = {}
        else:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
        if 'mcp_gateway' in config_data:
            config_data = config_data['mcp_gateway']['ros__parameters']
        self.service_configs = config_data.get('service_configs', {})
        self.action_configs = config_data.get('action_configs', {})
        self.topic_configs = config_data.get('topic_configs', {})
        self.mcp_services = {}
        self.mcp_actions = {}
        self.mcp_latest_msgs = {}
        self.shutdown_event = threading.Event()
        self._create_clients()
        self._subscribe_topics()

    def _create_clients(self):
        for name, meta in self.service_configs.items():
            srv_type = get_service(meta['type'])
            client = self.create_client(srv_type, meta['name'])
            self.mcp_services[name] = client
        for name, meta in self.action_configs.items():
            action_type = get_action(meta['type'])
            from rclpy.action import ActionClient
            client = ActionClient(self, action_type, meta['name'])
            self.mcp_actions[name] = (client, action_type)

    def _subscribe_topics(self):
        for topic_name, meta in self.topic_configs.items():
            msg_type = get_message(meta['type'])
            def callback(msg, topic_name=topic_name):
                self.mcp_latest_msgs[topic_name] = msg
            self.create_subscription(msg_type, meta['name'], callback, 10)

    async def call_service(self, name: str, args: Dict[str, Any]) -> Dict:
        client = self.mcp_services[name]
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {name} not available, waiting...')
        req = client.srv_type.Request()
        for key, val in args.items():
            setattr(req, key, val)
        try:
            rclpy_future = client.call_async(req)
            concurrent_future = rclpy_future_to_concurrent_future(rclpy_future, asyncio.get_running_loop())
            await asyncio.wait_for(concurrent_future, timeout=3.0)
            return ros_message_to_dict(concurrent_future.result())
        except asyncio.TimeoutError:
            return {"error": f"Service call to {name} timed out."}

    async def run_action(self, name: str, args: Dict[str, Any], stream_queue: asyncio.Queue):
        from rclpy.action import GoalResponse, CancelResponse
        client, action_type = self.mcp_actions[name]
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Action {name} not available, waiting...')
        goal_msg = action_type.Goal()
        for key, val in args.items():
            setattr(goal_msg, key, val)
        future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: stream_queue.put_nowait({"type":"progress","feedback":ros_message_to_dict(fb.feedback)})
        )
        goal_handle = await future
        if not goal_handle.accepted:
            await stream_queue.put({"type": "error", "message": "Goal rejected"})
            return
        result_future = goal_handle.get_result_async()
        result = await result_future
        await stream_queue.put({"type":"result","result":ros_message_to_dict(result.result)})

    async def get_latest_topic(self, name: str) -> Dict[str, Any]:
        msg = self.mcp_latest_msgs.get(name)
        if msg is None:
            return {"error": "No message received yet."}
        return ros_message_to_dict(msg)

# ---------------- Tool Generation ----------------

ros_node: MCPGateway = None

def ros_spin(node: MCPGateway):
    while rclpy.ok() and not node.shutdown_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)

def create_service_tool_function(name: str, ros_node: MCPGateway, fields: Dict[str, Any], description: str):
    args_str = ", ".join(fields.keys())
    func_code = f"""
async def {name}_tool({args_str}):
    args = {{{', '.join([f"'{k}': {k}" for k in fields.keys()])}}}
    return await ros_node.call_service("{name}", args)
"""
    namespace = {"ros_node": ros_node}
    exec(func_code, namespace)
    func = namespace[f"{name}_tool"]
    func.__doc__ = description
    return func

def create_action_tool_function(name: str, ros_node: MCPGateway, action_type, description: str):
    goal_fields = action_type.Goal.get_fields_and_field_types()
    args_str = ", ".join(goal_fields.keys())
    func_code = f"""
async def {name}_tool({args_str}):
    args = {{{', '.join([f"'{k}': {k}" for k in goal_fields.keys()])}}}
    await ros_node.run_action("{name}", args, None)
"""
    namespace = {"ros_node": ros_node}
    exec(func_code, namespace)
    func = namespace[f"{name}_tool"]
    func.__doc__ = description
    return func

def create_topic_tool_function(name: str, ros_node: MCPGateway, description: str):
    async def topic_tool():
        return await ros_node.get_latest_topic(name)
    topic_tool.__doc__ = description
    return topic_tool

# ---------------- Main ----------------

def main():
    global ros_node
    rclpy.init()
    ros_node = MCPGateway("ros2_mcp_gateway")

    # Service tools
    for name, meta in ros_node.service_configs.items():
        srv_type = get_service(meta['type'])
        fields = srv_type.Request.get_fields_and_field_types()
        func = create_service_tool_function(name, ros_node, fields, meta.get('description', ''))
        mcp.tool(name=name, description=meta.get('description', ''))(func)

    # Action tools
    for name, meta in ros_node.action_configs.items():
        action_type = get_action(meta['type'])
        func = create_action_tool_function(name, ros_node, action_type, meta.get('description', ''))
        mcp.tool(name=name, description=meta.get('description', ''))(func)

    # Topic tools
    for name, meta in ros_node.topic_configs.items():
        func = create_topic_tool_function(name, ros_node, meta.get('description', ''))
        mcp.tool(name=name, description=meta.get('description', ''))(func)

    # Start ROS spin in thread
    spin_thread = threading.Thread(target=ros_spin, args=(ros_node,))
    spin_thread.daemon = True
    spin_thread.start()

    try:
        mcp.run(transport="http", port=8000)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.shutdown_event.set()
        spin_thread.join()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
