import rclpy
from rclpy.node import Node
import asyncio
import threading
import json
from typing import Dict, Any, List
from fastmcp import FastMCP
import sys
import os
import yaml
from geometry_msgs.msg import Point
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.utilities import get_service
from rosidl_runtime_py.utilities import get_action
import inspect
import types
from pydantic import create_model, BaseModel, Field


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
    """Converts an rclpy.task.Future to a concurrent.futures.Future."""
    concurrent_future = asyncio.Future()

    def on_done(rclpy_future):
        try:
            result = rclpy_future.result()
            loop.call_soon_threadsafe(concurrent_future.set_result, result)
        except Exception as e:
            loop.call_soon_threadsafe(concurrent_future.set_exception, e)

    rclpy_future.add_done_callback(on_done)
    return concurrent_future


def ros_message_type_to_json_schema(ros_message_type):
    """Converts a ROS message type to a JSON schema."""
    schema = {"type": "object", "properties": {}, "required": []}
    for field_name, field_type_string in ros_message_type.get_fields_and_field_types().items():
        is_array = False
        if field_type_string.endswith('[]'):
            is_array = True
            field_type_string = field_type_string[:-2] # Remove '[]'

        # Handle basic types
        if field_type_string in ['bool', 'byte', 'char', 'float32', 'float64', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']:
            prop = {"type": "number"} # All numeric types
            if field_type_string == 'bool':
                prop = {"type": "boolean"}
        elif field_type_string == 'string':
            prop = {"type": "string"}
        elif field_type_string in ['time', 'duration']: # ROS specific types, treat as objects or strings
            prop = {"type": "object", "properties": {"sec": {"type": "integer"}, "nanosec": {"type": "integer"}}}
        else:
            # Nested message type
            try:
                nested_msg_type = get_message(field_type_string)
                prop = ros_message_type_to_json_schema(nested_msg_type)
            except Exception:
                # Fallback for unknown types, treat as generic object
                prop = {"type": "object"}

        if is_array:
            prop = {"type": "array", "items": prop}

        schema["properties"][field_name] = prop
        schema["required"].append(field_name) # Assuming all fields are required for simplicity

    return schema

def convert_dict_to_ros_message(data: Dict, ros_message_type):
    """Recursively converts a dictionary or Pydantic model to a ROS message object."""
    ros_msg = ros_message_type()
    for field_name, field_type_string in ros_message_type.get_fields_and_field_types().items():
        if field_name not in data:
            continue

        value = data[field_name]

        # If the value is a Pydantic model, convert it to a dictionary
        if isinstance(value, BaseModel):
            value = value.model_dump()

        is_array = False
        if field_type_string.endswith('[]'):
            is_array = True
            field_type_string = field_type_string[:-2]

        if is_array:
            nested_type = get_message(field_type_string)
            converted_list = []
            for item in value:
                # If item is a Pydantic model, convert it to a dictionary first
                if isinstance(item, BaseModel):
                    item = item.model_dump()

                if hasattr(nested_type, 'get_fields_and_field_types'): # It's a nested ROS message
                    converted_list.append(convert_dict_to_ros_message(item, nested_type))
                else:
                    converted_list.append(item) # Basic type in array
            setattr(ros_msg, field_name, converted_list)
        elif hasattr(get_message(field_type_string), 'get_fields_and_field_types'): # Nested ROS message
            nested_type = get_message(field_type_string)
            setattr(ros_msg, field_name, convert_dict_to_ros_message(value, nested_type))
        else: # Basic type
            setattr(ros_msg, field_name, value)
    return ros_msg


def ros_message_type_to_pydantic_model(ros_message_type):
    """Dynamically creates a Pydantic model from a ROS message type."""
    fields = {}

    for field_name, field_type_string in ros_message_type.get_fields_and_field_types().items():
        is_array = False
        if field_type_string.endswith('[]'):
            is_array = True
            field_type_string = field_type_string[:-2]

        py_type = Any # Default to Any for unknown types or complex nested types

        if field_type_string == 'bool':
            py_type = bool
        elif field_type_string == 'string':
            py_type = str
        elif field_type_string in ['float32', 'float64']:
            py_type = float
        elif field_type_string in ['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']:
            py_type = int
        elif field_type_string in ['time', 'duration']:
            py_type = Dict[str, int] # Represent time/duration as dicts
        else:
            # Nested message type, recursively create Pydantic model
            try:
                nested_msg_type = get_message(field_type_string)
                py_type = ros_message_type_to_pydantic_model(nested_msg_type)
            except Exception:
                pass # Keep as Any if cannot resolve

        if is_array:
            py_type = List[py_type]

        # Define the field with its type and a default value (or Field(...))
        # Add title and description for better schema generation
        fields[field_name] = (py_type, Field(..., title=field_name.replace('_', ' ').title(), description=f"The {field_name.replace('_', ' ').title()} coordinate."))

    model_name = ros_message_type.__class__.__name__
    if model_name == "Metaclass_Point":
        model_name = "Point"

    DynamicModel = create_model(model_name, **fields)
    return DynamicModel


def create_dynamic_action_tool_function(name, description, action_type, ros_node_instance):
    """
    Dynamically creates an async function for an MCP action tool.
    The function signature is generated based on the ROS action's goal message.
    """
    goal_fields = action_type.Goal.get_fields_and_field_types()
    
    # Construct the function signature string
    param_strings = []
    for field_name, field_type_string in goal_fields.items():
        # Convert ROS type string to Python type hint string
        py_type_hint = "Any" # Default
        if field_type_string == 'bool':
            py_type_hint = "bool"
        elif field_type_string == 'string':
            py_type_hint = "str"
        elif field_type_string in ['float32', 'float64']:
            py_type_hint = "float"
        elif field_type_string in ['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']:
            py_type_hint = "int"
        elif field_type_string in ['time', 'duration']:
            py_type_hint = "Dict[str, int]" # Represent time/duration as dicts
        else:
            # Nested message type, recursively create Pydantic model
            try:
                nested_ros_msg_type = get_message(field_type_string)
                py_type_hint = ros_message_type_to_pydantic_model(nested_ros_msg_type).__name__ # Use the name of the Pydantic model

            except Exception:
                py_type_hint = "Any" # Fallback if model creation failed earlier or type is truly unknown

        if field_type_string.endswith('[]'): # Handle arrays
            py_type_hint = f"List[{py_type_hint}]"

        param_strings.append(f"{field_name}: {py_type_hint}")
    
    signature_str = ", ".join(param_strings)

    # Construct the function body
    # Collect all dynamic arguments into an 'args' dictionary
    args_collection_str = "args = {" + ", ".join([f"'{field_name}': {field_name}" for field_name in goal_fields.keys()]) + "}"

    # The actual call to run_action
    function_body = f"""
async def {name}_tool({signature_str}):
    {args_collection_str}
    # Convert the dictionary args to the ROS action goal message type
    ros_goal_msg = convert_dict_to_ros_message(args, action_type.Goal)
    await ros_node_instance.run_action(name, ros_goal_msg, None) # Pass None for stream_queue
"""
    # Use exec to create the function
    local_namespace = {
        "ros_node_instance": ros_node_instance,
        "name": name,
        "action_type": action_type, # Pass action_type to local_namespace
        "convert_dict_to_ros_message": convert_dict_to_ros_message,
        "Dict": Dict,
        "List": List,
        "Any": Any,
        "get_message": get_message, # Ensure get_message is available
        "BaseModel": BaseModel, # Pass BaseModel for dynamic Pydantic model creation
        "Field": Field, # Pass Field for dynamic Pydantic model creation
        "create_model": create_model # Pass create_model for dynamic Pydantic model creation
    }
    # Pre-generate and add dynamically created Pydantic models to the local namespace
    # This ensures they are available when constructing the function signature and during exec
    for field_name, field_type_string in goal_fields.items():
        # Determine the actual ROS message type for nested fields
        if not field_type_string.endswith('[]') and not field_type_string in ['bool', 'string', 'float32', 'float64', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'time', 'duration']:
            try:
                nested_ros_msg_type = get_message(field_type_string)
                # Removed DEBUG log statement
                dynamic_model = ros_message_type_to_pydantic_model(nested_ros_msg_type)
                local_namespace[dynamic_model.__name__] = dynamic_model
            except Exception as e:
                ros_node_instance.get_logger().warn(f"Could not create Pydantic model for {field_type_string}: {e}")
                pass # Keep as Any if cannot resolve

    # Construct the function signature string
    param_strings = []
    for field_name, field_type_string in goal_fields.items():
        # Convert ROS type string to Python type hint string
        py_type_hint = "Any" # Default
        if field_type_string == 'bool':
            py_type_hint = "bool"
        elif field_type_string == 'string':
            py_type_hint = "str"
        elif field_type_string in ['float32', 'float64']:
            py_type_hint = "float"
        elif field_type_string in ['int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']:
            py_type_hint = "int"
        elif field_type_string in ['time', 'duration']:
            py_type_hint = "Dict[str, int]" # Represent time/duration as dicts
        else: # Assume nested messages are passed as dictionaries
            # Use the dynamically created Pydantic model for nested types
            try:
                nested_ros_msg_type = get_message(field_type_string)
                # Use the name of the Pydantic model, which should now be in local_namespace
                py_type_hint = ros_message_type_to_pydantic_model(nested_ros_msg_type).__name__
            except Exception:
                py_type_hint = "Any" # Fallback if model creation failed earlier or type is truly unknown

        if field_type_string.endswith('[]'): # Handle arrays
            py_type_hint = f"List[{py_type_hint}]"

        param_strings.append(f"{field_name}: {py_type_hint}")

    exec(function_body, globals(), local_namespace)
    
    return local_namespace[f"{name}_tool"]


mcp = FastMCP("ROS2 MCP Gateway")

class MCPGateway(Node):
    def __init__(self, name: str):
        super().__init__(name)

        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        if not config_file or not os.path.exists(config_file):
            self.get_logger().error(f"Config file not found or not specified: {config_file}")
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
        from rosidl_runtime_py.utilities import get_message
        for topic_name, meta in self.topic_configs.items():
            self.get_logger().info(f"Subscribing to topic: {topic_name}")
            msg_type = get_message(meta['type'])

            def callback(msg, topic_name=topic_name):
                self.get_logger().info(f"Received message on topic: {topic_name}")
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
            self.get_logger().error(f'Service {name} call timed out after 3 seconds.')
            return {"error": f"Service call to {name} timed out."}

    async def run_action(self, name: str, args: Dict[str, Any], stream_queue: asyncio.Queue):
        from rclpy.action import GoalResponse, CancelResponse
        client, action_type = self.mcp_actions[name]
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Action {name} not available, waiting...')

        goal_msg = action_type.Goal()
        for key, val in args.items():
            setattr(goal_msg, key, val)

        future = client.send_goal_async(goal_msg, feedback_callback=lambda fb: stream_queue.put_nowait({"type": "progress", "feedback": ros_message_to_dict(fb.feedback)}))
        goal_handle = await future

        if not goal_handle.accepted:
            await stream_queue.put({"type": "error", "message": "Goal rejected"})
            return

        result_future = goal_handle.get_result_async()
        result = await result_future
        await stream_queue.put({"type": "result", "result": ros_message_to_dict(result.result)})


    async def get_latest_topic(self, name: str) -> Dict[str, Any]:
        self.get_logger().info(f"Getting latest message for topic: {name}")
        msg = self.mcp_latest_msgs.get(name)
        if msg is None:
            self.get_logger().warn(f"No message received yet for topic: {name}")
            return {"error": "No message received yet."}
        return ros_message_to_dict(msg)


# Global placeholder for ROS node
ros_node: MCPGateway = None

def main():
    import sys
    rclpy.init()

    global ros_node
    
    ros_node = MCPGateway("ros2_mcp_gateway")

    def create_service_tool(name, meta):
        @mcp.tool(name=name, description=meta['description'])
        async def service_tool(args: Dict[str, Any]) -> Dict:
            print("Calling service " + name)
            return await ros_node.call_service(name, args)
        return service_tool

    for name, meta in ros_node.service_configs.items():
        create_service_tool(name, meta)

    def create_action_tool(name, meta):
        print("Jakub log", name, meta)
        action_type = get_action(meta['type'])
        
        # Dynamically create the action tool function
        dynamic_action_tool_func = create_dynamic_action_tool_function(
            name, meta['description'], action_type, ros_node
        )

        # Apply the mcp.tool decorator to the dynamically created function
        # The decorator will infer parameters from the function's signature
        decorated_tool = mcp.tool(
            name=name,
            description=meta['description']
        )(dynamic_action_tool_func)
        
        return decorated_tool

    for name, meta in ros_node.action_configs.items():
        create_action_tool(name, meta)

    def create_topic_tool(name, meta):
        @mcp.tool(name=name, description=meta['description'])
        async def topic_tool() -> Dict:
            return await ros_node.get_latest_topic(name)
        return topic_tool

    for name, meta in ros_node.topic_configs.items():
        create_topic_tool(name, meta)

    loop = asyncio.get_event_loop()
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

def ros_spin(node):
    while rclpy.ok() and not node.shutdown_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)


if __name__ == '__main__':
    main()
