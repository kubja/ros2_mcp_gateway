# ROS2 MCP Gateway

A gateway that exposes a robot's services, actions, and topics as MCP (Model Context Protocol) tools, allowing agents to interact with your ROS2 robot via MCP.

## Features

*   **ROS2 Integration:** Seamlessly bridges ROS2 communication with FastMCP.
*   **Configurable Interfaces:** Services, actions, and topics to be exposed are configurable via a YAML file.
*   **HTTP Transport:** Communicates over HTTP for broad compatibility.
*   **Graceful Shutdown:** Supports clean shutdown using CTRL+C.
*   **Python Virtual Environment:** Manages Python dependencies in an isolated environment.

## Setup and Installation

Follow these steps to set up and build the `ros2_mcp_gateway` workspace:

1.  **Navigate to the Workspace:**
    ```bash
    cd /home/jakub/code/ros2_mcp_gateway
    ```

2.  **Create and Activate Python Virtual Environment:**
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```

3.  **Install Python Dependencies:**
    ```bash
    pip install fastmcp
    ```

4.  **Build the ROS2 Workspace:**
    ```bash
    colcon build
    ```

## Configuration

The services, actions, and topics exposed by the gateway are configured via the `mcp_gateway.yaml` file located in the `config` directory of the `ros2_mcp_gateway` package.

**`src/ros2_mcp_gateway/config/mcp_gateway.yaml` example:**

```yaml
service_configs:
  get_status:
    type: my_package/srv/GetStatus
    description: Retrieve current robot system status.

action_configs:
  navigate_to:
    type: my_package/action/Navigate
    description: Navigate the robot to a given target pose.

topic_configs:
  status:
    type: my_package/msg/Status
    description: Get the latest system status message.
```

You can modify this file to define the ROS2 interfaces you wish to expose as FastMCP tools.

## Running the Gateway

To run the `ros2_mcp_gateway` node with the YAML configuration:

1.  **Activate the virtual environment and source ROS2 setup:**
    ```bash
    source .venv/bin/activate
    source install/setup.bash
    ```

2.  **Launch the gateway node:**
    ```bash
    ros2 launch ros2_mcp_gateway mcp_gateway.launch.py
    ```
    The server will start on `http://127.0.0.1:8000/mcp/` (or the port configured in `gateway_node.py`).

## Testing the Server

You can test the FastMCP server using a Python client. Save the following code as `test_client.py` in your project root:

```python
import asyncio
from fastmcp import Client

async def main():
    # Connect to the FastMCP server running on HTTP
    # The default port for FastMCP HTTP transport is 8003 (as configured)
    # and the default path is /mcp
    async with Client("http://localhost:8000/mcp") as client:
        print("Connected to FastMCP server.")
        tools = await client.list_tools()
        print("Available tools:")
        for tool in tools:
            print(f"- {tool.name}: {tool.description}")

if __name__ == "__main__":
    asyncio.run(main())
```

To run the client:

1.  **Ensure the `ros2_mcp_gateway` node is running.**
2.  **Open a new terminal, activate your virtual environment, and run the client script:**
    ```bash
    source .venv/bin/activate
    python test_client.py
    ```

This will connect to your running FastMCP server and print a list of the tools it exposes.

## License

This project is licensed under the Apache-2.0 License.
