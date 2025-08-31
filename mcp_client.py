import asyncio
import json
from fastmcp import Client

async def main():
    """An example of using the fastmcp client to connect to a local HTTP server."""
    
    # Specify the URL of the running server
    SERVER_URL = "http://127.0.0.1:8000/mcp"
    
    # Create a client instance, connecting to the HTTP server
    async with Client(SERVER_URL) as client:
        print(f"Connected to MCP server at {SERVER_URL}")
        
        # --- List tools ---
        print("\n--- Listing available tools ---")
        tools = await client.list_tools()
        for tool in tools:
            print(f"Name: {tool.name}")
            print(f"Description: {tool.description}")
            input_schema_json = json.dumps(tool.inputSchema, indent=2)
            print(f"Input Parameters:\n{input_schema_json}")
            print("-" * 20)

if __name__ == "__main__":
    asyncio.run(main())