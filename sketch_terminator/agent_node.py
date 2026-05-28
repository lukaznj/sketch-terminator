#!/usr/bin/env python3

import asyncio
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .agent import create_agent

class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')
        self.get_logger().info("Initializing prarob_integration ROSA agent node...")

        # Initialize ROSA agent
        try:
            self.agent = create_agent(streaming=True, verbose=False)
            self.get_logger().info("ROSA agent created successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to create ROSA agent: {e}")
            raise e

        # Subscription for queries
        self.command_sub = self.create_subscription(
            String,
            '/agent/command',
            self.command_callback,
            10
        )

        # Publisher for responses
        self.response_pub = self.create_publisher(
            String,
            '/agent/response',
            10
        )
        
        # Publisher for real-time tokens (streaming)
        self.token_pub = self.create_publisher(
            String,
            '/agent/tokens',
            10
        )

        self.get_logger().info("AgentNode initialized. Awaiting commands on '/agent/command'...")

    def command_callback(self, msg):
        command = msg.data.strip()
        if not command:
            return

        self.get_logger().info(f"Received command: '{command}'")
        
        # Execute query asynchronously
        asyncio.run(self.execute_command(command))

    async def execute_command(self, query):
        full_response = ""
        try:
            async for event in self.agent.astream(query):
                kind = event.get("type", "")
                if kind == "token":
                    token = event.get("content", "")
                    full_response += token
                    
                    # Publish token stream in real time
                    token_msg = String()
                    token_msg.data = json.dumps({"type": "token", "content": token})
                    self.token_pub.publish(token_msg)

                elif kind == "tool_start":
                    name = event.get("name", "tool")
                    tool_msg = String()
                    tool_msg.data = json.dumps({"type": "tool_start", "content": name})
                    self.token_pub.publish(tool_msg)
                    self.get_logger().info(f"Tool execution started: {name}")

                elif kind == "tool_end":
                    name = event.get("name", "tool")
                    output = event.get("content", "")
                    tool_msg = String()
                    tool_msg.data = json.dumps({"type": "tool_end", "content": name, "output": output})
                    self.token_pub.publish(tool_msg)
                    self.get_logger().info(f"Tool execution completed: {name}")

                elif kind == "error":
                    err_msg = event.get("content", "")
                    self.get_logger().error(f"ROSA Error: {err_msg}")
                    error_payload = String()
                    error_payload.data = json.dumps({"type": "error", "content": err_msg})
                    self.token_pub.publish(error_payload)

            # Publish final completed response
            final_msg = String()
            final_msg.data = full_response
            self.response_pub.publish(final_msg)
            self.get_logger().info("Query response published successfully.")

        except Exception as e:
            self.get_logger().error(f"Error during command execution: {e}")
            err_msg = String()
            err_msg.data = f"Error: {str(e)}"
            self.response_pub.publish(err_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
