#!/usr/bin/env python3
"""Exposes the ROSA agent over ROS topics, so the GUI can talk to it.

Commands arrive as plain text on ``/agent/command``. The agent's reply is
streamed token by token on ``/agent/tokens`` (as JSON events, so the GUI can
also show which tool is running) and the finished answer is published once on
``/agent/response``.

Topics
------
in   ``/agent/command``    natural-language request
out  ``/agent/tokens``     {"type": "token"|"tool_start"|"tool_end"|"error", ...}
out  ``/agent/response``   the complete answer, once the query finishes
"""

import asyncio
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from agent import create_agent


class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')
        self.get_logger().info('Creating the ROSA agent...')
        self.agent = create_agent(streaming=True, verbose=False)
        self.get_logger().info('ROSA agent ready.')

        self.create_subscription(String, '/agent/command', self.command_callback, 10)
        self.response_pub = self.create_publisher(String, '/agent/response', 10)
        self.token_pub = self.create_publisher(String, '/agent/tokens', 10)

        self.get_logger().info("Waiting for commands on '/agent/command'...")

    def command_callback(self, msg):
        command = msg.data.strip()
        if not command:
            return

        self.get_logger().info(f"Received command: '{command}'")
        # NOTE: this blocks the executor until the whole query finishes, so the
        # node handles one command at a time. That matches the hardware anyway
        # -- there is only one arm, and it can only draw one path at a time.
        asyncio.run(self.execute_command(command))

    def publish_event(self, event_type, content, **extra):
        msg = String()
        msg.data = json.dumps({'type': event_type, 'content': content, **extra})
        self.token_pub.publish(msg)

    async def execute_command(self, query):
        full_response = ''
        try:
            async for event in self.agent.astream(query):
                kind = event.get('type', '')
                content = event.get('content', '')

                if kind == 'token':
                    full_response += content
                    self.publish_event('token', content)
                elif kind == 'tool_start':
                    name = event.get('name', 'tool')
                    self.publish_event('tool_start', name)
                    self.get_logger().info(f'Tool started: {name}')
                elif kind == 'tool_end':
                    name = event.get('name', 'tool')
                    self.publish_event('tool_end', name, output=content)
                    self.get_logger().info(f'Tool finished: {name}')
                elif kind == 'error':
                    self.publish_event('error', content)
                    self.get_logger().error(f'ROSA error: {content}')

            response = String()
            response.data = full_response
            self.response_pub.publish(response)
            self.get_logger().info('Published the final response.')

        except Exception as exc:
            self.get_logger().error(f'Command failed: {exc}')
            # The GUI waits on /agent/response, so it has to hear about this
            # too or it will just sit there until its timeout.
            self.publish_event('error', str(exc))
            response = String()
            response.data = f'Error: {exc}'
            self.response_pub.publish(response)


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
