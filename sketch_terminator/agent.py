#!/usr/bin/env python3
"""Builds the ROSA agent that drives the robot from natural language.

ROSA (the JPL ROS Agent) wires an LLM up to the live ROS graph. On top of its
built-in introspection we hand it the tools in ``tools.py``, which are the only
way it can actually command the arm.

Configuration comes from ``config/.env`` -- copy ``config/.env.example`` and
fill in your OpenAI key.
"""

import os

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from rosa import ROSA, RobotSystemPrompts

from ros2_introspection import scan_ros2_environment, format_state_for_prompt
from tools import TOOLS

DEFAULT_MODEL = 'gpt-5-mini'


def load_config():
    """Load ``config/.env`` into the environment, unless it is already set.

    Looks in the installed package's share directory first, then falls back to
    python-dotenv's own search so the agent also runs straight from a source
    checkout.
    """
    if os.getenv('OPENAI_API_KEY'):
        return

    try:
        share_env = os.path.join(
            get_package_share_directory('sketch_terminator'), 'config', '.env'
        )
        if os.path.isfile(share_env):
            load_dotenv(share_env)
            return
    except Exception:
        # Package is not installed -- fall through to the source-tree search.
        pass

    load_dotenv()


def get_llm():
    """Create the chat model, raising a clear error if the key is missing."""
    load_config()

    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        raise ValueError(
            'OPENAI_API_KEY is not set. Copy config/.env.example to config/.env '
            'and fill in your key, or export it in your shell.'
        )

    return ChatOpenAI(
        model=os.getenv('LLM_MODEL', DEFAULT_MODEL),
        temperature=float(os.getenv('LLM_TEMPERATURE', '0.0')),
        api_key=api_key,
    )


def describe_environment(scan_environment):
    """Snapshot the live ROS graph for the system prompt.

    Scanning shells out to ``ros2 topic/service/action list`` and then to
    ``ros2 interface show`` once per type, which costs roughly 15 seconds at
    startup. It is off by default for that reason -- ROSA can still discover
    everything on demand through its own tools, just a little slower per query.
    """
    if not scan_environment:
        return ('ROS 2 environment scanning is disabled. Use your introspection '
                'tools to discover topics, services and actions as you need them.')

    return ('The following is a live snapshot of the ROS 2 environment. '
            'Treat it as ground truth:\n\n'
            + format_state_for_prompt(scan_ros2_environment()))


def create_agent(streaming=True, verbose=False, scan_environment=False):
    """Create the ROSA agent, wired up to this robot's tools and prompts."""
    prompts = RobotSystemPrompts(
        embodiment_and_persona=(
            'You are Sketch Terminator, the assistant for a 3-DOF drawing '
            'robot arm. The arm holds a marker over a sheet of paper and a '
            'fixed overhead camera watches the paper.'
        ),
        about_your_operators=(
            'Your operators are robotics engineers. Be concise, precise and '
            'technical. Skip the pleasantries.'
        ),
        about_your_environment=describe_environment(scan_environment),
        critical_instructions=(
            'You have five tools for working with the robot:\n'
            "1. 'get_detected_objects' - what the camera sees, in base frame metres.\n"
            "2. 'plan_and_move_to_object' - plan a collision-free path between two "
            'objects and draw it.\n'
            "3. 'move_robot_joints' - drive the joints directly, in radians.\n"
            "4. 'get_joint_states' - read the current joint angles.\n"
            "5. 'get_end_effector_pose' - where the marker tip is, via forward "
            'kinematics.\n\n'
            "When asked to draw between objects (e.g. 'go from the car to the "
            "traffic light avoiding the cat'), call 'plan_and_move_to_object' "
            'directly. It handles lifting, drawing and parking on its own, so do '
            'not stack other tool calls around it unless you are asked to.'
        ),
    )

    return ROSA(
        ros_version=2,
        llm=get_llm(),
        tools=TOOLS,
        prompts=prompts,
        streaming=streaming,
        verbose=verbose,
    )
