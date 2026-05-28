#!/usr/bin/env python3

import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from ament_index_python.packages import get_package_share_directory
from rosa import ROSA, RobotSystemPrompts
from .ros2_introspection import scan_ros2_environment, format_state_for_prompt
from .tools import TOOLS

def get_llm():
    """Create and return the ChatOpenAI model using credentials from package config/.env or environment."""
    # 1. If key is already in the environment, use it!
    api_key = os.getenv("OPENAI_API_KEY")
    if api_key:
        return ChatOpenAI(
            model_name=os.getenv("LLM_MODEL", "gpt-4o-mini"),
            temperature=float(os.getenv("LLM_TEMPERATURE", "0.0")),
            openai_api_key=api_key
        )

    # 2. Compile a list of candidate .env paths
    candidates = []

    # A. Check package share directory
    try:
        package_share = get_package_share_directory('sketch_terminator')
        candidates.append(os.path.join(package_share, 'config', '.env'))
    except Exception:
        pass

    # B. Walk up from current working directory
    cwd = os.getcwd()
    for _ in range(5):
        candidates.append(os.path.join(cwd, '.env'))
        candidates.append(os.path.join(cwd, 'config', '.env'))
        candidates.append(os.path.join(cwd, 'src', 'sketch_terminator', 'config', '.env'))
        parent = os.path.dirname(cwd)
        if parent == cwd:
            break
        cwd = parent

    # C. Walk up from the location of this python file (__file__)
    this_dir = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        candidates.append(os.path.join(this_dir, '.env'))
        candidates.append(os.path.join(this_dir, 'config', '.env'))
        candidates.append(os.path.join(this_dir, '..', 'config', '.env'))
        candidates.append(os.path.join(this_dir, 'src', 'sketch_terminator', 'config', '.env'))
        parent = os.path.dirname(this_dir)
        if parent == this_dir:
            break
        this_dir = parent

    # 3. Load the first candidate path that exists
    loaded = False
    for path in candidates:
        abs_path = os.path.abspath(path)
        if os.path.isfile(abs_path):
            load_dotenv(abs_path)
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key:
                loaded = True
                break

    # 4. Fallback to standard load_dotenv() if nothing was loaded yet
    if not loaded:
        load_dotenv()
        api_key = os.getenv("OPENAI_API_KEY")

    if not api_key:
        checked_str = "\n".join(sorted(list(set([f"  - {os.path.abspath(p)} (Exists: {os.path.isfile(p)})" for p in candidates]))))
        raise ValueError(
            f"OPENAI_API_KEY not found. Checked the following .env locations:\n{checked_str}\n"
            "Please ensure it is defined in one of these files or export it directly in your terminal:\n"
            "  export OPENAI_API_KEY='your-key-here'"
        )

    model = os.getenv("LLM_MODEL", "gpt-4o-mini")
    temperature = float(os.getenv("LLM_TEMPERATURE", "0.0"))

    return ChatOpenAI(
        model_name=model,
        temperature=temperature,
        openai_api_key=api_key
    )

def create_agent(
    streaming=True,
    verbose=False,
    ros2_state=None
):
    """
    Creates and returns a configured ROSA agent initialized with our custom tools
    and ROS 2 environment introspection context.
    """
    llm = get_llm()

    # Pre-fetch ROS2 environment snapshot
    if ros2_state is None:
        try:
            ros2_state = scan_ros2_environment()
        except Exception:
            ros2_state = None

    env_description = format_state_for_prompt(ros2_state) if ros2_state else "ROS2 Environment state not scanned."

    prompts = RobotSystemPrompts(
        embodiment_and_persona=(
            "You are Antigravity-ROSA, a state-of-the-art ROS 2 AI robotics assistant. "
            "You are pair programming and coordinating operations for a 3-DOF drawing robotic manipulator."
        ),
        about_your_operators=(
            "Your operators are lead robotics engineers and researchers. "
            "Be highly concise, precise, and technical. Avoid excessive politeness."
        ),
        about_your_environment=(
            "The following is a COMPLETE live snapshot of the ROS2 "
            "environment. This data is ground truth:\n\n"
            + env_description
        ),
        critical_instructions=(
            "You have access to specialized tools to interact with the robot:\n"
            "1. 'get_detected_objects': Fetch visible object positions (car, cat, traffic light) in base frame coordinates (in mm).\n"
            "2. 'plan_and_move_to_object': Plan a 2D collision-free path from start to goal avoiding specific obstacles and drive the robot.\n"
            "3. 'move_robot_joints': Move the joints [joint1, joint2, joint3] directly to specified angles (in radians).\n"
            "4. 'get_joint_states': Get the current raw positions of all joints.\n"
            "5. 'get_end_effector_pose': Use direct kinematics to get the current Cartesian [X, Y, Z] position of the marker.\n\n"
            "When the user asks to plan a path or move between objects (e.g. 'Idi od car do traffic light i izbjegni cat'), "
            "use the 'plan_and_move_to_object' tool directly. Do not call intermediate tools unless asked."
        )
    )

    return ROSA(
        ros_version=2,
        llm=llm,
        tools=TOOLS,
        prompts=prompts,
        streaming=streaming,
        verbose=verbose
    )
