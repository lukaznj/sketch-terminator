"""Streamlit control panel for the Sketch Terminator arm.

Two tabs:

* **Manual Control** -- joint sliders (forward kinematics) beside Cartesian
  sliders (inverse kinematics). Moving either side updates the other and sends
  the arm a trajectory, so the two views never disagree.
* **AI Chat** -- a chat box wired to the ROSA agent node over ROS topics.

Run it with ``streamlit run dashboard.py``; ``master_robot.launch.py`` starts it
headless alongside everything else.

A note on the naming: the GUI labels the joints J1/J2/J3 in the order an
operator thinks about them (base, then shoulder, then elbow), while the
controllers name them joint1/joint2/joint3 in a different order. ``GUI_TO_WIRE``
below is the one place that mapping lives.
"""

import json
import math
import os
import queue
import sys
import time

import rclpy
import streamlit as st
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# kinematics.py is installed into the package's lib/ directory, while this file
# lands under share/. Streamlit does not know about either, so find the right
# one here. Two layouts to cover: installed (ask ament where the package is)
# and a plain source checkout (kinematics.py is one directory over).
def _kinematics_dir():
    try:
        from ament_index_python.packages import get_package_prefix
        installed = os.path.join(
            get_package_prefix('sketch_terminator'), 'lib', 'sketch_terminator'
        )
        if os.path.isdir(installed):
            return installed
    except Exception:
        pass    # not installed -- fall through to the source tree

    source = os.path.join(os.path.dirname(__file__), os.pardir, 'sketch_terminator')
    return source if os.path.isdir(source) else None


_KINEMATICS_DIR = _kinematics_dir()
if _KINEMATICS_DIR:
    sys.path.insert(0, os.path.abspath(_KINEMATICS_DIR))

from kinematics import Kinematics  # noqa: E402  (needs the path set up above)

# GUI joint index -> wire joint index. J1 is the base (joint3), J2 the shoulder
# (joint1), J3 the elbow (joint2).
GUI_TO_WIRE = [2, 0, 1]
JOINT_NAMES = ['joint1', 'joint2', 'joint3']

# Slider ranges, in the GUI's J1/J2/J3 order. These are the arm's usable travel,
# not the motors' full range.
JOINT_LIMITS = [
    (-2.9, 0.0),     # J1 base
    (-3.14, 3.14),   # J2 shoulder
    (-3.14, 1.7),    # J3 elbow
]
CARTESIAN_LIMITS = [
    (0.07, 0.45),    # X, metres
    (-0.45, 0.45),   # Y
    (0.0, 0.09),     # Z, 0 is on the paper
]

# Slow, deliberate moves: the GUI jumps straight to a pose rather than
# streaming a path, so give the arm time to get there.
GUI_MOVE_DURATION = Duration(sec=1, nanosec=500_000_000)
AGENT_TIMEOUT_SEC = 60.0

st.set_page_config(
    page_title='Sketch Terminator Control Panel',
    page_icon='🤖',
    layout='wide',
    initial_sidebar_state='expanded',
)

st.markdown("""
<style>
    @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&family=Outfit:wght@300;400;600&display=swap');

    html, body, [class*="css"] {
        font-family: 'Outfit', sans-serif;
        background-color: #0c0f1d;
        color: #e2e8f0;
    }

    .stApp {
        background: radial-gradient(circle at 50% 50%, #151a30 0%, #080a12 100%);
    }

    h1, h2, h3, h4 {
        font-family: 'Orbitron', sans-serif !important;
        letter-spacing: 2px;
        color: #00f2fe;
        text-shadow: 0 0 10px rgba(0, 242, 254, 0.4);
    }

    /* Glassmorphic cards, applied to Streamlit's bordered containers. */
    div[data-testid="stVerticalBlockBorderWrapper"] {
        background: rgba(21, 26, 48, 0.45) !important;
        backdrop-filter: blur(12px) !important;
        -webkit-backdrop-filter: blur(12px) !important;
        border: 1px solid rgba(255, 255, 255, 0.08) !important;
        border-radius: 16px !important;
        padding: 24px !important;
        margin-bottom: 24px !important;
        box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37) !important;
    }

    .glow-text {
        color: #4facfe;
        text-shadow: 0 0 8px rgba(79, 172, 254, 0.4);
        font-family: 'Orbitron', sans-serif;
    }
</style>
""", unsafe_allow_html=True)


# --- Helpers -------------------------------------------------------------

def clamp(value, limits):
    low, high = limits
    return max(low, min(high, value))


def gui_to_wire(gui_angles):
    """Reorder [J1, J2, J3] into the controller's [joint1, joint2, joint3]."""
    return [gui_angles[GUI_TO_WIRE.index(i)] for i in range(3)]


def wire_to_gui(wire_angles):
    """Reorder [joint1, joint2, joint3] into the GUI's [J1, J2, J3]."""
    return [wire_angles[i] for i in GUI_TO_WIRE]


def forward_kinematics(gui_angles):
    """Cartesian tip position for the GUI's J1/J2/J3 angles."""
    beta, gama, alpha = gui_to_wire(gui_angles)
    return st.session_state.kinematics.get_dk(beta, gama, alpha)


def sync_widget_states():
    """Push the authoritative values back into the slider and text widgets.

    Streamlit widgets own their own state, so after we change a value
    programmatically (from IK, or from a clamp) the widgets have to be told.
    """
    for i, limits in enumerate(JOINT_LIMITS, start=1):
        value = float(st.session_state[f'j{i}_val'])
        st.session_state[f'j{i}_slider'] = clamp(value, limits)
        st.session_state[f'j{i}_text'] = f'{value:.3f}'

    for axis, limits in zip('xyz', CARTESIAN_LIMITS):
        value = float(st.session_state[f'ik_{axis}_val'])
        st.session_state[f'ik_{axis}_slider'] = clamp(value, limits)
        st.session_state[f'ik_{axis}_text'] = f'{value:.3f}'


def read_current_joints(node, timeout_sec=1.0):
    """Read one JointState so the sliders start where the arm actually is."""
    positions = [0.0, 0.0, 0.0]
    received = False

    def callback(msg):
        nonlocal positions, received
        angles = dict(zip(msg.name, msg.position))
        if all(name in angles for name in JOINT_NAMES):
            positions = [angles[name] for name in JOINT_NAMES]
            received = True

    sub = node.create_subscription(JointState, '/joint_states', callback, 10)
    deadline = time.time() + timeout_sec
    while not received and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return positions


def publish_joints(gui_angles):
    """Send the arm to a pose given in the GUI's J1/J2/J3 order."""
    try:
        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in gui_to_wire(gui_angles)]
        point.time_from_start = GUI_MOVE_DURATION

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points.append(point)
        st.session_state.joint_pub.publish(msg)
    except Exception as exc:
        st.toast(f'Could not send the trajectory: {exc}', icon='🛑')


def apply_joint_angles(gui_angles):
    """Validate a pose, then move there and refresh the Cartesian readout."""
    x, y, z = forward_kinematics(gui_angles)

    # The marker is rigid; driving it below the paper jams it into the surface.
    if z < 0.0:
        st.toast('Blocked: that pose would push the marker through the paper (Z < 0).',
                 icon='🛑')
        return False

    for i, angle in enumerate(gui_angles, start=1):
        st.session_state[f'j{i}_val'] = angle
    st.session_state.ik_x_val, st.session_state.ik_y_val, st.session_state.ik_z_val = x, y, z

    publish_joints(gui_angles)
    return True


def apply_cartesian_target():
    """Solve IK for the current XYZ target and move there."""
    target = [st.session_state.ik_x_val, st.session_state.ik_y_val, st.session_state.ik_z_val]
    try:
        gui_angles = wire_to_gui(st.session_state.kinematics.get_ik(*target))
    except Exception:
        st.toast('That point is not reachable.', icon='🛑')
        # Snap the Cartesian readout back to where the arm actually is.
        current = [st.session_state[f'j{i}_val'] for i in (1, 2, 3)]
        (st.session_state.ik_x_val,
         st.session_state.ik_y_val,
         st.session_state.ik_z_val) = forward_kinematics(current)
        return

    for i, angle in enumerate(gui_angles, start=1):
        st.session_state[f'j{i}_val'] = float(angle)
    publish_joints(gui_angles)


def on_joint_change(index, source):
    """Handle a joint slider or text box changing. ``index`` is 1, 2 or 3."""
    limits = JOINT_LIMITS[index - 1]
    try:
        raw = float(st.session_state[f'j{index}_{source}'])
    except ValueError:
        sync_widget_states()
        return

    angles = [st.session_state[f'j{i}_val'] for i in (1, 2, 3)]
    angles[index - 1] = clamp(raw, limits)
    apply_joint_angles(angles)
    sync_widget_states()


def on_cartesian_change(axis, source):
    """Handle an X/Y/Z slider or text box changing."""
    limits = CARTESIAN_LIMITS['xyz'.index(axis)]
    try:
        raw = float(st.session_state[f'ik_{axis}_{source}'])
    except ValueError:
        sync_widget_states()
        return

    st.session_state[f'ik_{axis}_val'] = clamp(raw, limits)
    apply_cartesian_target()
    sync_widget_states()


# --- ROS setup (once per session) ----------------------------------------

if 'kinematics' not in st.session_state:
    st.session_state.kinematics = Kinematics()

if 'ros_node' not in st.session_state:
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('streamlit_gui_node')
    st.session_state.ros_node = node

    st.session_state.joint_pub = node.create_publisher(
        JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
    )
    st.session_state.command_pub = node.create_publisher(String, '/agent/command', 10)

    # Callbacks fire on the Streamlit script thread via spin_once, so the
    # queues just buffer whatever arrived since the last rerun.
    st.session_state.token_queue = queue.Queue()
    st.session_state.response_queue = queue.Queue()
    node.create_subscription(
        String, '/agent/tokens', lambda msg: st.session_state.token_queue.put(msg.data), 10
    )
    node.create_subscription(
        String, '/agent/response', lambda msg: st.session_state.response_queue.put(msg.data), 10
    )

    st.session_state.true_joints = [0.0, 0.0, 0.0]

    def joint_state_callback(msg):
        angles = dict(zip(msg.name, msg.position))
        if all(name in angles for name in JOINT_NAMES):
            st.session_state.true_joints = [angles[name] for name in JOINT_NAMES]

    node.create_subscription(JointState, '/joint_states', joint_state_callback, 10)

    st.session_state.chat_history = [{
        'role': 'assistant',
        'content': 'Hello! I am Sketch Terminator. What would you like me to draw?',
    }]

if 'j1_val' not in st.session_state:
    # Start the sliders from the arm's real pose, so the first nudge does not
    # make it jump across the workspace.
    try:
        wire_angles = read_current_joints(st.session_state.ros_node)
    except Exception:
        wire_angles = [0.0, 0.0, 0.0]

    for i, angle in enumerate(wire_to_gui(wire_angles), start=1):
        st.session_state[f'j{i}_val'] = float(angle)

    x, y, z = forward_kinematics([st.session_state[f'j{i}_val'] for i in (1, 2, 3)])
    st.session_state.ik_x_val, st.session_state.ik_y_val, st.session_state.ik_z_val = x, y, z
    sync_widget_states()


# --- Sidebar -------------------------------------------------------------

logo_path = os.path.join(os.path.dirname(__file__), 'public', 'logo.png')
if os.path.exists(logo_path):
    st.sidebar.image(logo_path, use_container_width=True)
st.sidebar.markdown(
    "<h1 style='text-align: center; font-size: 20px; color: #00f2fe; "
    "text-shadow: 0 0 10px rgba(0, 242, 254, 0.4); margin-top: -10px;'>"
    'SKETCH TERMINATOR</h1>',
    unsafe_allow_html=True,
)
st.sidebar.markdown('---')


@st.fragment(run_every=1.0)
def live_sidebar_status():
    """Poll ROS once a second and show the measured joint angles."""
    if 'ros_node' in st.session_state:
        rclpy.spin_once(st.session_state.ros_node, timeout_sec=0.01)

    st.markdown('### 🦾 Joint Status')
    joints = wire_to_gui(st.session_state.true_joints)
    for label, value in zip(('J1 (Base)', 'J2 (Shoulder)', 'J3 (Elbow)'), joints):
        st.markdown(f'- **{label}:** `{value:.3f} rad`')


with st.sidebar:
    live_sidebar_status()


# --- Manual control ------------------------------------------------------

tab_control, tab_agent = st.tabs(['🕹️ Manual Control', '🤖 AI Chat'])

with tab_control:
    col_fk, col_ik = st.columns(2)

    with col_fk:
        with st.container(border=True):
            st.markdown('<h2>Direct Kinematics</h2>', unsafe_allow_html=True)

            for index, label in enumerate(
                ('Joint 1 (Base)', 'Joint 2 (Shoulder)', 'Joint 3 (Elbow)'), start=1
            ):
                low, high = JOINT_LIMITS[index - 1]
                st.markdown(f'#### {label} [rad]')
                slider_col, text_col = st.columns([3, 1])
                slider_col.slider(
                    label, low, high, key=f'j{index}_slider', step=0.01,
                    label_visibility='collapsed',
                    on_change=on_joint_change, args=(index, 'slider'),
                )
                text_col.text_input(
                    f'{label} value', key=f'j{index}_text',
                    label_visibility='collapsed',
                    on_change=on_joint_change, args=(index, 'text'),
                )

            st.markdown("<h3 class='glow-text'>Computed Cartesian Pose:</h3>",
                        unsafe_allow_html=True)
            x, y, z = forward_kinematics([st.session_state[f'j{i}_val'] for i in (1, 2, 3)])
            st.code('\n'.join(
                f'{axis}: {value * 1000.0:7.1f} mm  ({value:.4f} m)'
                for axis, value in zip('XYZ', (x, y, z))
            ))

    with col_ik:
        with st.container(border=True):
            st.markdown('<h2>Inverse Kinematics</h2>', unsafe_allow_html=True)

            for axis in 'xyz':
                low, high = CARTESIAN_LIMITS['xyz'.index(axis)]
                st.markdown(f'#### {axis.upper()} Position [m]')
                slider_col, text_col = st.columns([3, 1])
                slider_col.slider(
                    f'{axis.upper()} position', low, high, key=f'ik_{axis}_slider',
                    step=0.005, label_visibility='collapsed',
                    on_change=on_cartesian_change, args=(axis, 'slider'),
                )
                text_col.text_input(
                    f'{axis.upper()} value', key=f'ik_{axis}_text',
                    label_visibility='collapsed',
                    on_change=on_cartesian_change, args=(axis, 'text'),
                )

            st.markdown("<h3 class='glow-text'>Computed Joint Values:</h3>",
                        unsafe_allow_html=True)
            try:
                gui_angles = wire_to_gui(st.session_state.kinematics.get_ik(
                    st.session_state.ik_x_val,
                    st.session_state.ik_y_val,
                    st.session_state.ik_z_val,
                ))
                st.code('\n'.join(
                    f'J{i} ({name}): {angle:7.4f} rad ({math.degrees(angle):6.1f}°)'
                    for i, (name, angle) in enumerate(
                        zip(('Base', 'Shoulder', 'Elbow'), gui_angles), start=1
                    )
                ))
            except Exception:
                st.error('IK failed: that target is out of reach.')


# --- AI chat -------------------------------------------------------------

with tab_agent:
    with st.container(border=True):
        st.markdown('<h2>ROSA Autonomous AI Command Center</h2>', unsafe_allow_html=True)
        st.markdown("<p style='color:#a0aec0;'>Tell the robot what to draw, in plain "
                    'language.</p>', unsafe_allow_html=True)
        st.markdown('---')

        for message in st.session_state.chat_history:
            with st.chat_message(message['role']):
                st.markdown(message['content'])

        command = st.chat_input(
            "e.g. 'draw a line from the car to the traffic light, avoiding the cat'"
        )

        if command:
            st.session_state.chat_history.append({'role': 'user', 'content': command})
            with st.chat_message('user'):
                st.markdown(command)

            with st.chat_message('assistant'):
                status_container = st.empty()
                response_container = st.empty()

                cmd_msg = String()
                cmd_msg.data = command
                st.session_state.command_pub.publish(cmd_msg)

                # Drop anything left over from a previous query.
                st.session_state.token_queue = queue.Queue()
                st.session_state.response_queue = queue.Queue()

                full_text = ''
                completed = False
                deadline = time.time() + AGENT_TIMEOUT_SEC

                with st.spinner('Agent is thinking...'):
                    status_container.info('🧠 Starting the reasoning process...')

                    while time.time() < deadline:
                        rclpy.spin_once(st.session_state.ros_node, timeout_sec=0.02)

                        if not st.session_state.response_queue.empty():
                            st.session_state.response_queue.get()
                            completed = True
                            break

                        while not st.session_state.token_queue.empty():
                            try:
                                event = json.loads(st.session_state.token_queue.get_nowait())
                            except (json.JSONDecodeError, queue.Empty):
                                continue

                            kind = event.get('type', '')
                            content = event.get('content', '')
                            # Any activity means the agent is alive, so push the
                            # timeout back rather than cutting off a long answer.
                            deadline = time.time() + AGENT_TIMEOUT_SEC

                            if kind == 'token':
                                full_text += content
                                response_container.markdown(full_text)
                            elif kind == 'tool_start':
                                status_container.info(f'⚙️ Running tool `{content}`...')
                            elif kind == 'tool_end':
                                status_container.success(f'✅ Tool `{content}` finished.')
                            elif kind == 'error':
                                st.error(f'❌ Agent error: {content}')
                                completed = True
                                break

                        if completed:
                            break

                if completed:
                    st.session_state.chat_history.append(
                        {'role': 'assistant', 'content': full_text}
                    )
                    status_container.empty()
                else:
                    st.warning('⚠️ Timed out. Is agent_node running?')
