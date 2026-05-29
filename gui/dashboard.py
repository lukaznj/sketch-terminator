import os
import asyncio
import time
import queue
import json
import streamlit as st
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from builtin_interfaces.msg import Duration

from sketch_terminator.kinematics import Kinematics

# Set page config
st.set_page_config(
    page_title="Sketch Terminator ROS 2 Control Panel",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom Premium CSS Styling (Glassmorphism & Cyber-Dark UI)
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
    
    /* Premium Glassmorphic Cards applied to Streamlit's container borders */
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
    
    /* Interactive Cyber Button */
    .stButton>button {
        background: linear-gradient(135deg, #00f2fe 0%, #4facfe 100%) !important;
        color: #080a12 !important;
        font-weight: 700 !important;
        font-family: 'Orbitron', sans-serif !important;
        border: none !important;
        border-radius: 8px !important;
        padding: 10px 24px !important;
        box-shadow: 0 0 15px rgba(0, 242, 254, 0.3) !important;
        transition: all 0.3s ease !important;
    }
    
    .stButton>button:hover {
        transform: translateY(-2px) !important;
        box-shadow: 0 0 25px rgba(0, 242, 254, 0.6) !important;
    }

    /* Subtitle glow */
    .glow-text {
        color: #4facfe;
        text-shadow: 0 0 8px rgba(79, 172, 254, 0.4);
        font-family: 'Orbitron', sans-serif;
    }
</style>
""", unsafe_allow_html=True)

# Function to fetch current joints from ROS 2 on startup
def get_current_joints(node):
    class JointStatesSubscriber:
        def __init__(self):
            self.received = False
            self.positions = [0.0, 0.0, 0.0]
        def callback(self, msg):
            name_map = {n: i for i, n in enumerate(msg.name)}
            if 'joint1' in name_map and 'joint2' in name_map and 'joint3' in name_map:
                self.positions = [
                    msg.position[name_map['joint1']],
                    msg.position[name_map['joint2']],
                    msg.position[name_map['joint3']]
                ]
                self.received = True

    sub_obj = JointStatesSubscriber()
    sub = node.create_subscription(JointState, '/joint_states', sub_obj.callback, 10)
    
    # Spin up to 1 second
    start_time = time.time()
    while not sub_obj.received and (time.time() - start_time) < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        
    node.destroy_subscription(sub)
    return sub_obj.positions if sub_obj.received else [0.0, 0.0, 0.0]

# Initialize ROS 2 Node in session state
if 'ros_node' not in st.session_state:
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('streamlit_gui_node')
    st.session_state.ros_node = node
    st.session_state.joint_pub = node.create_publisher(
        JointTrajectory,
        '/joint_trajectory_controller/joint_trajectory',
        10
    )
    st.session_state.kinematics = Kinematics()

    # ROS 2 Native Decoupled AI Agent topics setup
    st.session_state.command_pub = node.create_publisher(
        String,
        '/agent/command',
        10
    )
    st.session_state.token_queue = queue.Queue()
    st.session_state.response_queue = queue.Queue()

    # Subscriptions
    st.session_state.token_sub = node.create_subscription(
        String,
        '/agent/tokens',
        lambda msg: st.session_state.token_queue.put(msg.data),
        10
    )
    st.session_state.response_sub = node.create_subscription(
        String,
        '/agent/response',
        (lambda msg: st.session_state.response_queue.put(msg.data)),
        10
    )

# Helper function to sync GUI session state values back to widget keys
def sync_widget_states():
    st.session_state.j1_slider = float(st.session_state.j1_val)
    st.session_state.j1_text = f"{st.session_state.j1_val:.3f}"
    st.session_state.j2_slider = float(st.session_state.j2_val)
    st.session_state.j2_text = f"{st.session_state.j2_val:.3f}"
    st.session_state.j3_slider = float(st.session_state.j3_val)
    st.session_state.j3_text = f"{st.session_state.j3_val:.3f}"
    
    st.session_state.ik_x_slider = float(st.session_state.ik_x_val)
    st.session_state.ik_x_text = f"{st.session_state.ik_x_val:.3f}"
    st.session_state.ik_y_slider = float(st.session_state.ik_y_val)
    st.session_state.ik_y_text = f"{st.session_state.ik_y_val:.3f}"
    st.session_state.ik_z_slider = float(st.session_state.ik_z_val)
    st.session_state.ik_z_text = f"{st.session_state.ik_z_val:.3f}"

# Initialize session state variables on startup
if 'j1_val' not in st.session_state:
    # 1. Fetch current joint states from hardware
    init_joints = [0.0, 0.0, 0.0]
    try:
        init_joints = get_current_joints(st.session_state.ros_node)
    except Exception:
        pass
    
    # Order: [joint1 (beta), joint2 (gama), joint3 (alpha)]
    st.session_state.j1_val = float(init_joints[2]) # alpha
    st.session_state.j2_val = float(init_joints[0]) # beta
    st.session_state.j3_val = float(init_joints[1]) # gama

    # Calculate default Cartesian position
    x, y, z = st.session_state.kinematics.get_dk(st.session_state.j2_val, st.session_state.j3_val, st.session_state.j1_val)
    st.session_state.ik_x_val = float(x)
    st.session_state.ik_y_val = float(y)
    st.session_state.ik_z_val = float(z)
    
    sync_widget_states()

# Sidebar layout - Rendered safely after initialization
st.sidebar.markdown("<h1 style='text-align: center; font-size: 20px; color: #00f2fe; text-shadow: 0 0 10px rgba(0, 242, 254, 0.4);'>SKETCH TERMINATOR</h1>", unsafe_allow_html=True)
st.sidebar.markdown("---")

# Joint monitoring in Sidebar to keep it useful
st.sidebar.markdown("### 🦾 System Status")
st.sidebar.markdown(f"- **J1 (Base):** `{st.session_state.j1_val:.3f} rad`")
st.sidebar.markdown(f"- **J2 (Shoulder):** `{st.session_state.j2_val:.3f} rad`")
st.sidebar.markdown(f"- **J3 (Elbow):** `{st.session_state.j3_val:.3f} rad`")

# Real-time Trajectory Publisher
def publish_joints(j1, j2, j3):
    try:
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3']
        point = JointTrajectoryPoint()
        point.positions = [float(j2), float(j3), float(j1)]
        point.time_from_start = Duration(sec=0, nanosec=150000000) # 150ms for smooth tracking
        msg.points.append(point)
        st.session_state.joint_pub.publish(msg)
    except Exception:
        pass

def on_joint_change():
    # Sync IK values
    x, y, z = st.session_state.kinematics.get_dk(st.session_state.j2_val, st.session_state.j3_val, st.session_state.j1_val)
    st.session_state.ik_x_val = float(x)
    st.session_state.ik_y_val = float(y)
    st.session_state.ik_z_val = float(z)
    # Publish to robot
    publish_joints(st.session_state.j1_val, st.session_state.j2_val, st.session_state.j3_val)

def on_ik_change():
    # Solve IK
    try:
        b_val, g_val, a_val = st.session_state.kinematics.get_ik(st.session_state.ik_x_val, st.session_state.ik_y_val, st.session_state.ik_z_val)
        st.session_state.j1_val = float(a_val)
        st.session_state.j2_val = float(b_val)
        st.session_state.j3_val = float(g_val)
        # Publish to robot
        publish_joints(a_val, b_val, g_val)
    except Exception:
        pass

# Bidirectional sync callbacks
def update_j1_slider():
    st.session_state.j1_val = st.session_state.j1_slider
    on_joint_change()
    sync_widget_states()

def update_j1_text():
    try:
        val = float(st.session_state.j1_text)
        st.session_state.j1_val = max(-3.14, min(3.14, val))
    except ValueError:
        pass
    on_joint_change()
    sync_widget_states()

def update_j2_slider():
    st.session_state.j2_val = st.session_state.j2_slider
    on_joint_change()
    sync_widget_states()

def update_j2_text():
    try:
        val = float(st.session_state.j2_text)
        st.session_state.j2_val = max(-3.14, min(3.14, val))
    except ValueError:
        pass
    on_joint_change()
    sync_widget_states()

def update_j3_slider():
    st.session_state.j3_val = st.session_state.j3_slider
    on_joint_change()
    sync_widget_states()

def update_j3_text():
    try:
        val = float(st.session_state.j3_text)
        st.session_state.j3_val = max(-3.14, min(3.14, val))
    except ValueError:
        pass
    on_joint_change()
    sync_widget_states()

def update_ik_x_slider():
    st.session_state.ik_x_val = st.session_state.ik_x_slider
    on_ik_change()
    sync_widget_states()

def update_ik_x_text():
    try:
        val = float(st.session_state.ik_x_text)
        st.session_state.ik_x_val = max(-0.45, min(0.45, val))
    except ValueError:
        pass
    on_ik_change()
    sync_widget_states()

def update_ik_y_slider():
    st.session_state.ik_y_val = st.session_state.ik_y_slider
    on_ik_change()
    sync_widget_states()

def update_ik_y_text():
    try:
        val = float(st.session_state.ik_y_text)
        st.session_state.ik_y_val = max(-0.45, min(0.45, val))
    except ValueError:
        pass
    on_ik_change()
    sync_widget_states()

def update_ik_z_slider():
    st.session_state.ik_z_val = st.session_state.ik_z_slider
    on_ik_change()
    sync_widget_states()

def update_ik_z_text():
    try:
        val = float(st.session_state.ik_z_text)
        st.session_state.ik_z_val = max(-0.05, min(0.09, val))
    except ValueError:
        pass
    on_ik_change()
    sync_widget_states()

# Header Section
st.markdown("<h1 style='text-align: center; margin-bottom: 30px;'>🤖 ROBOTIC CONTROL DASHBOARD</h1>", unsafe_allow_html=True)

# Initialize Chat History
if 'chat_history' not in st.session_state:
    st.session_state.chat_history = []

# Modern tabs swapper
tab_manual, tab_rosa = st.tabs(["🎛️ MANUAL CONTROL", "🤖 AUTONOMOUS ROSA"])

with tab_manual:
    col1, col2 = st.columns(2)

    with col1:
        with st.container(border=True):
            st.markdown("<h2>Direct Kinematics</h2>", unsafe_allow_html=True)
            st.markdown("<p style='color:#a0aec0;'>Drag sliders or enter text to command robot joints in real time.</p>", unsafe_allow_html=True)
            
            # Row 1: Joint 1
            st.markdown("#### Joint 1 (Base Rotation) [rad]")
            c_sld, c_txt = st.columns([3, 1])
            c_sld.slider("j1_sld_lbl", -3.14, 3.14, key="j1_slider", on_change=update_j1_slider, label_visibility="collapsed", step=0.01)
            c_txt.text_input("j1_txt_lbl", key="j1_text", on_change=update_j1_text, label_visibility="collapsed")

            # Row 2: Joint 2
            st.markdown("#### Joint 2 (Shoulder) [rad]")
            c_sld, c_txt = st.columns([3, 1])
            c_sld.slider("j2_sld_lbl", -3.14, 3.14, key="j2_slider", on_change=update_j2_slider, label_visibility="collapsed", step=0.01)
            c_txt.text_input("j2_txt_lbl", key="j2_text", on_change=update_j2_text, label_visibility="collapsed")

            # Row 3: Joint 3
            st.markdown("#### Joint 3 (Elbow) [rad]")
            c_sld, c_txt = st.columns([3, 1])
            c_sld.slider("j3_sld_lbl", -3.14, 3.14, key="j3_slider", on_change=update_j3_slider, label_visibility="collapsed", step=0.01)
            c_txt.text_input("j3_txt_lbl", key="j3_text", on_change=update_j3_text, label_visibility="collapsed")

            st.markdown("<h3 class='glow-text'>Computed Cartesian Pose:</h3>", unsafe_allow_html=True)
            # DK calculations
            x_dk, y_dk, z_dk = st.session_state.kinematics.get_dk(st.session_state.j2_val, st.session_state.j3_val, st.session_state.j1_val)
            st.code(f"X: {x_dk*1000.0:.1f} mm\n"
                    f"Y: {y_dk*1000.0:.1f} mm\n"
                    f"Z: {z_dk*1000.0:.1f} mm")

    with col2:
        with st.container(border=True):
            st.markdown("<h2>Inverse Kinematics</h2>", unsafe_allow_html=True)
            st.markdown("<p style='color:#a0aec0;'>Drag sliders or enter text to command Cartesian target in real time.</p>", unsafe_allow_html=True)

            # Row 1: X Position
            st.markdown("#### X Position [m]")
            c_sld, c_txt = st.columns([3, 1])
            c_sld.slider("ik_x_sld_lbl", -0.45, 0.45, key="ik_x_slider", on_change=update_ik_x_slider, label_visibility="collapsed", step=0.005)
            c_txt.text_input("ik_x_txt_lbl", key="ik_x_text", on_change=update_ik_x_text, label_visibility="collapsed")

            # Row 2: Y Position
            st.markdown("#### Y Position [m]")
            c_sld, c_txt = st.columns([3, 1])
            c_sld.slider("ik_y_sld_lbl", -0.45, 0.45, key="ik_y_slider", on_change=update_ik_y_slider, label_visibility="collapsed", step=0.005)
            c_txt.text_input("ik_y_txt_lbl", key="ik_y_text", on_change=update_ik_y_text, label_visibility="collapsed")

            # Row 3: Z Position
            st.markdown("#### Z Position [m]")
            c_sld, c_txt = st.columns([3, 1])
            c_sld.slider("ik_z_sld_lbl", -0.05, 0.09, key="ik_z_slider", on_change=update_ik_z_slider, label_visibility="collapsed", step=0.005)
            c_txt.text_input("ik_z_txt_lbl", key="ik_z_text", on_change=update_ik_z_text, label_visibility="collapsed")

            # Calculate IK for display status
            try:
                b_val, g_val, a_val = st.session_state.kinematics.get_ik(st.session_state.ik_x_val, st.session_state.ik_y_val, st.session_state.ik_z_val)
                ik_success = True
            except Exception:
                ik_success = False

            st.markdown("<h3 class='glow-text'>Computed Joint Values:</h3>", unsafe_allow_html=True)
            if ik_success:
                st.code(f"J1: {b_val:.4f} rad ({b_val * 180.0 / 3.1415:.1f}°)\n"
                        f"J2: {g_val:.4f} rad ({g_val * 180.0 / 3.1415:.1f}°)\n"
                        f"J3: {a_val:.4f} rad ({a_val * 180.0 / 3.1415:.1f}°)")
            else:
                st.error("IK Resolution Failed: Target is out of reach.")

with tab_rosa:
    with st.container(border=True):
        st.markdown("<h2>ROSA Autonomous AI Command Center</h2>", unsafe_allow_html=True)
        st.markdown("<p style='color:#a0aec0;'>Interact with the robotic system using natural language instructions.</p>", unsafe_allow_html=True)
        st.markdown("---")

        # Render Chat History in premium chat bubbles
        for msg in st.session_state.chat_history:
            with st.chat_message(msg["role"]):
                st.markdown(msg["content"])

        # Modern chat input box
        command = st.chat_input("Enter natural language request (e.g. 'Idi od car do traffic light i izbjegni cat')...")
        
        col_clear = st.columns([1, 6])[0]
        if col_clear.button("CLEAR CHAT"):
            st.session_state.chat_history = []
            st.rerun()

        if command:
            # Add user prompt to history and render it immediately
            st.session_state.chat_history.append({"role": "user", "content": command})
            with st.chat_message("user"):
                st.markdown(command)

            # Assistant generation block
            with st.chat_message("assistant"):
                status_container = st.empty()
                response_container = st.empty()
                
                # Publish command to ROS 2 topic
                cmd_msg = String()
                cmd_msg.data = command
                st.session_state.command_pub.publish(cmd_msg)
                
                # Clear queues
                st.session_state.token_queue = queue.Queue()
                st.session_state.response_queue = queue.Queue()

                full_text = ""
                completed = False
                start_time = time.time()
                timeout_limit = 60.0  # 60s max execution time

                # ChatGPT-like loading/thinking spinner wrapper
                with st.spinner("Razmišljam... (AI Agent is thinking)"):
                    status_container.info("🧠 Pokrećem ROSA proces razmišljanja...")
                    
                    while (time.time() - start_time) < timeout_limit:
                         # Spin once to fetch latest ROS 2 subscription messages
                         rclpy.spin_once(st.session_state.ros_node, timeout_sec=0.02)
                         
                         # Check for completed response
                         if not st.session_state.response_queue.empty():
                             st.session_state.response_queue.get()
                             completed = True
                             break

                         # Process all accumulated streaming tokens/events
                         while not st.session_state.token_queue.empty():
                             try:
                                 event_data_str = st.session_state.token_queue.get_nowait()
                                 event = json.loads(event_data_str)
                                 kind = event.get("type", "")
                                 
                                 if kind == "token":
                                     full_text += event.get("content", "")
                                     response_container.markdown(full_text)
                                     start_time = time.time()  # Keep-alive on new token receipt
                                 elif kind == "tool_start":
                                     tool_name = event.get("content", "tool")
                                     status_container.info(f"⚙️ Pokrećem robotski alat (tool): `{tool_name}`...")
                                     start_time = time.time()
                                 elif kind == "tool_end":
                                     tool_name = event.get("content", "tool")
                                     status_container.success(f"✅ Alat `{tool_name}` uspješno izvršen.")
                                     start_time = time.time()
                                 elif kind == "error":
                                     err_msg = event.get("content", "Unknown error")
                                     st.error(f"❌ ROSA Error: {err_msg}")
                                     completed = True
                                     break
                             except Exception:
                                 pass
                         
                         if completed:
                             break

                    if completed:
                        st.session_state.chat_history.append({"role": "assistant", "content": full_text})
                        status_container.empty() # Clean up status box
                    else:
                        st.warning("⚠️ Execution timed out or no agent_node is currently running in the background.")
