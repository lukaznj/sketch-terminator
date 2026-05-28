import os
import asyncio
import streamlit as st
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from sketch_terminator.kinematics import Kinematics
from sketch_terminator.agent import create_agent

# Set page config
st.set_page_config(
    page_title="Antigravity ROS 2 Control Panel",
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

    h1, h2, h3 {
        font-family: 'Orbitron', sans-serif !important;
        letter-spacing: 2px;
        color: #00f2fe;
        text-shadow: 0 0 10px rgba(0, 242, 254, 0.4);
    }
    
    /* Premium Glassmorphic Cards */
    .glass-card {
        background: rgba(21, 26, 48, 0.45);
        backdrop-filter: blur(12px);
        -webkit-backdrop-filter: blur(12px);
        border: 1px solid rgba(255, 255, 255, 0.08);
        border-radius: 16px;
        padding: 24px;
        margin-bottom: 24px;
        box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.37);
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

# Initialize ROSA Agent in session state
if 'agent' not in st.session_state:
    try:
        st.session_state.agent = create_agent(streaming=True, verbose=False)
        st.session_state.agent_ready = True
    except Exception as e:
        st.session_state.agent_ready = False
        st.session_state.agent_error = str(e)

# Sidebar layout
st.sidebar.markdown("<h1 style='text-align: center;'>ANTIGRAVITY</h1>", unsafe_allow_html=True)
st.sidebar.markdown("<p style='text-align: center; color: #4facfe;'>Premium Manipulator Control</p>", unsafe_allow_html=True)
st.sidebar.markdown("---")

mode = st.sidebar.radio(
    "SELECT SYSTEM MODE:",
    ["Manual Kinematics Mode", "Autonomous ROSA Mode"]
)

# Header Section
st.markdown("<h1 style='text-align: center; margin-bottom: 30px;'>🤖 ROBOTIC CONTROL DASHBOARD</h1>", unsafe_allow_html=True)

if mode == "Manual Kinematics Mode":
    col1, col2 = st.columns(2)

    with col1:
        st.markdown("<div class='glass-card'>", unsafe_allow_html=True)
        st.markdown("<h2>⚡ Direct Kinematics</h2>", unsafe_allow_html=True)
        st.markdown("<p style='color:#a0aec0;'>Enter joint angles to compute Cartesian pose.</p>", unsafe_allow_html=True)

        dk_input_type = st.radio("DK Control Type:", ["Sliders", "Text Inputs"], key="dk_type")
        
        if dk_input_type == "Sliders":
            j1 = st.slider("Joint 1 (Base Rotation) [rad]:", -3.14, 3.14, 0.0, step=0.01)
            j2 = st.slider("Joint 2 (Shoulder) [rad]:", -3.14, 3.14, 0.0, step=0.01)
            j3 = st.slider("Joint 3 (Elbow) [rad]:", -3.14, 3.14, 0.0, step=0.01)
        else:
            j1 = float(st.text_input("Joint 1 (Base Rotation) [rad]:", "0.0", key="dk_j1"))
            j2 = float(st.text_input("Joint 2 (Shoulder) [rad]:", "0.0", key="dk_j2"))
            j3 = float(st.text_input("Joint 3 (Elbow) [rad]:", "0.0", key="dk_j3"))

        # Calculate pose
        x, y, z = st.session_state.kinematics.get_dk(j2, j3, j1) # DK maps beta=j2, gama=j3, alpha=j1

        st.markdown("<h3 class='glow-text'>Computed Cartesian Pose:</h3>", unsafe_allow_html=True)
        st.code(f"X (End Effector) = {x:.4f} m ({x*1000.0:.1f} mm)\nY (End Effector) = {y:.4f} m ({y*1000.0:.1f} mm)\nZ (End Effector) = {z:.4f} m ({z*1000.0:.1f} mm)")

        if st.button("Move to Joint Position"):
            msg = JointTrajectory()
            msg.joint_names = ['joint1', 'joint2', 'joint3']
            point = JointTrajectoryPoint()
            point.positions = [float(j2), float(j3), float(j1)]  # Joint1=beta, Joint2=gama, Joint3=alpha
            point.time_from_start = Duration(sec=2, nanosec=0)
            msg.points.append(point)
            
            st.session_state.joint_pub.publish(msg)
            st.success(f"Commanded Joint Angles: {point.positions}")
        
        st.markdown("</div>", unsafe_allow_html=True)

    with col2:
        st.markdown("<div class='glass-card'>", unsafe_allow_html=True)
        st.markdown("<h2>📐 Inverse Kinematics</h2>", unsafe_allow_html=True)
        st.markdown("<p style='color:#a0aec0;'>Enter Cartesian coordinates to compute joint angles.</p>", unsafe_allow_html=True)

        ik_input_type = st.radio("IK Control Type:", ["Sliders", "Text Inputs"], key="ik_type")

        if ik_input_type == "Sliders":
            X_val = st.slider("X Position [m]:", -0.45, 0.45, 0.2, step=0.005)
            Y_val = st.slider("Y Position [m]:", -0.45, 0.45, 0.0, step=0.005)
            Z_val = st.slider("Z Position [m]:", -0.05, 0.09, 0.0, step=0.005)
        else:
            X_val = float(st.text_input("X Position [m]:", "0.2", key="ik_x"))
            Y_val = float(st.text_input("Y Position [m]:", "0.0", key="ik_y"))
            Z_val = float(st.text_input("Z Position [m]:", "0.0", key="ik_z"))

        # Calculate IK
        try:
            b_val, g_val, a_val = st.session_state.kinematics.get_ik(X_val, Y_val, Z_val)
            ik_success = True
        except Exception as e:
            ik_success = False
            ik_err = str(e)

        st.markdown("<h3 class='glow-text'>Computed Joint Values:</h3>", unsafe_allow_html=True)
        if ik_success:
            st.code(f"Joint 1 (Shoulder - beta) = {b_val:.4f} rad ({b_val * 180.0 / 3.1415:.1f}°)\n"
                    f"Joint 2 (Elbow - gama)    = {g_val:.4f} rad ({g_val * 180.0 / 3.1415:.1f}°)\n"
                    f"Joint 3 (Base - alpha)    = {a_val:.4f} rad ({a_val * 180.0 / 3.1415:.1f}°)")
            
            if st.button("Move to Cartesian Target"):
                msg = JointTrajectory()
                msg.joint_names = ['joint1', 'joint2', 'joint3']
                point = JointTrajectoryPoint()
                point.positions = [float(b_val), float(g_val), float(a_val)]
                point.time_from_start = Duration(sec=2, nanosec=0)
                msg.points.append(point)
                
                st.session_state.joint_pub.publish(msg)
                st.success(f"Commanded joints for pose [{X_val}, {Y_val}, {Z_val}]: {point.positions}")
        else:
            st.error(f"IK Resolution Failed: Target is out of reach.")
        
        st.markdown("</div>", unsafe_allow_html=True)

else:
    st.markdown("<div class='glass-card'>", unsafe_allow_html=True)
    st.markdown("<h2>⚡ ROSA Autonomous AI Command Center</h2>", unsafe_allow_html=True)
    st.markdown("<p style='color:#a0aec0;'>Interact with the robotic system using natural language instructions.</p>", unsafe_allow_html=True)
    st.markdown("---")

    if not st.session_state.get('agent_ready', False):
        st.error(f"ROSA Agent is unavailable. Please check that docs/.env has a valid OpenAI key. Error: {st.session_state.get('agent_error')}")
    else:
        # Chat interface
        command = st.text_input("Enter natural language request (e.g. 'Idi od car do traffic light i izbjegni cat'):")
        
        col_exec, col_clear = st.columns([1, 6])
        execute_clicked = col_exec.button("EXECUTE")
        clear_clicked = col_clear.button("CLEAR CHAT")

        if clear_clicked:
            st.session_state.agent.clear_chat()
            st.success("Chat history cleared.")

        if execute_clicked and command:
            st.write("---")
            st.markdown("<h3 class='glow-text'>Execution Stream:</h3>", unsafe_allow_html=True)
            
            # Setup containers
            tool_status = st.empty()
            token_stream = st.empty()

            async def stream_response(query):
                full_text = ""
                async for event in st.session_state.agent.astream(query):
                    kind = event.get("type", "")
                    if kind == "token":
                        full_text += event.get("content", "")
                        token_stream.markdown(f"<div style='background-color:#111528; padding:15px; border-radius:8px; border:1px solid #2d3748;'>{full_text}</div>", unsafe_allow_html=True)
                    elif kind == "tool_start":
                        tool_name = event.get("name", "tool")
                        tool_status.info(f"🛠️ Executing robotic system tool: `{tool_name}`...")
                    elif kind == "tool_end":
                        tool_name = event.get("name", "tool")
                        tool_status.success(f"✅ Completed executing tool: `{tool_name}` successfully.")

            # Run async stream
            asyncio.run(stream_response(command))

    st.markdown("</div>", unsafe_allow_html=True)
