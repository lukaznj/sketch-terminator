cmake_minimum_required(VERSION 3.8)
project(sketch_terminator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(dynamixel_hardware REQUIRED)

# Python nodes. These are installed as plain executables rather than as a
# Python package, so they import their siblings by bare module name
# (`from kinematics import Kinematics`) -- everything lands in one directory.
install(PROGRAMS
  sketch_terminator/agent.py
  sketch_terminator/agent_node.py
  sketch_terminator/generate_smooth_path.py
  sketch_terminator/kinematics.py
  sketch_terminator/kinematics_node.py
  sketch_terminator/move_robot.py
  sketch_terminator/path_planner_node.py
  sketch_terminator/ros2_introspection.py
  sketch_terminator/tools.py
  sketch_terminator/visibility_graph.py
  sketch_terminator/yolo_node.py
  sketch_terminator/yolo_workspace_processing_node.py
  scripts/camera_to_world.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files, the robot model, configuration, meshes and the GUI.
install(
  DIRECTORY config controllers gui launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
