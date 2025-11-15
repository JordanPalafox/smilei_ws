cmake_minimum_required(VERSION 3.8)
project(smilei_dual_arm_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Generate actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteGesture.action"
)

# Set Python install directory
set(PYTHON_INSTALL_DIR "local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages")

# Install Python package manually to site-packages
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables separately (with .py extension)
install(PROGRAMS
  smilei_dual_arm_ik/forward_kinematics_urdf_based.py
  smilei_dual_arm_ik/forward_kinematics_dual_arm.py
  smilei_dual_arm_ik/trajectory_executor_node.py
  smilei_dual_arm_ik/trajectory_executor_dual_arm.py
  smilei_dual_arm_ik/gesture_executor_dual_arm.py
  smilei_dual_arm_ik/interactive_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files recursively (including gestures subdirectory)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF files
install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

# Install RViz files
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

# Configure PYTHONPATH hook
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/pythonpath.sh"
  "export PYTHONPATH=\"\${AMENT_CURRENT_PREFIX}/${PYTHON_INSTALL_DIR}:\$PYTHONPATH\"\n"
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/pythonpath.sh"
  DESTINATION share/${PROJECT_NAME}/environment
)

ament_package()
