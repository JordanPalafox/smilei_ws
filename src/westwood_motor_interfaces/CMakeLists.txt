cmake_minimum_required(VERSION 3.8)
project(westwood_motor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)


set(srv_files
  "srv/SetMotorIdAndTarget.srv"
  "srv/GetMotorPositions.srv"
  "srv/GetAvailableMotors.srv"
  "srv/SetGains.srv"
)


# Generamos los mensajes, servicios y acciones
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package() 