cmake_minimum_required(VERSION 3.8)
project(westwood_motor_control_sdk_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

# Determinar la versión de Python
execute_process(
  COMMAND python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
  OUTPUT_VARIABLE PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Asegurar que la estructura del paquete Python está completa
execute_process(
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/python_modules/westwood_motor_control_sdk_wrapper/pybear
)

# Copiar archivos Python al directorio de construcción
file(GLOB PYTHON_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/westwood_motor_control_sdk_wrapper/*.py
)
file(GLOB PYBEAR_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/westwood_motor_control_sdk_wrapper/pybear/*.py
)

foreach(PYTHON_FILE ${PYTHON_FILES})
  get_filename_component(FILENAME ${PYTHON_FILE} NAME)
  configure_file(
    ${PYTHON_FILE}
    ${CMAKE_CURRENT_BINARY_DIR}/python_modules/westwood_motor_control_sdk_wrapper/${FILENAME}
    COPYONLY
  )
endforeach()

foreach(PYBEAR_FILE ${PYBEAR_FILES})
  get_filename_component(FILENAME ${PYBEAR_FILE} NAME)
  configure_file(
    ${PYBEAR_FILE}
    ${CMAKE_CURRENT_BINARY_DIR}/python_modules/westwood_motor_control_sdk_wrapper/pybear/${FILENAME}
    COPYONLY
  )
endforeach()

# Install Python modules directly to sitedir
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python_modules/westwood_motor_control_sdk_wrapper/
  DESTINATION lib/python${PYTHON_VERSION}/site-packages/westwood_motor_control_sdk_wrapper
)

# Install Python scripts
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 