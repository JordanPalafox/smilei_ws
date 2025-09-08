#!/usr/bin/env python3
"""
ARCHIVO PARA LA OTRA MÁQUINA (192.168.4.238)

Copie TODO este contenido a:
src/smilei_state_machine/config/remote_teleop_config.py

Luego ejecute:
colcon build --packages-select smilei_state_machine
source install/setup.bash
ros2 run smilei_state_machine send_state_command remote_teleoperation
"""

# ================================
# CONFIGURACIÓN SIMPLE
# ================================

# ¿Esta es la máquina A? (True/False)
IS_MACHINE_A = False  # <<<< CAMBIO PRINCIPAL: False para máquina B

# IPs de las máquinas
MACHINE_A_IP = '192.168.4.241'  # Tu máquina original
MACHINE_B_IP = '192.168.4.238'  # Esta máquina

# IDs de motores (lista de enteros)
MOTOR_IDS = [1]

# ================================
# CONFIGURACIÓN AUTOMÁTICA
# ================================
# No cambies esto - se configura automáticamente

if IS_MACHINE_A:
    # Configuración para Máquina A
    LOCAL_IP = MACHINE_A_IP
    REMOTE_IP = MACHINE_B_IP  
    SEND_PORT = 4000      # A envía al puerto 4000 de B
    RECEIVE_PORT = 5001   # A recibe en puerto 5001
else:
    # Configuración para Máquina B
    LOCAL_IP = MACHINE_B_IP
    REMOTE_IP = MACHINE_A_IP
    SEND_PORT = 5001      # B envía al puerto 5001 de A  
    RECEIVE_PORT = 4000   # B recibe en puerto 4000

# ================================
# CONFIGURACIÓN DE CONTROL
# ================================
CONTROL_GAINS = {
    'kp': [1.75] * 8,
    'kd': [0.1] * 8,
}

POSITION_LIMITS = [
    (-1.58, 1.58),    # Motor 1
    (-0.79, 1.58),    # Motor 2  
    (-3.1416, 1.58),  # Motor 3
    (-0.18, 1.16),    # Motor 4
    (-1.58, 1.58),    # Motor 5
    (-1.58, 0.79),    # Motor 6
    (-1.58, 3.1416),  # Motor 7
    (-1.16, 0.18)     # Motor 8
]

SOCKET_TIMEOUT = 0.001
MAX_COMMUNICATION_ERRORS = 10
CONTROL_FREQUENCY = 1000

# ================================
# FUNCIONES PARA LA MÁQUINA DE ESTADOS
# ================================
def get_network_config():
    return {
        'local_ip': LOCAL_IP,
        'remote_ip': REMOTE_IP,
        'send_port': SEND_PORT,
        'receive_port': RECEIVE_PORT,
        'socket_timeout': SOCKET_TIMEOUT
    }

def get_motor_config():
    return {
        'motor_ids': MOTOR_IDS,
        'control_gains': CONTROL_GAINS,
        'position_limits': POSITION_LIMITS
    }

def get_communication_config():
    return {
        'max_errors': MAX_COMMUNICATION_ERRORS,
        'control_frequency': CONTROL_FREQUENCY
    }

# ================================
# INFORMACIÓN DE DEBUG
# ================================
if __name__ == "__main__":
    machine_name = "A" if IS_MACHINE_A else "B"
    print(f"=== CONFIGURACIÓN MÁQUINA {machine_name} ===")
    print(f"Local IP: {LOCAL_IP}")
    print(f"Remote IP: {REMOTE_IP}")
    print(f"Send Port: {SEND_PORT}")
    print(f"Receive Port: {RECEIVE_PORT}")
    print(f"Motor IDs: {MOTOR_IDS}")
    print("=" * 40)