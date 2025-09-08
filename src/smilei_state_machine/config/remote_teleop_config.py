#!/usr/bin/env python3
"""
Configuración para teleoperación remota entre dos computadoras.

Este archivo permite configurar las direcciones IP y puertos para la comunicación UDP
entre dos máquinas ejecutando el sistema SMILEi.

Configuración de Red:
- LOCAL_IP: IP de esta máquina
- REMOTE_IP: IP de la máquina remota
- SEND_PORT: Puerto para enviar datos
- RECEIVE_PORT: Puerto para recibir datos

Configuración de Motores:
- MOTOR_IDS: Lista de IDs de motores a controlar
- LEADER_MOTOR_COUNT: Número de motores líderes (primera mitad de la lista)

Uso:
1. Copie este archivo a ambas máquinas
2. Modifique las IPs según su red
3. Asegúrese de que los puertos estén abiertos en el firewall
4. Execute el estado 'remote_teleoperation' en ambas máquinas
"""

# ================================
# CONFIGURACIÓN DE RED
# ================================

# Máquina A (ejemplo: Computadora principal)
LOCAL_IP_A = '192.168.4.241'
REMOTE_IP_A = '192.168.4.238'  # Cambiar XXX por la IP de la máquina B

# Máquina B (ejemplo: Computadora secundaria)  
LOCAL_IP_B = '192.168.4.238'   # IP real de la máquina B
REMOTE_IP_B = '192.168.4.241'

# Puertos UDP (deben ser diferentes para evitar conflictos)
SEND_PORT = 5005
RECEIVE_PORT = 4000

# ================================
# CONFIGURACIÓN ACTUAL
# ================================
# Cambie estos valores según la máquina actual

# Para Máquina A (192.168.4.241):
LOCAL_IP = '192.168.4.241'   # Esta máquina
REMOTE_IP = '192.168.4.238'  # Máquina remota

# Para Máquina B (descomente las siguientes líneas):
# LOCAL_IP = LOCAL_IP_B
# REMOTE_IP = REMOTE_IP_B

# ================================
# CONFIGURACIÓN DE MOTORES
# ================================

# IDs de motores a controlar (máximo 8)
# MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8]  # Configuración completa

# Para prueba con un motor por máquina:
MOTOR_IDS = [1]  # Solo motor ID=1

# ================================
# CONFIGURACIÓN DE CONTROL
# ================================

# Ganancias de control (PD)
CONTROL_GAINS = {
    'kp': [1.75] * 8,  # Ganancia proporcional para cada motor
    'kd': [0.1] * 8,   # Ganancia derivativa para cada motor
}

# Límites de seguridad para cada motor (min, max) en radianes
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

# ================================
# CONFIGURACIÓN DE COMUNICACIÓN
# ================================

# Timeout para sockets UDP (segundos)
SOCKET_TIMEOUT = 0.001

# Máximo número de errores de comunicación antes de terminar
MAX_COMMUNICATION_ERRORS = 10

# Frecuencia de control (Hz)
CONTROL_FREQUENCY = 1000  # 1 kHz como en el código original

# ================================
# FUNCIONES DE CONFIGURACIÓN
# ================================

def get_network_config():
    """Retorna la configuración de red actual"""
    return {
        'local_ip': LOCAL_IP,
        'remote_ip': REMOTE_IP,
        'send_port': SEND_PORT,
        'receive_port': RECEIVE_PORT,
        'socket_timeout': SOCKET_TIMEOUT
    }

def get_motor_config():
    """Retorna la configuración de motores"""
    return {
        'motor_ids': MOTOR_IDS,
        'control_gains': CONTROL_GAINS,
        'position_limits': POSITION_LIMITS
    }

def get_communication_config():
    """Retorna la configuración de comunicación"""
    return {
        'max_errors': MAX_COMMUNICATION_ERRORS,
        'control_frequency': CONTROL_FREQUENCY
    }

def validate_config():
    """Valida la configuración"""
    errors = []
    
    # Validar IPs
    if LOCAL_IP == REMOTE_IP:
        errors.append("LOCAL_IP y REMOTE_IP no pueden ser iguales")
    
    # Validar motores
    if len(MOTOR_IDS) == 0:
        errors.append("MOTOR_IDS no puede estar vacío")
    
    if len(MOTOR_IDS) > 8:
        errors.append("Máximo 8 motores soportados")
    
    # Validar ganancias
    if len(CONTROL_GAINS['kp']) != len(MOTOR_IDS):
        errors.append("Número de ganancias kp debe coincidir con número de motores")
    
    if len(CONTROL_GAINS['kd']) != len(MOTOR_IDS):
        errors.append("Número de ganancias kd debe coincidir con número de motores")
    
    # Validar límites
    if len(POSITION_LIMITS) < len(MOTOR_IDS):
        errors.append("Debe haber límites de posición para cada motor")
    
    return errors

def print_config_summary():
    """Imprime un resumen de la configuración"""
    print("=== CONFIGURACIÓN DE TELEOPERACIÓN REMOTA ===")
    print(f"IP Local: {LOCAL_IP}")
    print(f"IP Remota: {REMOTE_IP}")
    print(f"Puerto Envío: {SEND_PORT}")
    print(f"Puerto Recepción: {RECEIVE_PORT}")
    print(f"Motores: {MOTOR_IDS}")
    print(f"Frecuencia Control: {CONTROL_FREQUENCY} Hz")
    
    errors = validate_config()
    if errors:
        print("\n⚠️  ERRORES DE CONFIGURACIÓN:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("\n✅ Configuración válida")
    print("=" * 45)

# ================================
# EJEMPLO DE USO
# ================================

if __name__ == "__main__":
    print_config_summary()
    
    # Mostrar configuración para ambas máquinas
    print("\n=== CONFIGURACIÓN PARA MÁQUINA A ===")
    print(f"LOCAL_IP = '{LOCAL_IP_A}'")
    print(f"REMOTE_IP = '{REMOTE_IP_A}'")
    
    print("\n=== CONFIGURACIÓN PARA MÁQUINA B ===")
    print(f"LOCAL_IP = '{LOCAL_IP_B}'")
    print(f"REMOTE_IP = '{REMOTE_IP_B}'")
    
    print("\n📝 INSTRUCCIONES:")
    print("1. Copie este archivo a ambas máquinas")
    print("2. En cada máquina, modifique LOCAL_IP y REMOTE_IP según corresponda")
    print("3. Asegúrese de que los puertos estén abiertos en el firewall")
    print("4. Execute el estado 'remote_teleoperation' en ambas máquinas")
    print("5. Un motor en cada máquina replicará el movimiento del otro")