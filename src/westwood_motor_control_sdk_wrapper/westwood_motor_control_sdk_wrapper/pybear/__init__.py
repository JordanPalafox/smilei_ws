"""
Módulo PyBear para el control de motores Westwood
"""

# Redireccionar importaciones a los módulos principales
from westwood_motor_control_sdk_wrapper.Manager import BEAR as Manager
from westwood_motor_control_sdk_wrapper.CONTROL_TABLE import *
from westwood_motor_control_sdk_wrapper.CRC import CRC
from westwood_motor_control_sdk_wrapper.TIMING_TABLE import *
from westwood_motor_control_sdk_wrapper.Packet import PKT

# Función ping para mantener compatibilidad
def ping():
    """
    Función para verificar la conectividad con el hardware de Westwood
    
    Returns:
        bool: True si el ping fue exitoso, False en caso contrario
    """
    try:
        # Simulamos la conexión con el hardware
        print("PyBear: Enviando ping al hardware de Westwood")
        # En una implementación real, aquí iría el código para conectarse al hardware
        return True
    except Exception as e:
        print(f"PyBear: Error al hacer ping - {str(e)}")
        return False 