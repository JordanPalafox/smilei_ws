"""
Paquete Python para la interfaz con westwood_motor_control_sdk
"""

# Importar componentes principales para facilitar su uso
from westwood_motor_control_sdk_wrapper.Manager import BEAR as Manager
from westwood_motor_control_sdk_wrapper.CONTROL_TABLE import *
from westwood_motor_control_sdk_wrapper.CRC import CRC
from westwood_motor_control_sdk_wrapper.TIMING_TABLE import *

# Crear alias para pybear para compatibilidad
import westwood_motor_control_sdk_wrapper.Manager as pybear
