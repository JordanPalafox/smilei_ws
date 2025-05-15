#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import SetMotorIdAndTarget, GetMotorPositions, GetAvailableMotors
from westwood_motor_interfaces.srv import SetGains
import sys
import os

# Verificar si el módulo está disponible en el sistema
try:
    import westwood_motor_control_sdk_wrapper
except ImportError:
    # Si no está disponible, intentar importarlo desde la ruta relativa al script
    package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if package_path not in sys.path:
        sys.path.insert(0, package_path)
    
    # Intenta importar de nuevo
    try:
        import westwood_motor_control_sdk_wrapper
        from westwood_motor_control_sdk_wrapper import Manager
    except ImportError as e:
        print(f"Error al importar el módulo: {e}")
        print(f"Rutas de Python: {sys.path}")
        raise
else:
    # Si está disponible, importa los componentes
    from westwood_motor_control_sdk_wrapper import Manager

class WestwoodMotorServer(Node):
    def __init__(self):
        super().__init__('westwood_motor_server')
        self.get_logger().info('Westwood Motor Server started')
        
        # Parámetros configurables - usando específicamente el puerto pybear_UB00E9
        self.port = '/dev/pybear_UB00E9'  # Puerto fijo para asegurar que se use este dispositivo
        self.declare_parameter('baudrate', 8000000)
        self.declare_parameter('motor_ids', [1])
        self.declare_parameter('debug', False)
        
        # Obtener parámetros
        self.baudrate = self.get_parameter('baudrate').value
        self.motor_ids = self.get_parameter('motor_ids').value
        self.debug = self.get_parameter('debug').value
        
        # Inicializar el Manager de PyBear
        try:
            self.get_logger().info(f'Intentando conectar al puerto: {self.port}')
            self.manager = Manager(
                port=self.port,
                baudrate=self.baudrate,
                debug=self.debug
            )
            self.get_logger().info('Manager de PyBear inicializado correctamente')
            self.get_logger().info(f'Puerto: {self.port}, Baudrate: {self.baudrate}')
        except Exception as e:
            self.get_logger().error(f'Error al inicializar Manager de PyBear: {str(e)}')
            self.manager = None
        
        # Añadir el servicio para manejar múltiples motores con posiciones
        try:
            self.set_motor_id_and_target_service = self.create_service(
                SetMotorIdAndTarget,
                'westwood_motor/set_motor_id_and_target',
                self.handle_motor_ids_and_target
            )
            self.get_logger().info('Servicio de control multi-motor con arrays registrado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio multi-motor con arrays: {str(e)}')
        
        # Añadir servicio para obtener posiciones actuales de motores
        try:
            self.get_motor_positions_service = self.create_service(
                GetMotorPositions,
                'westwood_motor/get_motor_positions',
                self.handle_get_motor_positions
            )
            self.get_logger().info('Servicio para obtener posiciones de motores registrado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio de obtención de posiciones: {str(e)}')
        
        # Añadir servicio para obtener IDs de motores disponibles
        try:
            self.get_available_motors_service = self.create_service(
                GetAvailableMotors,
                'westwood_motor/get_available_motors',
                self.handle_get_available_motors
            )
            self.get_logger().info('Servicio para obtener IDs de motores disponibles registrado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio de obtención de motores disponibles: {str(e)}')
        
        # Añadir servicio para configurar ganancias de control de posición
        try:
            self.set_position_gains_service = self.create_service(
                SetGains,
                'westwood_motor/set_position_gains',
                self.handle_set_position_gains
            )
            self.get_logger().info('Servicio para configurar ganancias de posición registrado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio de configuración de ganancias de posición: {str(e)}')
        
        # Añadir servicio para configurar ganancias de control de corriente
        try:
            self.set_current_gains_service = self.create_service(
                SetGains,
                'westwood_motor/set_current_gains',
                self.handle_set_current_gains
            )
            self.get_logger().info('Servicio para configurar ganancias de corriente registrado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio de configuración de ganancias de corriente: {str(e)}')
        
    # Función principal para manejar IDs de motores y sus posiciones objetivo
    def handle_motor_ids_and_target(self, request, response):
        """Callback para controlar múltiples motores con posiciones objetivo individuales"""
        try:
            # Si no hay motores especificados, no hay nada que hacer
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            # Si la cantidad de motores no coincide con la cantidad de posiciones
            if len(request.motor_ids) != len(request.target_positions):
                response.success = False
                response.message = "La cantidad de IDs de motores no coincide con la cantidad de posiciones objetivo"
                return response
            
            connected_motors = []
            failed_motor_ids = []
            previous_positions = []
            
            # Si el manager está disponible, intentamos mover los motores reales
            if self.manager is not None:
                # Verificar conexión de cada motor y preparar lista de motores conectados
                for motor_id in request.motor_ids:
                    try:
                        ping_result = self.manager.ping(motor_id)
                        if ping_result:
                            connected_motors.append(motor_id)
                            self.get_logger().info(f'Motor {motor_id} conectado y listo para control')
                        else:
                            failed_motor_ids.append(motor_id)
                            self.get_logger().warning(f'Motor {motor_id} no responde')
                    except Exception as e:
                        self.get_logger().error(f'Error al hacer ping al motor {motor_id}: {str(e)}')
                        failed_motor_ids.append(motor_id)
                
                # Para cada motor conectado, configurar y mover
                for i, motor_id in enumerate(connected_motors):
                    if motor_id not in request.motor_ids:
                        continue
                    
                    # Obtener el índice del motor en la lista original
                    idx = request.motor_ids.index(motor_id)
                    
                    try:
                        # Leer posición actual y guardarla
                        current_position_result = self.manager.get_present_position(motor_id)
                        if current_position_result and len(current_position_result) > 0:
                            current_position = float(current_position_result[0][0][0])
                            previous_positions.append(current_position)
                            
                            # Configurar PID para el control de posición
                            # PID id/iq
                            self.manager.set_p_gain_iq((motor_id, 0.02))
                            self.manager.set_i_gain_iq((motor_id, 0.02))
                            self.manager.set_d_gain_iq((motor_id, 0))
                            self.manager.set_p_gain_id((motor_id, 0.02))
                            self.manager.set_i_gain_id((motor_id, 0.02))
                            self.manager.set_d_gain_id((motor_id, 0))
                            
                            # PID position mode
                            p_gain = 5.0
                            d_gain = 0.2
                            i_gain = 0.0
                            self.manager.set_p_gain_position((motor_id, p_gain))
                            self.manager.set_i_gain_position((motor_id, i_gain))
                            self.manager.set_d_gain_position((motor_id, d_gain))
                            
                            # Poner en modo posición
                            self.manager.set_mode((motor_id, 2))
                            
                            # Establecer límite de corriente
                            iq_max = 1.5
                            self.manager.set_limit_iq_max((motor_id, iq_max))
                            
                            # Establecer posición inicial antes de habilitar torque
                            self.manager.set_goal_position((motor_id, current_position))
                            
                            # Habilitar torque
                            self.manager.set_torque_enable((motor_id, 1))
                            
                            # Mover a la posición objetivo
                            target_position = request.target_positions[idx]
                            self.manager.set_goal_position((motor_id, target_position))
                            self.get_logger().info(f'Motor {motor_id} movido a {target_position} radianes')
                        else:
                            previous_positions.append(0.0)
                            failed_motor_ids.append(motor_id)
                            self.get_logger().warning(f'No se pudo leer la posición actual del motor {motor_id}')
                    except Exception as e:
                        self.get_logger().error(f'Error al configurar/mover motor {motor_id}: {str(e)}')
                        if motor_id in connected_motors:
                            connected_motors.remove(motor_id)
                        failed_motor_ids.append(motor_id)
                        previous_positions.append(0.0)
            
            # Si no hay motores conectados o hay errores, simulamos la respuesta
            if not connected_motors:
                for i, motor_id in enumerate(request.motor_ids):
                    # Simulamos que la posición anterior era 0.0
                    if len(previous_positions) <= i:
                        previous_positions.append(0.0)
                    self.get_logger().info(f'Simulando movimiento del motor {motor_id} a {request.target_positions[i]} radianes')
            
            # Asegurar que previous_positions tiene la misma longitud que request.motor_ids
            while len(previous_positions) < len(request.motor_ids):
                previous_positions.append(0.0)
            
            # Preparar respuesta
            response.success = True
            if connected_motors:
                response.message = f"Control de {len(connected_motors)} motores"
                if failed_motor_ids:
                    response.message += f" ({len(failed_motor_ids)} fallaron)"
            else:
                response.message = "Modo simulación: movimientos registrados pero no ejecutados"
            
            response.previous_positions = previous_positions
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio multi-motor: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            # Aún en caso de error, simulamos una respuesta
            response.success = True
            response.message = f"Error: {str(e)}. Modo simulación activado"
            response.previous_positions = [0.0] * len(request.motor_ids)
            return response

    # Función para obtener las posiciones actuales de los motores
    def handle_get_motor_positions(self, request, response):
        """Callback para obtener las posiciones actuales de los motores especificados"""
        try:
            # Si no hay motores especificados, utilizar la lista configurada
            motor_ids = request.motor_ids
            if not motor_ids or len(motor_ids) == 0:
                motor_ids = self.motor_ids
                
            positions = []
            connected_motors = []
            failed_motor_ids = []
            
            # Intenta obtener posiciones reales si el manager está disponible
            if self.manager is not None:
                for motor_id in motor_ids:
                    try:
                        ping_result = self.manager.ping(motor_id)
                        if ping_result:
                            connected_motors.append(motor_id)
                            # Obtener posición actual del motor
                            position_result = self.manager.get_present_position(motor_id)
                            if position_result and len(position_result) > 0:
                                current_position = float(position_result[0][0][0])
                                positions.append(current_position)
                                self.get_logger().info(f'Motor {motor_id}: posición actual {current_position}')
                            else:
                                positions.append(0.0)  # Valor por defecto si no se pudo leer
                                failed_motor_ids.append(motor_id)
                                self.get_logger().warning(f'No se pudo leer la posición del motor {motor_id}')
                        else:
                            failed_motor_ids.append(motor_id)
                            positions.append(0.0)  # Valor por defecto para motores desconectados
                            self.get_logger().warning(f'Motor {motor_id} no responde')
                    except Exception as e:
                        self.get_logger().error(f'Error al leer posición del motor {motor_id}: {str(e)}')
                        failed_motor_ids.append(motor_id)
                        positions.append(0.0)
            
            # Si no pudimos obtener posiciones reales, enviamos valores simulados
            if not positions:
                for motor_id in motor_ids:
                    positions.append(0.0)  # Posición inicial simulada en 0
                    self.get_logger().info(f'Usando posición simulada 0.0 para motor {motor_id}')
            
            # Asegurarse de que tenemos una posición para cada motor solicitado
            while len(positions) < len(motor_ids):
                positions.append(0.0)
            
            # Preparar respuesta
            response.success = True
            response.message = "Lectura de posiciones (reales o simuladas)"
            response.positions = positions
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error en servicio de lectura de posiciones: {str(e)}')
            # Aún en caso de error, devolvemos valores por defecto
            positions = [0.0] * len(request.motor_ids) if request.motor_ids else [0.0]
            response.success = True
            response.message = f"Error: {str(e)}. Usando posiciones simuladas 0.0"
            response.positions = positions
            return response

    # Función para obtener las IDs de los motores disponibles
    def handle_get_available_motors(self, request, response):
        """Callback para obtener las IDs de los motores disponibles actualmente"""
        try:
            available_motors = []
            
            # Verificar si manager está inicializado
            if self.manager is None:
                self.get_logger().error('Error: Manager es None, no hay conexión establecida')
                # Incluso si no hay conexión, devolvemos al menos un motor por defecto
                available_motors = [1]
                response.success = True
                response.message = "Modo simulación: usando motor virtual ID 1"
                response.motor_ids = available_motors
                self.get_logger().info(f'Usando motores virtuales: {available_motors}')
                return response
            
            # Intentar obtener motores reales
            try:
                # Primero intenta hacer ping a algunos motores (1-3)
                max_id_to_scan = 10
                
                for motor_id in range(1, max_id_to_scan + 1):
                    try:
                        self.get_logger().info(f'Probando motor ID: {motor_id}')
                        ping_result = self.manager.ping(motor_id)
                        
                        if ping_result and ping_result[0] is not None:
                            self.get_logger().info(f'Motor detectado en ID: {motor_id}, respuesta: {ping_result}')
                            available_motors.append(motor_id)
                        else:
                            self.get_logger().info(f'No se encontró motor en ID: {motor_id}, respuesta: {ping_result}')
                    except Exception as e:
                        self.get_logger().warning(f'Error al hacer ping al motor {motor_id}: {str(e)}')
            except Exception as e:
                self.get_logger().error(f'Error al escanear motores: {str(e)}')
            
            # Si no se encontraron motores, devolver al menos uno virtual
            if not available_motors:
                available_motors = [1]
                response.success = True
                response.message = "No se detectaron motores físicos. Usando motor virtual ID 1"
                self.get_logger().info('No se detectaron motores físicos. Usando motor virtual ID 1')
            else:
                response.success = True
                response.message = f"Se encontraron {len(available_motors)} motores disponibles: {available_motors}"
            
            response.motor_ids = available_motors
            self.get_logger().info(f'Motores disponibles (reales o virtuales): {available_motors}')
            
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio de obtención de motores disponibles: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            
            # Aún en caso de error, devolvemos al menos un motor
            available_motors = [1]
            response.success = True
            response.message = f"Error: {str(e)}. Usando motor virtual ID 1"
            response.motor_ids = available_motors
            self.get_logger().info(f'Usando motor virtual tras error: {available_motors}')
            
            return response
            
    # Función para configurar las ganancias de control de posición
    def handle_set_position_gains(self, request, response):
        """Callback para configurar las ganancias de control de posición"""
        try:
            # Si no hay motores especificados, no hay nada que hacer
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            connected_motors = []
            failed_motor_ids = []
            
            # Si el manager está disponible, intentamos configurar los motores reales
            if self.manager is not None:
                # Verificar conexión de cada motor
                for motor_id in request.motor_ids:
                    try:
                        ping_result = self.manager.ping(motor_id)
                        if ping_result:
                            connected_motors.append(motor_id)
                            self.get_logger().info(f'Motor {motor_id} conectado y listo para configuración')
                        else:
                            failed_motor_ids.append(motor_id)
                            self.get_logger().warning(f'Motor {motor_id} no responde')
                    except Exception as e:
                        self.get_logger().error(f'Error al hacer ping al motor {motor_id}: {str(e)}')
                        failed_motor_ids.append(motor_id)
                
                # Para cada motor conectado, configurar las ganancias
                for motor_id in connected_motors:
                    try:
                        # Configurar PID para modo posición
                        self.manager.set_p_gain_position((motor_id, request.p_gain_position))
                        self.manager.set_i_gain_position((motor_id, request.i_gain_position))
                        self.manager.set_d_gain_position((motor_id, request.d_gain_position))
                        
                        # Configurar PID para modo corriente (iq/id)
                        self.manager.set_p_gain_iq((motor_id, request.p_gain_iq))
                        self.manager.set_i_gain_iq((motor_id, request.i_gain_iq))
                        self.manager.set_d_gain_iq((motor_id, request.d_gain_iq))
                        self.manager.set_p_gain_id((motor_id, request.p_gain_id))
                        self.manager.set_i_gain_id((motor_id, request.i_gain_id))
                        self.manager.set_d_gain_id((motor_id, request.d_gain_id))
                        
                        # Establecer límite de corriente
                        self.manager.set_limit_iq_max((motor_id, request.iq_max))
                        
                        self.get_logger().info(f'Ganancias de posición configuradas para motor {motor_id}')
                    except Exception as e:
                        self.get_logger().error(f'Error al configurar ganancias del motor {motor_id}: {str(e)}')
                        if motor_id in connected_motors:
                            connected_motors.remove(motor_id)
                        failed_motor_ids.append(motor_id)
            
            # Preparar respuesta
            if connected_motors:
                response.success = True
                response.message = f"Ganancias de posición configuradas en {len(connected_motors)} motores"
                if failed_motor_ids:
                    response.message += f" ({len(failed_motor_ids)} fallaron)"
            else:
                response.success = False
                response.message = "No se pudo configurar ningún motor"
            
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio de configuración de ganancias: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    # Función para configurar las ganancias de control de corriente
    def handle_set_current_gains(self, request, response):
        """Callback para configurar las ganancias de control de corriente"""
        try:
            # Si no hay motores especificados, no hay nada que hacer
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            connected_motors = []
            failed_motor_ids = []
            
            # Si el manager está disponible, intentamos configurar los motores reales
            if self.manager is not None:
                # Verificar conexión de cada motor
                for motor_id in request.motor_ids:
                    try:
                        ping_result = self.manager.ping(motor_id)
                        if ping_result:
                            connected_motors.append(motor_id)
                            self.get_logger().info(f'Motor {motor_id} conectado y listo para configuración')
                        else:
                            failed_motor_ids.append(motor_id)
                            self.get_logger().warning(f'Motor {motor_id} no responde')
                    except Exception as e:
                        self.get_logger().error(f'Error al hacer ping al motor {motor_id}: {str(e)}')
                        failed_motor_ids.append(motor_id)
                
                # Para cada motor conectado, configurar las ganancias
                for motor_id in connected_motors:
                    try:
                        # Configurar PID para modo posición (a cero en modo corriente)
                        self.manager.set_p_gain_position((motor_id, 0.0))
                        self.manager.set_i_gain_position((motor_id, 0.0))
                        self.manager.set_d_gain_position((motor_id, 0.0))
                        
                        # Configurar PID para modo corriente (iq/id)
                        self.manager.set_p_gain_iq((motor_id, request.p_gain_iq))
                        self.manager.set_i_gain_iq((motor_id, request.i_gain_iq))
                        self.manager.set_d_gain_iq((motor_id, request.d_gain_iq))
                        self.manager.set_p_gain_id((motor_id, request.p_gain_id))
                        self.manager.set_i_gain_id((motor_id, request.i_gain_id))
                        self.manager.set_d_gain_id((motor_id, request.d_gain_id))
                        
                        # Configurar PID para modo fuerza (si existe)
                        if hasattr(self.manager, 'set_p_gain_force'):
                            self.manager.set_p_gain_force((motor_id, request.p_gain_force))
                            self.manager.set_i_gain_force((motor_id, request.i_gain_force))
                            self.manager.set_d_gain_force((motor_id, request.d_gain_force))
                        
                        # Establecer límite de corriente
                        self.manager.set_limit_iq_max((motor_id, request.iq_max))
                        
                        # Cambiar a modo corriente (0)
                        self.manager.set_mode((motor_id, 0))
                        
                        self.get_logger().info(f'Ganancias de corriente configuradas para motor {motor_id}')
                    except Exception as e:
                        self.get_logger().error(f'Error al configurar ganancias del motor {motor_id}: {str(e)}')
                        if motor_id in connected_motors:
                            connected_motors.remove(motor_id)
                        failed_motor_ids.append(motor_id)
            
            # Preparar respuesta
            if connected_motors:
                response.success = True
                response.message = f"Ganancias de corriente configuradas en {len(connected_motors)} motores"
                if failed_motor_ids:
                    response.message += f" ({len(failed_motor_ids)} fallaron)"
            else:
                response.success = False
                response.message = "No se pudo configurar ningún motor"
            
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio de configuración de ganancias: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {str(e)}"
            return response

def main():
    rclpy.init()
    node = WestwoodMotorServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error en la ejecución: {str(e)}")
        import traceback
        print(traceback.format_exc())
    finally:
        if hasattr(node, 'manager') and node.manager is not None:
            try:
                node.manager.close()
            except Exception as e:
                print(f"Error al cerrar el manager: {str(e)}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 