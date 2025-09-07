#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import SetMotorIdAndTarget, GetMotorPositions, GetAvailableMotors
from westwood_motor_interfaces.srv import SetGains, SetMode, SetTorqueEnable, SetGoalIq
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
        
        # Initialize empty motor mapping for early service setup
        self.motor_to_usb_map = {}
        self.managers = []
        
        # Parámetros configurables para dual USB
        self.declare_parameter('usb_ports', ['/dev/ttyUSB0', '/dev/ttyUSB1'])
        self.declare_parameter('baudrate', 8000000)
        self.declare_parameter('motor_ids_usb0', [1, 2, 3, 4])  # Motors en USB0
        self.declare_parameter('motor_ids_usb1', [5, 6, 7, 8])  # Motors en USB1 (IDs globales)
        self.declare_parameter('debug', False)
        
        # Obtener parámetros
        self.usb_ports = self.get_parameter('usb_ports').value
        self.baudrate = self.get_parameter('baudrate').value
        self.motor_ids_usb0 = self.get_parameter('motor_ids_usb0').value
        self.motor_ids_usb1 = self.get_parameter('motor_ids_usb1').value
        self.debug = self.get_parameter('debug').value
        
        # Crear mapeo de motor ID a USB y manager
        self.motor_to_usb_map = {}
        self.usb_to_manager_map = {}
        
        # Inicializar managers para cada USB con timeout
        self.managers = []
        for i, port in enumerate(self.usb_ports):
            manager = None
            try:
                self.get_logger().info(f'Intentando conectar al puerto: {port}')
                
                # Verificar si el puerto existe antes de intentar conectar
                if not os.path.exists(port):
                    self.get_logger().warning(f'Puerto {port} no existe, saltando...')
                    self.managers.append(None)
                    continue
                
                # Intentar inicializar manager con timeout implícito
                manager = Manager(
                    port=port,
                    baudrate=self.baudrate,
                    debug=self.debug
                )
                
                self.managers.append(manager)
                self.usb_to_manager_map[i] = manager
                
                self.get_logger().info(f'✅ Manager de PyBear inicializado correctamente para {port}')
                
                # Crear mapeo de IDs a USB solo si el manager se inicializó correctamente
                if i == 0:  # USB0
                    for motor_id in self.motor_ids_usb0:
                        self.motor_to_usb_map[motor_id] = {'usb_index': 0, 'local_id': motor_id, 'manager': manager}
                elif i == 1:  # USB1
                    # Los motores en el segundo USB tienen IDs locales 1-4, pero IDs globales 5-8
                    for global_id, local_id in zip(self.motor_ids_usb1, self.motor_ids_usb0):
                        self.motor_to_usb_map[global_id] = {'usb_index': 1, 'local_id': local_id, 'manager': manager}
                        
            except Exception as e:
                self.get_logger().error(f'❌ Error al inicializar Manager para {port}: {str(e)}')
                if manager is not None:
                    try:
                        manager.close()
                    except:
                        pass
                self.managers.append(None)

        # Logging del estado de inicialización
        working_managers = sum(1 for m in self.managers if m is not None)
        self.get_logger().info(f'📊 Managers inicializados: {working_managers}/{len(self.usb_ports)}')
        
        if working_managers == 0:
            self.get_logger().warning('⚠️ No se pudo inicializar ningún manager USB - funcionando en modo simulación')
        else:
            self.get_logger().info(f'✅ Sistema listo con {working_managers} USB(s) funcional(es)')
        
        self.get_logger().info(f'🗺️ Mapeo de motores: {self.motor_to_usb_map}')
        
        # Inicialización de servicios
        self.get_logger().info('🚀 Iniciando configuración de servicios...')
        try:
            self.setup_services()
            self.get_logger().info('✅ Configuración de servicios completada')
        except Exception as e:
            self.get_logger().error(f'❌ Error al configurar servicios: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def get_manager_for_motor(self, motor_id):
        """Obtener el manager correcto para un motor ID dado"""
        if motor_id in self.motor_to_usb_map:
            return self.motor_to_usb_map[motor_id]['manager'], self.motor_to_usb_map[motor_id]['local_id']
        return None, motor_id
    
    def ping_motor(self, motor_id):
        """Hacer ping a un motor específico usando el manager correcto"""
        manager, local_id = self.get_manager_for_motor(motor_id)
        if manager is None:
            return None
        try:
            # manager.ping() devuelve una lista, ej: [(status, error)] o [None]
            return manager.ping(local_id)
        except Exception as e:
            self.get_logger().warning(f"Excepción durante el ping para el motor {motor_id} (local {local_id}): {e}")
            return None
    
    def get_all_motor_ids(self):
        """Obtener todos los IDs de motores configurados"""
        return sorted(list(self.motor_to_usb_map.keys()))
    
    def setup_services(self):
        """Configurar todos los servicios ROS2"""
        self.create_service(SetMotorIdAndTarget, 'westwood_motor/set_motor_id_and_target', self.handle_motor_ids_and_target)
        self.create_service(GetMotorPositions, 'westwood_motor/get_motor_positions', self.handle_get_motor_positions)
        self.create_service(GetAvailableMotors, 'westwood_motor/get_available_motors', self.handle_get_available_motors)
        self.create_service(SetGains, 'westwood_motor/set_position_gains', self.handle_set_position_gains)
        self.create_service(SetGains, 'westwood_motor/set_current_gains', self.handle_set_current_gains)
        self.create_service(SetMode, 'westwood_motor/set_mode', self.handle_set_mode)
        self.create_service(SetTorqueEnable, 'westwood_motor/set_torque_enable', self.handle_set_torque_enable)
        self.create_service(SetGoalIq, 'westwood_motor/set_goal_iq', self.handle_set_goal_iq)
        self.get_logger().info('Todos los servicios han sido registrados.')

    def handle_get_available_motors(self, request, response):
        """Callback para obtener las IDs de los motores disponibles actualmente"""
        try:
            available_motors = []
            
            if not any(self.managers):
                self.get_logger().error('Error: No hay managers inicializados, funcionando en modo simulación.')
                response.success = True
                response.message = "Modo simulación: no hay managers USB disponibles."
                response.motor_ids = []
                return response
            
            all_motor_ids = self.get_all_motor_ids()
            
            for motor_id in all_motor_ids:
                try:
                    self.get_logger().info(f'Probando motor ID: {motor_id}')
                    ping_result = self.ping_motor(motor_id)
                    
                    # El ping devuelve una lista con una tupla en caso de éxito [((firmware, hardware), error_code)] o [None] en caso de fallo.
                    if ping_result and ping_result[0] is not None:
                        self.get_logger().info(f'Motor detectado en ID: {motor_id} -> Respuesta: {ping_result}')
                        available_motors.append(motor_id)
                    else:
                        self.get_logger().info(f'No se encontró motor en ID: {motor_id}')
                except Exception as e:
                    self.get_logger().warning(f'Error al hacer ping al motor {motor_id}: {str(e)}')
            
            if not available_motors:
                response.success = True
                response.message = "No se detectaron motores físicos."
                self.get_logger().info('No se detectaron motores físicos.')
            else:
                response.success = True
                response.message = f"Se encontraron {len(available_motors)} motores disponibles: {available_motors}"
            
            response.motor_ids = available_motors
            self.get_logger().info(f'Motores disponibles: {available_motors}')
            
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio de obtención de motores disponibles: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            
            response.success = False
            response.message = f"Error: {str(e)}."
            response.motor_ids = []
            return response

    # Función principal para manejar IDs de motores y sus posiciones objetivo
    def handle_motor_ids_and_target(self, request, response):
        """Callback para controlar múltiples motores con posiciones objetivo individuales"""
        try:
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            if len(request.motor_ids) != len(request.target_positions):
                response.success = False
                response.message = "La cantidad de IDs de motores no coincide con la cantidad de posiciones objetivo"
                return response
            
            connected_motors = []
            failed_motor_ids = []
            previous_positions = []
            
            for motor_id in request.motor_ids:
                ping_result = self.ping_motor(motor_id)
                if ping_result and ping_result[0] is not None:
                    connected_motors.append(motor_id)
                else:
                    failed_motor_ids.append(motor_id)

            for i, motor_id in enumerate(connected_motors):
                idx = request.motor_ids.index(motor_id)
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    failed_motor_ids.append(motor_id)
                    previous_positions.append(0.0)
                    continue
                
                try:
                    current_position_result = manager.get_present_position(local_id)
                    if current_position_result and len(current_position_result) > 0:
                        current_position = float(current_position_result[0][0][0])
                        previous_positions.append(current_position)
                        
                        manager.set_p_gain_iq((local_id, 0.02))
                        manager.set_i_gain_iq((local_id, 0.02))
                        manager.set_d_gain_iq((local_id, 0))
                        manager.set_p_gain_id((local_id, 0.02))
                        manager.set_i_gain_id((local_id, 0.02))
                        manager.set_d_gain_id((local_id, 0))
                        
                        manager.set_p_gain_position((local_id, 5.0))
                        manager.set_i_gain_position((local_id, 0.0))
                        manager.set_d_gain_position((local_id, 0.2))
                        
                        manager.set_mode((local_id, 2))
                        manager.set_limit_iq_max((local_id, 1.5))
                        manager.set_goal_position((local_id, current_position))
                        manager.set_torque_enable((local_id, 1))
                        
                        target_position = request.target_positions[idx]
                        manager.set_goal_position((local_id, target_position))
                        self.get_logger().info(f'Motor {motor_id} (local {local_id}) movido a {target_position} radianes')
                    else:
                        previous_positions.append(0.0)
                        failed_motor_ids.append(motor_id)
                except Exception as e:
                    self.get_logger().error(f'Error al configurar/mover motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
                    previous_positions.append(0.0)
            
            while len(previous_positions) < len(request.motor_ids):
                previous_positions.append(0.0)
            
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
            response.success = True
            response.message = f"Error: {str(e)}. Modo simulación activado"
            response.previous_positions = [0.0] * len(request.motor_ids)
            return response

    def handle_get_motor_positions(self, request, response):
        """Callback para obtener las posiciones actuales de los motores especificados"""
        try:
            motor_ids = request.motor_ids
            if not motor_ids or len(motor_ids) == 0:
                motor_ids = self.get_all_motor_ids()
                
            positions = []
            
            for motor_id in motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    positions.append(0.0)
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result and ping_result[0] is not None:
                        position_result = manager.get_present_position(local_id)
                        if position_result and len(position_result) > 0:
                            current_position = float(position_result[0][0][0])
                            positions.append(current_position)
                        else:
                            positions.append(0.0)
                    else:
                        positions.append(0.0)
                except Exception as e:
                    self.get_logger().error(f'Error al leer posición del motor {motor_id}: {str(e)}')
                    positions.append(0.0)
            
            response.success = True
            response.message = "Lectura de posiciones (reales o simuladas)"
            response.positions = positions
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error en servicio de lectura de posiciones: {str(e)}')
            response.success = True
            response.message = f"Error: {str(e)}. Usando posiciones simuladas 0.0"
            response.positions = [0.0] * len(request.motor_ids) if request.motor_ids else [0.0]
            return response

    def handle_set_position_gains(self, request, response):
        """Callback para configurar las ganancias de control de posición"""
        try:
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            for motor_id in request.motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                if manager is None: continue
                
                ping_result = self.ping_motor(motor_id)
                if ping_result and ping_result[0] is not None:
                    try:
                        manager.set_p_gain_position((local_id, request.p_gain_position))
                        manager.set_i_gain_position((local_id, request.i_gain_position))
                        manager.set_d_gain_position((local_id, request.d_gain_position))
                        manager.set_p_gain_iq((local_id, request.p_gain_iq))
                        manager.set_i_gain_iq((local_id, request.i_gain_iq))
                        manager.set_d_gain_iq((local_id, request.d_gain_iq))
                        manager.set_p_gain_id((local_id, request.p_gain_id))
                        manager.set_i_gain_id((local_id, request.i_gain_id))
                        manager.set_d_gain_id((local_id, request.d_gain_id))
                        manager.set_limit_iq_max((local_id, request.iq_max))
                    except Exception as e:
                        self.get_logger().error(f'Error al configurar ganancias del motor {motor_id}: {str(e)}')

            response.success = True
            response.message = "Configuración de ganancias de posición intentada."
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error en servicio de configuración de ganancias: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def handle_set_current_gains(self, request, response):
        """Callback para configurar las ganancias de control de corriente"""
        try:
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            for motor_id in request.motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                if manager is None: continue

                ping_result = self.ping_motor(motor_id)
                if ping_result and ping_result[0] is not None:
                    try:
                        manager.set_p_gain_iq((local_id, request.p_gain_iq))
                        manager.set_i_gain_iq((local_id, request.i_gain_iq))
                        manager.set_d_gain_iq((local_id, request.d_gain_iq))
                        manager.set_p_gain_id((local_id, request.p_gain_id))
                        manager.set_i_gain_id((local_id, request.i_gain_id))
                        manager.set_d_gain_id((local_id, request.d_gain_id))
                        manager.set_limit_iq_max((local_id, request.iq_max))
                        manager.set_mode((local_id, 0))
                    except Exception as e:
                        self.get_logger().error(f'Error al configurar ganancias del motor {motor_id}: {str(e)}')

            response.success = True
            response.message = "Configuración de ganancias de corriente intentada."
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error en servicio de configuración de ganancias: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            return response

    def handle_set_mode(self, request, response):
        """Callback para configurar el modo de operación de los motores"""
        try:
            if not request.motor_ids or len(request.motor_ids) != len(request.modes):
                response.success = False
                response.message = "El número de IDs de motor no coincide con el número de modos"
                return response
            
            for i, motor_id in enumerate(request.motor_ids):
                manager, local_id = self.get_manager_for_motor(motor_id)
                if manager is None: continue

                ping_result = self.ping_motor(motor_id)
                if ping_result and ping_result[0] is not None:
                    try:
                        manager.set_mode((local_id, request.modes[i]))
                    except Exception as e:
                        self.get_logger().error(f'Error al configurar modo del motor {motor_id}: {str(e)}')

            response.success = True
            response.message = "Configuración de modo intentada."
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error en servicio de configuración de modo: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    def handle_set_torque_enable(self, request, response):
        """Callback para habilitar/deshabilitar el torque de los motores"""
        try:
            if not request.motor_ids or len(request.motor_ids) != len(request.enable_torque):
                response.success = False
                response.message = "El número de IDs de motor no coincide con el número de estados de torque"
                return response
            
            for i, motor_id in enumerate(request.motor_ids):
                manager, local_id = self.get_manager_for_motor(motor_id)
                if manager is None: continue

                ping_result = self.ping_motor(motor_id)
                if ping_result and ping_result[0] is not None:
                    try:
                        torque_state = 1 if request.enable_torque[i] else 0
                        manager.set_torque_enable((local_id, torque_state))
                    except Exception as e:
                        self.get_logger().error(f'Error al configurar torque del motor {motor_id}: {str(e)}')

            response.success = True
            response.message = "Configuración de torque intentada."
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error en servicio de configuración de torque: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            return response

    def handle_set_goal_iq(self, request, response):
        """Callback para establecer la corriente iq objetivo de los motores"""
        try:
            if not request.motor_ids or len(request.motor_ids) != len(request.goal_iq):
                response.success = False
                response.message = "El número de IDs de motor no coincide con el número de corrientes objetivo"
                return response
            
            for i, motor_id in enumerate(request.motor_ids):
                manager, local_id = self.get_manager_for_motor(motor_id)
                if manager is None: continue

                ping_result = self.ping_motor(motor_id)
                if ping_result and ping_result[0] is not None:
                    try:
                        manager.set_goal_iq((local_id, request.goal_iq[i]))
                    except Exception as e:
                        self.get_logger().error(f'Error al establecer corriente del motor {motor_id}: {str(e)}')

            response.success = True
            response.message = "Configuración de corriente IQ intentada."
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error en servicio de establecimiento de corriente iq: {str(e)}')
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
        # Cerrar todos los managers
        if hasattr(node, 'managers'):
            for i, manager in enumerate(node.managers):
                if manager is not None:
                    try:
                        manager.close()
                        print(f"Manager {i} cerrado correctamente")
                    except Exception as e:
                        print(f"Error al cerrar el manager {i}: {str(e)}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()