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
        
        # Setup services early to ensure they are available
        self.setup_services()
        self.get_logger().info('✅ Servicios configurados temprano')
        
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
                import os
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
                
                # Hacer un ping rápido para verificar que la comunicación funciona
                test_ping = manager.ping(1)  # Ping rápido para verificar comunicación
                
                self.managers.append(manager)
                self.usb_to_manager_map[i] = manager
                
                self.get_logger().info(f'✅ Manager de PyBear inicializado correctamente para {port}')
                self.get_logger().info(f'Puerto: {port}, Baudrate: {self.baudrate}')
                
                # Crear mapeo de IDs a USB solo si el manager se inicializó correctamente
                if i == 0:  # USB0
                    for motor_id in self.motor_ids_usb0:
                        self.motor_to_usb_map[motor_id] = {'usb_index': 0, 'local_id': motor_id, 'manager': manager}
                elif i == 1:  # USB1
                    for global_id, local_id in zip(self.motor_ids_usb1, [1, 2, 3, 4]):
                        self.motor_to_usb_map[global_id] = {'usb_index': 1, 'local_id': local_id, 'manager': manager}
                        
            except Exception as e:
                self.get_logger().error(f'❌ Error al inicializar Manager para {port}: {str(e)}')
                if manager is not None:
                    try:
                        manager.close()
                    except:
                        pass
                self.managers.append(None)
                
                # Si es el primer USB y falla, crear mapeo vacío para evitar errores
                if i == 0:
                    self.get_logger().warning(f'USB0 falló, pero continuando con inicialización...')
                elif i == 1:
                    self.get_logger().warning(f'USB1 falló, pero continuando con inicialización...')
        
        # Mantener compatibilidad con código existente
        self.manager = None
        for manager in self.managers:
            if manager is not None:
                self.manager = manager
                break
        
        # Logging del estado de inicialización
        working_managers = sum(1 for m in self.managers if m is not None)
        self.get_logger().info(f'📊 Managers inicializados: {working_managers}/{len(self.usb_ports)}')
        
        if working_managers == 0:
            self.get_logger().warning('⚠️ No se pudo inicializar ningún manager USB - funcionando en modo simulación')
        else:
            self.get_logger().info(f'✅ Sistema listo con {working_managers} USB(s) funcional(es)')
        
        self.get_logger().info(f'🗺️ Mapeo de motores: {self.motor_to_usb_map}')
        
        self.get_logger().info('DEBUG: Punto de control antes de servicios')
        # Continuar con la inicialización de servicios independientemente del estado de USB
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
            return False
        try:
            return manager.ping(local_id)
        except Exception:
            return False
    
    def get_all_motor_ids(self):
        """Obtener todos los IDs de motores configurados"""
        return list(self.motor_to_usb_map.keys())
    
    def setup_services(self):
        """Configurar todos los servicios ROS2"""
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
        
        # Añadir servicio para configurar el modo de operación
        try:
            self.set_mode_service = self.create_service(
                SetMode,
                'westwood_motor/set_mode',
                self.handle_set_mode
            )
            self.get_logger().info('Servicio para configurar el modo de operación registrado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio de configuración de modo: {str(e)}')
        
        # Añadir servicio para habilitar/deshabilitar el torque
        try:
            self.set_torque_enable_service = self.create_service(
                SetTorqueEnable,
                'westwood_motor/set_torque_enable',
                self.handle_set_torque_enable
            )
            self.get_logger().info('Servicio para habilitar/deshabilitar torque registrado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio de habilitación de torque: {str(e)}')
        
        # Añadir servicio para establecer corriente iq objetivo
        try:
            self.set_goal_iq_service = self.create_service(
                SetGoalIq,
                'westwood_motor/set_goal_iq',
                self.handle_set_goal_iq
            )
            self.get_logger().info('Servicio para establecer corriente iq objetivo registrado correctamente')
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio de establecimiento de corriente iq: {str(e)}')
        
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
            
            # Verificar conexión de cada motor usando el manager correcto
            for motor_id in request.motor_ids:
                try:
                    ping_result = self.ping_motor(motor_id)
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
                
                # Obtener el manager correcto para este motor
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    self.get_logger().error(f'No se encontró manager para motor {motor_id}')
                    failed_motor_ids.append(motor_id)
                    previous_positions.append(0.0)
                    continue
                
                try:
                    # Leer posición actual y guardarla
                    current_position_result = manager.get_present_position(local_id)
                    if current_position_result and len(current_position_result) > 0:
                        current_position = float(current_position_result[0][0][0])
                        previous_positions.append(current_position)
                        
                        # Configurar PID para el control de posición
                        # PID id/iq
                        manager.set_p_gain_iq((local_id, 0.02))
                        manager.set_i_gain_iq((local_id, 0.02))
                        manager.set_d_gain_iq((local_id, 0))
                        manager.set_p_gain_id((local_id, 0.02))
                        manager.set_i_gain_id((local_id, 0.02))
                        manager.set_d_gain_id((local_id, 0))
                        
                        # PID position mode
                        p_gain = 5.0
                        d_gain = 0.2
                        i_gain = 0.0
                        manager.set_p_gain_position((local_id, p_gain))
                        manager.set_i_gain_position((local_id, i_gain))
                        manager.set_d_gain_position((local_id, d_gain))
                        
                        # Poner en modo posición
                        manager.set_mode((local_id, 2))
                        
                        # Establecer límite de corriente
                        iq_max = 1.5
                        manager.set_limit_iq_max((local_id, iq_max))
                        
                        # Establecer posición inicial antes de habilitar torque
                        manager.set_goal_position((local_id, current_position))
                        
                        # Habilitar torque
                        manager.set_torque_enable((local_id, 1))
                        
                        # Mover a la posición objetivo
                        target_position = request.target_positions[idx]
                        manager.set_goal_position((local_id, target_position))
                        self.get_logger().info(f'Motor {motor_id} (local {local_id}) movido a {target_position} radianes')
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
            # Si no hay motores especificados, utilizar todos los motores configurados
            motor_ids = request.motor_ids
            if not motor_ids or len(motor_ids) == 0:
                motor_ids = self.get_all_motor_ids()
                
            positions = []
            connected_motors = []
            failed_motor_ids = []
            
            # Intentar obtener posiciones reales
            for motor_id in motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    failed_motor_ids.append(motor_id)
                    positions.append(0.0)
                    self.get_logger().warning(f'No se encontró manager para motor {motor_id}')
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        # Obtener posición actual del motor
                        position_result = manager.get_present_position(local_id)
                        if position_result and len(position_result) > 0:
                            current_position = float(position_result[0][0][0])
                            positions.append(current_position)
                            self.get_logger().info(f'Motor {motor_id} (local {local_id}): posición actual {current_position}')
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
            
            # Verificar si hay managers inicializados
            if not self.managers or all(m is None for m in self.managers):
                self.get_logger().error('Error: No hay managers inicializados')
                # Incluso si no hay conexión, devolvemos los motores configurados
                available_motors = self.get_all_motor_ids()
                response.success = True
                response.message = "Modo simulación: usando motores virtuales"
                response.motor_ids = available_motors
                self.get_logger().info(f'Usando motores virtuales: {available_motors}')
                return response
            
            # Intentar obtener motores reales escaneando los IDs configurados
            try:
                configured_motor_ids = self.get_all_motor_ids()
                
                for motor_id in configured_motor_ids:
                    try:
                        self.get_logger().info(f'Probando motor ID: {motor_id}')
                        ping_result = self.ping_motor(motor_id)
                        
                        if ping_result:
                            self.get_logger().info(f'Motor detectado en ID: {motor_id}')
                            available_motors.append(motor_id)
                        else:
                            self.get_logger().info(f'No se encontró motor en ID: {motor_id}')
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
            
            # Verificar conexión de cada motor y configurar las ganancias
            for motor_id in request.motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager para motor {motor_id}')
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        self.get_logger().info(f'Motor {motor_id} conectado y listo para configuración')
                        
                        # Configurar PID para modo posición
                        manager.set_p_gain_position((local_id, request.p_gain_position))
                        manager.set_i_gain_position((local_id, request.i_gain_position))
                        manager.set_d_gain_position((local_id, request.d_gain_position))
                        
                        # Configurar PID para modo corriente (iq/id)
                        manager.set_p_gain_iq((local_id, request.p_gain_iq))
                        manager.set_i_gain_iq((local_id, request.i_gain_iq))
                        manager.set_d_gain_iq((local_id, request.d_gain_iq))
                        manager.set_p_gain_id((local_id, request.p_gain_id))
                        manager.set_i_gain_id((local_id, request.i_gain_id))
                        manager.set_d_gain_id((local_id, request.d_gain_id))
                        
                        # Establecer límite de corriente
                        manager.set_limit_iq_max((local_id, request.iq_max))
                        
                        self.get_logger().info(f'Ganancias de posición configuradas para motor {motor_id} (local {local_id})')
                    else:
                        failed_motor_ids.append(motor_id)
                        self.get_logger().warning(f'Motor {motor_id} no responde')
                except Exception as e:
                    self.get_logger().error(f'Error al configurar ganancias del motor {motor_id}: {str(e)}')
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
            
            # Verificar conexión de cada motor y configurar las ganancias
            for motor_id in request.motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager para motor {motor_id}')
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        self.get_logger().info(f'Motor {motor_id} conectado y listo para configuración')
                        
                        # Configurar PID para modo posición (a cero en modo corriente)
                        manager.set_p_gain_position((local_id, 0.0))
                        manager.set_i_gain_position((local_id, 0.0))
                        manager.set_d_gain_position((local_id, 0.0))
                        
                        # Configurar PID para modo corriente (iq/id)
                        manager.set_p_gain_iq((local_id, request.p_gain_iq))
                        manager.set_i_gain_iq((local_id, request.i_gain_iq))
                        manager.set_d_gain_iq((local_id, request.d_gain_iq))
                        manager.set_p_gain_id((local_id, request.p_gain_id))
                        manager.set_i_gain_id((local_id, request.i_gain_id))
                        manager.set_d_gain_id((local_id, request.d_gain_id))
                        
                        # Configurar PID para modo fuerza (si existe)
                        if hasattr(manager, 'set_p_gain_force'):
                            manager.set_p_gain_force((local_id, request.p_gain_force))
                            manager.set_i_gain_force((local_id, request.i_gain_force))
                            manager.set_d_gain_force((local_id, request.d_gain_force))
                        
                        # Establecer límite de corriente
                        manager.set_limit_iq_max((local_id, request.iq_max))
                        
                        # Cambiar a modo corriente (0)
                        manager.set_mode((local_id, 0))
                        
                        self.get_logger().info(f'Ganancias de corriente configuradas para motor {motor_id} (local {local_id})')
                    else:
                        failed_motor_ids.append(motor_id)
                        self.get_logger().warning(f'Motor {motor_id} no responde')
                except Exception as e:
                    self.get_logger().error(f'Error al configurar ganancias del motor {motor_id}: {str(e)}')
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

    # Función para configurar el modo de operación
    def handle_set_mode(self, request, response):
        """Callback para configurar el modo de operación de los motores"""
        try:
            # Si no hay motores o modos especificados, no hay nada que hacer
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            # Si la cantidad de motores no coincide con la cantidad de modos
            if len(request.motor_ids) != len(request.modes):
                response.success = False
                response.message = "La cantidad de IDs de motores no coincide con la cantidad de modos"
                return response
            
            connected_motors = []
            failed_motor_ids = []
            
            # Configurar el modo para cada motor
            for motor_id in request.motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager para motor {motor_id}')
                    continue
                
                # Obtener el índice del motor en la lista original
                idx = request.motor_ids.index(motor_id)
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        self.get_logger().info(f'Motor {motor_id} conectado y listo para configuración')
                        
                        # Establecer el modo
                        manager.set_mode((local_id, request.modes[idx]))
                        self.get_logger().info(f'Modo {request.modes[idx]} configurado para motor {motor_id} (local {local_id})')
                    else:
                        failed_motor_ids.append(motor_id)
                        self.get_logger().warning(f'Motor {motor_id} no responde')
                except Exception as e:
                    self.get_logger().error(f'Error al configurar modo del motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
            
            # Preparar respuesta
            if connected_motors:
                response.success = True
                response.message = f"Modo configurado en {len(connected_motors)} motores"
                if failed_motor_ids:
                    response.message += f" ({len(failed_motor_ids)} fallaron)"
            else:
                response.success = False
                response.message = "No se pudo configurar ningún motor"
            
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio de configuración de modo: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
    
    # Función para habilitar/deshabilitar el torque
    def handle_set_torque_enable(self, request, response):
        """Callback para habilitar/deshabilitar el torque de los motores"""
        try:
            # Si no hay motores especificados, no hay nada que hacer
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            # Si la cantidad de motores no coincide con la cantidad de estados de torque
            if len(request.motor_ids) != len(request.enable_torque):
                response.success = False
                response.message = "La cantidad de IDs de motores no coincide con la cantidad de estados de torque"
                return response
            
            connected_motors = []
            failed_motor_ids = []
            
            # Configurar el torque para cada motor
            for motor_id in request.motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager para motor {motor_id}')
                    continue
                
                # Obtener el índice del motor en la lista original
                idx = request.motor_ids.index(motor_id)
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        self.get_logger().info(f'Motor {motor_id} conectado y listo para configuración')
                        
                        # Convertir bool a int (1: habilitado, 0: deshabilitado)
                        torque_state = 1 if request.enable_torque[idx] else 0
                        
                        # Establecer el estado del torque
                        manager.set_torque_enable((local_id, torque_state))
                        state_str = "habilitado" if torque_state == 1 else "deshabilitado"
                        self.get_logger().info(f'Torque {state_str} para motor {motor_id} (local {local_id})')
                    else:
                        failed_motor_ids.append(motor_id)
                        self.get_logger().warning(f'Motor {motor_id} no responde')
                except Exception as e:
                    self.get_logger().error(f'Error al configurar torque del motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
            
            # Preparar respuesta
            if connected_motors:
                response.success = True
                response.message = f"Torque configurado en {len(connected_motors)} motores"
                if failed_motor_ids:
                    response.message += f" ({len(failed_motor_ids)} fallaron)"
            else:
                response.success = False
                response.message = "No se pudo configurar ningún motor"
            
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio de configuración de torque: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {str(e)}"
            return response

    # Función para establecer la corriente iq objetivo
    def handle_set_goal_iq(self, request, response):
        """Callback para establecer la corriente iq objetivo de los motores"""
        try:
            # Si no hay motores especificados, no hay nada que hacer
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            # Si la cantidad de motores no coincide con la cantidad de corrientes
            if len(request.motor_ids) != len(request.goal_iq):
                response.success = False
                response.message = "La cantidad de IDs de motores no coincide con la cantidad de corrientes objetivo"
                return response
            
            connected_motors = []
            failed_motor_ids = []
            
            # Establecer corriente para cada motor
            for motor_id in request.motor_ids:
                manager, local_id = self.get_manager_for_motor(motor_id)
                
                if manager is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager para motor {motor_id}')
                    continue
                
                # Obtener el índice del motor en la lista original
                idx = request.motor_ids.index(motor_id)
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        self.get_logger().info(f'Motor {motor_id} conectado y listo para control de corriente')
                        
                        # Establecer corriente iq objetivo
                        manager.set_goal_iq((local_id, request.goal_iq[idx]))
                        self.get_logger().info(f'Corriente iq objetivo {request.goal_iq[idx]} configurada para motor {motor_id} (local {local_id})')
                    else:
                        failed_motor_ids.append(motor_id)
                        self.get_logger().warning(f'Motor {motor_id} no responde')
                except Exception as e:
                    self.get_logger().error(f'Error al establecer corriente del motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
            
            # Preparar respuesta
            if connected_motors:
                response.success = True
                response.message = f"Corriente iq configurada en {len(connected_motors)} motores"
                if failed_motor_ids:
                    response.message += f" ({len(failed_motor_ids)} fallaron)"
            else:
                response.success = False
                response.message = "No se pudo configurar ningún motor"
            
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio de establecimiento de corriente iq: {str(e)}')
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
        # Cerrar todos los managers
        if hasattr(node, 'managers'):
            for i, manager in enumerate(node.managers):
                if manager is not None:
                    try:
                        manager.close()
                        print(f"Manager {i} cerrado correctamente")
                    except Exception as e:
                        print(f"Error al cerrar el manager {i}: {str(e)}")
        # Mantener compatibilidad
        elif hasattr(node, 'manager') and node.manager is not None:
            try:
                node.manager.close()
            except Exception as e:
                print(f"Error al cerrar el manager: {str(e)}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 