#!/usr/bin/env python3

import os
import time
from pybear import Manager

class HardwareManager:
    """Manager robusto para múltiples USBs con remapeo de IDs automático"""
    
    def __init__(self, node, usb_ports=None, baudrate=8000000, auto_detect=True, debug=False):
        self.node = node
        self.usb_ports = usb_ports or ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']
        self.baudrate = baudrate
        self.auto_detect = auto_detect
        self.debug = debug
        
        # Cache para reducir llamadas redundantes
        self.position_cache = {}
        self.velocity_cache = {}
        self.cache_timeout = 0.001  # 1ms cache like pd_control_node.py
        
        # Mapeo de motor ID global a información del USB
        self.motor_to_usb_map = {}
        self.managers = []
        self.usb_to_manager_map = {}
        self.detected_motors = set()
        self.max_motor_scan_range = 10
        
        # Estado de conexión
        self.hardware_connected = False
        
        self.initialize_hardware()
    
    def initialize_hardware(self):
        """Inicializar conexiones de hardware de forma robusta"""
        self.node.get_logger().info('🔧 Inicializando hardware manager...')
        
        # Limpiar estado previo
        self.managers.clear()
        self.usb_to_manager_map.clear()
        self.motor_to_usb_map.clear()
        self.detected_motors.clear()
        
        # Inicializar managers para cada USB con manejo de errores
        for i, port in enumerate(self.usb_ports):
            manager = None
            try:
                # Verificar si el puerto existe
                if not os.path.exists(port):
                    self.node.get_logger().info(f'Puerto {port} no existe, saltando...')
                    self.managers.append(None)
                    continue
                
                self.node.get_logger().info(f'Conectando a {port}...')
                
                # Crear manager
                manager = Manager.BEAR(port=port, baudrate=self.baudrate)
                
                # Ping de prueba rápido
                test_ping = manager.ping(1)
                
                self.managers.append(manager)
                self.usb_to_manager_map[i] = manager
                self.hardware_connected = True
                
                self.node.get_logger().info(f'✅ Conexión exitosa a {port}')
                        
            except Exception as e:
                self.node.get_logger().warning(f'⚠️ No se pudo conectar a {port}: {str(e)}')
                if manager is not None:
                    try:
                        manager.close()
                    except:
                        pass
                self.managers.append(None)
        
        # Si hay hardware conectado, detectar motores
        if self.hardware_connected and self.auto_detect:
            self.detect_and_map_motors()
        
        # Log del estado final
        if self.hardware_connected:
            self.node.get_logger().info(f'✅ Hardware manager inicializado con {len([m for m in self.managers if m is not None])} USB(s)')
        else:
            self.node.get_logger().warning('⚠️ Hardware manager en modo simulación (sin hardware)')
    
    def detect_and_map_motors(self):
        """Detectar automáticamente motores y crear mapeo inteligente como el servidor"""
        self.node.get_logger().info('🔍 Detectando motores...')
        
        self.detected_motors.clear()
        
        # Detectar motores en todos los USBs
        all_detections = {}
        
        for usb_index, manager in enumerate(self.managers):
            if manager is None:
                continue
            
            detected_motors = []
            try:
                for motor_id in range(1, self.max_motor_scan_range + 1):
                    try:
                        result = manager.ping(motor_id)
                        if result and len(result) > 0 and result[0] is not None:
                            detected_motors.append(motor_id)
                    except Exception:
                        continue
            except Exception as e:
                self.node.get_logger().error(f"Error durante detección en USB{usb_index}: {e}")
                continue

            if detected_motors:
                all_detections[usb_index] = detected_motors
                self.node.get_logger().info(f'USB{usb_index}: {len(detected_motors)} motor(es) - IDs locales: {detected_motors}')
        
        # Remapear IDs para evitar conflictos
        used_global_ids = set()
        next_available_id = 1
        
        for usb_index in sorted(all_detections.keys()):
            detected_motors = all_detections[usb_index]
            manager = self.managers[usb_index]
            
            for local_id in detected_motors:
                # Si el ID local no está en uso, usarlo como global
                if local_id not in used_global_ids:
                    global_id = local_id
                    used_global_ids.add(global_id)
                else:
                    # Buscar siguiente ID disponible
                    while next_available_id in used_global_ids:
                        next_available_id += 1
                    global_id = next_available_id
                    used_global_ids.add(global_id)
                    next_available_id += 1
                    self.node.get_logger().info(f'Remapeo: Motor USB{usb_index} local {local_id} -> global {global_id}')
                
                self.motor_to_usb_map[global_id] = {
                    'usb_index': usb_index,
                    'local_id': local_id,
                    'manager': manager
                }
                self.detected_motors.add(global_id)
        
        # Mostrar resumen
        if self.motor_to_usb_map:
            total = len(self.motor_to_usb_map)
            self.node.get_logger().info(f'✅ {total} motor(es) detectado(s) y mapeado(s)')
            for usb_idx in sorted(set(m['usb_index'] for m in self.motor_to_usb_map.values())):
                mappings = [(gid, m['local_id']) for gid, m in self.motor_to_usb_map.items() if m['usb_index'] == usb_idx]
                mappings.sort()
                mapping_str = [f"global {g}->local {l}" for g, l in mappings]
                self.node.get_logger().info(f'  USB{usb_idx}: {mapping_str}')
        else:
            self.node.get_logger().warning('⚠️ No se detectaron motores')
    
    def get_available_motors(self):
        """Obtener lista de motores disponibles"""
        return sorted(list(self.detected_motors))
    
    def is_motor_available(self, motor_id):
        """Verificar si un motor está disponible"""
        return motor_id in self.detected_motors
    
    def get_manager_for_motor(self, motor_id):
        """Obtener manager y ID local para un motor global"""
        if motor_id not in self.motor_to_usb_map:
            return None, None
        
        motor_info = self.motor_to_usb_map[motor_id]
        return motor_info['manager'], motor_info['local_id']
    
    def ping(self, *motor_ids):
        """Hacer ping a motores usando IDs globales"""
        if not self.hardware_connected:
            # Modo simulación - devolver valores simulados
            return [True for _ in motor_ids]
        
        results = []
        for motor_id in motor_ids:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                results.append(False)
                continue
            
            try:
                result = manager.ping(local_id)
                results.append(result[0] if result and len(result) > 0 else False)
            except Exception as e:
                self.node.get_logger().warning(f"Error ping motor {motor_id}: {e}")
                results.append(False)
        
        return results
    
    def set_goal_position(self, *position_pairs):
        """Establecer posiciones objetivo usando IDs globales"""
        if not self.hardware_connected:
            # Modo simulación - log las posiciones
            for motor_id, position in position_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} -> posición {position}")
            return True
        
        # Agrupar comandos por manager
        manager_commands = {}
        
        for motor_id, position in position_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, position))
        
        # Enviar comandos a cada manager
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_goal_position(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error estableciendo posiciones: {e}")
                success = False
        
        return success
    
    def set_torque_enable(self, *motor_enable_pairs):
        """Habilitar/deshabilitar torque usando IDs globales"""
        if not self.hardware_connected:
            # Modo simulación
            for motor_id, enable in motor_enable_pairs:
                state = "habilitado" if enable else "deshabilitado"
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} torque {state}")
            return True
        
        # Agrupar por manager
        manager_commands = {}
        
        for motor_id, enable in motor_enable_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, enable))
        
        # Enviar comandos
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_torque_enable(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando torque: {e}")
                success = False
        
        return success
    
    def set_mode(self, *motor_mode_pairs):
        """Establecer modo de control usando IDs globales"""
        if not self.hardware_connected:
            # Modo simulación
            for motor_id, mode in motor_mode_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} modo {mode}")
            return True
        
        # Agrupar por manager
        manager_commands = {}
        
        for motor_id, mode in motor_mode_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, mode))
        
        # Enviar comandos
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_mode(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando modo: {e}")
                success = False
        
        return success
    
    def get_present_position(self, *motor_ids):
        """Obtener posiciones actuales usando IDs globales"""
        if not self.hardware_connected:
            # Modo simulación - devolver posiciones cero
            return [0.0 for _ in motor_ids]
        
        positions = []
        for motor_id in motor_ids:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                positions.append(0.0)
                continue
            
            try:
                result = manager.get_present_position(local_id)
                
                # Debug: log the raw result to understand the format
                if self.debug:
                    self.node.get_logger().info(f"Motor {motor_id} raw result: {result}")
                
                # Parse de forma consistente con pd_control_node.py línea 67:
                # self.pos1 = self.bear.get_present_position(self.m_id_1)[0][0][0]
                position = 0.0
                if result is not None and len(result) > 0:
                    # Formato [[[position]]] -> extraer [0][0][0]
                    if isinstance(result[0], (list, tuple)) and len(result[0]) > 0:
                        if isinstance(result[0][0], (list, tuple)) and len(result[0][0]) > 0:
                            position = float(result[0][0][0])
                        else:
                            position = float(result[0][0])
                    else:
                        position = float(result[0])
                
                positions.append(position)
                
            except Exception as e:
                # Manejo silencioso de errores de comunicación como en pd_control_node.py
                if 'list index out of range' not in str(e) and 'device disconnected' not in str(e):
                    self.node.get_logger().debug(f"Error leyendo posición motor {motor_id}: {e}")
                positions.append(0.0)
        
        return positions
    
    def configure_pid_gains(self, motor_ids, p_gain=5.0, d_gain=0.2, i_gain=0.0):
        """Configurar ganancias PID como en el código de referencia"""
        if not self.hardware_connected:
            self.node.get_logger().info("[SIM] Configurando ganancias PID")
            return True
        
        success = True
        
        # Configurar ganancias de posición (las más importantes para control de posición)
        position_pairs = [(motor_id, p_gain) for motor_id in motor_ids]
        
        # Agrupar comandos por manager
        manager_commands = {}
        for motor_id, gain_value in position_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = {'p_pairs': [], 'i_pairs': [], 'd_pairs': []}
            
            manager_commands[manager]['p_pairs'].append((local_id, p_gain))
            manager_commands[manager]['i_pairs'].append((local_id, i_gain))
            manager_commands[manager]['d_pairs'].append((local_id, d_gain))
        
        # Enviar comandos a cada manager
        for manager, commands in manager_commands.items():
            try:
                if commands['p_pairs']:
                    manager.set_p_gain_position(*commands['p_pairs'])
                if commands['i_pairs']:
                    manager.set_i_gain_position(*commands['i_pairs'])
                if commands['d_pairs']:
                    manager.set_d_gain_position(*commands['d_pairs'])
                    
                self.node.get_logger().info(f"Ganancias PID configuradas: P={p_gain}, I={i_gain}, D={d_gain}")
                
            except Exception as e:
                self.node.get_logger().error(f"Error configurando ganancias PID: {e}")
                success = False
        
        return success
    
    def set_position_mode_and_limits(self, motor_ids, iq_max=3.0):
        """Configurar modo de posición y límites como en el código de referencia"""
        if not self.hardware_connected:
            self.node.get_logger().info("[SIM] Configurando modo posición y límites")
            return True
        
        success = True
        
        # Agrupar comandos por manager
        manager_commands = {}
        for motor_id in motor_ids:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = {'mode_pairs': [], 'limit_pairs': []}
            
            manager_commands[manager]['mode_pairs'].append((local_id, 2))  # Modo posición
            manager_commands[manager]['limit_pairs'].append((local_id, iq_max))
        
        # Enviar comandos
        for manager, commands in manager_commands.items():
            try:
                if commands['mode_pairs']:
                    manager.set_mode(*commands['mode_pairs'])
                if commands['limit_pairs']:
                    manager.set_limit_i_max(*commands['limit_pairs'])
                    
                self.node.get_logger().info(f"Modo posición configurado, límite iq_max={iq_max}")
                
            except Exception as e:
                self.node.get_logger().error(f"Error configurando modo/límites: {e}")
                success = False
        
        return success
    
    def set_goal_iq(self, *motor_current_pairs):
        """Establecer corrientes objetivo usando IDs globales (para teleoperación remota)"""
        if not self.hardware_connected:
            # Modo simulación - log las corrientes
            for motor_id, current in motor_current_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} -> corriente {current:.3f} A")
            return True
        
        # Agrupar comandos por manager
        manager_commands = {}
        
        for motor_id, current in motor_current_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, current))
        
        # Enviar comandos a cada manager
        success = True
        for manager, commands in manager_commands.items():
            try:
                # Usar set_goal_iq del manager de Pybear
                manager.set_goal_iq(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error estableciendo corrientes: {e}")
                success = False
        
        return success

    def set_p_gain_iq(self, *motor_gain_pairs):
        """Establecer ganancia P para control iq usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} P_gain_iq -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_p_gain_iq(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando P_gain_iq: {e}")
                success = False
        
        return success

    def set_i_gain_iq(self, *motor_gain_pairs):
        """Establecer ganancia I para control iq usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} I_gain_iq -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_i_gain_iq(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando I_gain_iq: {e}")
                success = False
        
        return success

    def set_d_gain_iq(self, *motor_gain_pairs):
        """Establecer ganancia D para control iq usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} D_gain_iq -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_d_gain_iq(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando D_gain_iq: {e}")
                success = False
        
        return success

    def set_p_gain_id(self, *motor_gain_pairs):
        """Establecer ganancia P para control id usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} P_gain_id -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_p_gain_id(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando P_gain_id: {e}")
                success = False
        
        return success

    def set_i_gain_id(self, *motor_gain_pairs):
        """Establecer ganancia I para control id usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} I_gain_id -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_i_gain_id(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando I_gain_id: {e}")
                success = False
        
        return success

    def set_d_gain_id(self, *motor_gain_pairs):
        """Establecer ganancia D para control id usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} D_gain_id -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_d_gain_id(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando D_gain_id: {e}")
                success = False
        
        return success

    def set_p_gain_position(self, *motor_gain_pairs):
        """Establecer ganancia P para control de posición usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} P_gain_position -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_p_gain_position(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando P_gain_position: {e}")
                success = False
        
        return success

    def set_i_gain_position(self, *motor_gain_pairs):
        """Establecer ganancia I para control de posición usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} I_gain_position -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_i_gain_position(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando I_gain_position: {e}")
                success = False
        
        return success

    def set_d_gain_position(self, *motor_gain_pairs):
        """Establecer ganancia D para control de posición usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} D_gain_position -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_d_gain_position(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando D_gain_position: {e}")
                success = False
        
        return success

    def set_p_gain_force(self, *motor_gain_pairs):
        """Establecer ganancia P para control de fuerza usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} P_gain_force -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_p_gain_force(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando P_gain_force: {e}")
                success = False
        
        return success

    def set_i_gain_force(self, *motor_gain_pairs):
        """Establecer ganancia I para control de fuerza usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} I_gain_force -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_i_gain_force(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando I_gain_force: {e}")
                success = False
        
        return success

    def set_d_gain_force(self, *motor_gain_pairs):
        """Establecer ganancia D para control de fuerza usando IDs globales"""
        if not self.hardware_connected:
            for motor_id, gain in motor_gain_pairs:
                self.node.get_logger().debug(f"[SIM] Motor {motor_id} D_gain_force -> {gain}")
            return True
        
        manager_commands = {}
        for motor_id, gain in motor_gain_pairs:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                self.node.get_logger().warning(f"Motor {motor_id} no disponible")
                continue
            
            if manager not in manager_commands:
                manager_commands[manager] = []
            manager_commands[manager].append((local_id, gain))
        
        success = True
        for manager, commands in manager_commands.items():
            try:
                manager.set_d_gain_force(*commands)
            except Exception as e:
                self.node.get_logger().error(f"Error configurando D_gain_force: {e}")
                success = False
        
        return success

    def get_present_velocity(self, *motor_ids):
        """Obtener velocidades actuales usando IDs globales con formato pd_control_node.py"""
        if not self.hardware_connected:
            # Modo simulación - devolver velocidades cero
            return [0.0 for _ in motor_ids]
        
        velocities = []
        for motor_id in motor_ids:
            manager, local_id = self.get_manager_for_motor(motor_id)
            if manager is None or local_id is None:
                velocities.append(0.0)
                continue
            
            try:
                # Usar el mismo formato que pd_control_node.py línea 69:
                # self.vel1 = self.bear.get_present_velocity(self.m_id_1)[0][0][0]
                result = manager.get_present_velocity(local_id)
                
                # Parse de forma consistente con pd_control_node.py
                velocity = 0.0
                if result is not None and len(result) > 0:
                    # Formato [[[velocity]]] -> extraer [0][0][0]
                    if isinstance(result[0], (list, tuple)) and len(result[0]) > 0:
                        if isinstance(result[0][0], (list, tuple)) and len(result[0][0]) > 0:
                            velocity = float(result[0][0][0])
                        else:
                            velocity = float(result[0][0])
                    else:
                        velocity = float(result[0])

                velocities.append(velocity)
                
            except Exception as e:
                # Manejo silencioso de errores de comunicación como en pd_control_node.py
                if 'list index out of range' not in str(e) and 'device disconnected' not in str(e):
                    self.node.get_logger().debug(f"Error leyendo velocidad motor {motor_id}: {e}")
                velocities.append(0.0)
        
        return velocities

    def close(self):
        """Cerrar todas las conexiones"""
        for manager in self.managers:
            if manager is not None:
                try:
                    manager.close()
                except:
                    pass
        
        self.managers.clear()
        self.usb_to_manager_map.clear()
        self.motor_to_usb_map.clear()
        self.detected_motors.clear()
        self.hardware_connected = False