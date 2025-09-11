#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from westwood_motor_interfaces.srv import SetMotorIdAndTarget, SetMotorIdAndTargetVelocity, SetMotorIdAndTargetCurrent, GetMotorPositions, GetMotorVelocities, GetAvailableMotors, GetMotorCurrents
from westwood_motor_interfaces.srv import SetGains, SetMode, SetTorqueEnable, SetGoalIq
import sys
import os
import sched
import threading
import time
import errno
from contextlib import contextmanager
import signal
import functools
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
import queue

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

def timeout_handler(signum, frame):
    """Signal handler for timeout"""
    raise TimeoutError("Operation timed out")

def safe_motor_operation(operation_func, timeout=0.1):
    """Execute motor operation with timeout protection for real-time systems"""
    result_queue = queue.Queue()
    exception_queue = queue.Queue()
    
    def worker():
        try:
            result = operation_func()
            result_queue.put(result)
        except Exception as e:
            exception_queue.put(e)
    
    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    thread.join(timeout)
    
    if thread.is_alive():
        # Operation timed out - this prevents hanging
        raise TimeoutError(f"Motor operation timed out after {timeout}s")
    
    # Check for exceptions
    if not exception_queue.empty():
        raise exception_queue.get()
    
    # Return result if available
    if not result_queue.empty():
        return result_queue.get()
    
    return None

class WestwoodMotorServer(Node):
    def __init__(self):
        super().__init__('westwood_motor_server')
        self.get_logger().info('Westwood Motor Server started - REALTIME MODE')
        
        # Initialize empty motor mapping for early service setup
        self.motor_to_usb_map = {}
        self.managers = []
        
        # Setup services early to ensure they are available
        self.setup_services()
        self.get_logger().info('✅ Servicios configurados temprano')
        
        # Parámetros configurables para múltiples USBs
        self.declare_parameter('usb_ports', ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3'])
        self.declare_parameter('baudrate', 8000000)
        self.declare_parameter('motor_ids_usb0', [])  # Motors detectados automáticamente en USB0
        self.declare_parameter('motor_ids_usb1', [])  # Motors detectados automáticamente en USB1
        self.declare_parameter('motor_ids_usb2', [])  # Motors detectados automáticamente en USB2
        self.declare_parameter('motor_ids_usb3', [])  # Motors detectados automáticamente en USB3
        self.declare_parameter('auto_detect', True)  # Detectar motores automáticamente
        self.declare_parameter('debug', False)
        
        # Real-time configuration parameters
        self.declare_parameter('realtime_priority', 50)
        self.declare_parameter('nice_value', -10)
        
        # Motor control default parameters
        self.declare_parameter('default_position_gains.p_gain_position', 5.0)
        self.declare_parameter('default_position_gains.i_gain_position', 0.0)
        self.declare_parameter('default_position_gains.d_gain_position', 0.2)
        self.declare_parameter('default_position_gains.iq_max', 3.0)
        
        self.declare_parameter('default_current_gains.p_gain_iq', 0.277)
        self.declare_parameter('default_current_gains.i_gain_iq', 0.061)
        self.declare_parameter('default_current_gains.d_gain_iq', 0.0)
        self.declare_parameter('default_current_gains.p_gain_id', 0.277)
        self.declare_parameter('default_current_gains.i_gain_id', 0.061)
        self.declare_parameter('default_current_gains.d_gain_id', 0.0)
        self.declare_parameter('default_current_gains.iq_max', 3.0)
        
        # Communication settings
        self.declare_parameter('ping_timeout_ms', 100)
        self.declare_parameter('max_motor_scan_range', 10)
        self.declare_parameter('communication_timeout_s', 2.0)  # Timeout for individual operations
        self.declare_parameter('max_retry_attempts', 2)  # Number of retries before giving up
        
        # Safety limits
        self.declare_parameter('max_position', 6.28)
        self.declare_parameter('min_position', -6.28)
        self.declare_parameter('max_velocity', 10.0)
        self.declare_parameter('max_current', 5.0)
        
        # Obtener parámetros
        self.usb_ports = self.get_parameter('usb_ports').value
        self.baudrate = self.get_parameter('baudrate').value
        # Obtener arrays de motor IDs con manejo de errores
        try:
            self.motor_ids_usb0 = self.get_parameter('motor_ids_usb0').value
        except:
            self.motor_ids_usb0 = []
        try:
            self.motor_ids_usb1 = self.get_parameter('motor_ids_usb1').value
        except:
            self.motor_ids_usb1 = []
        try:
            self.motor_ids_usb2 = self.get_parameter('motor_ids_usb2').value
        except:
            self.motor_ids_usb2 = []
        try:
            self.motor_ids_usb3 = self.get_parameter('motor_ids_usb3').value
        except:
            self.motor_ids_usb3 = []
        self.auto_detect = self.get_parameter('auto_detect').value
        self.debug = self.get_parameter('debug').value
        
        # Real-time configuration parameters
        self.realtime_priority = self.get_parameter('realtime_priority').value
        self.nice_value = self.get_parameter('nice_value').value
        
        # Motor control default parameters
        self.default_position_gains = {
            'p_gain_position': self.get_parameter('default_position_gains.p_gain_position').value,
            'i_gain_position': self.get_parameter('default_position_gains.i_gain_position').value,
            'd_gain_position': self.get_parameter('default_position_gains.d_gain_position').value,
            'iq_max': self.get_parameter('default_position_gains.iq_max').value,
        }
        
        self.default_current_gains = {
            'p_gain_iq': self.get_parameter('default_current_gains.p_gain_iq').value,
            'i_gain_iq': self.get_parameter('default_current_gains.i_gain_iq').value,
            'd_gain_iq': self.get_parameter('default_current_gains.d_gain_iq').value,
            'p_gain_id': self.get_parameter('default_current_gains.p_gain_id').value,
            'i_gain_id': self.get_parameter('default_current_gains.i_gain_id').value,
            'd_gain_id': self.get_parameter('default_current_gains.d_gain_id').value,
            'iq_max': self.get_parameter('default_current_gains.iq_max').value,
        }
        
        # Communication settings
        self.ping_timeout_ms = self.get_parameter('ping_timeout_ms').value
        self.max_motor_scan_range = self.get_parameter('max_motor_scan_range').value
        self.communication_timeout = self.get_parameter('communication_timeout_s').value
        self.max_retry_attempts = self.get_parameter('max_retry_attempts').value
        
        # Safety limits
        self.max_position = self.get_parameter('max_position').value
        self.min_position = self.get_parameter('min_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_current = self.get_parameter('max_current').value
        
        # Configure realtime scheduling now that parameters are loaded
        self.setup_realtime_scheduling()
        
        # Log loaded parameters for debugging
        self.get_logger().info(f'📋 Parámetros cargados:')
        self.get_logger().info(f'   USB ports: {self.usb_ports}')
        self.get_logger().info(f'   Baudrate: {self.baudrate}')
        self.get_logger().info(f'   Auto detect: {self.auto_detect}')
        self.get_logger().info(f'   Debug: {self.debug}')
        self.get_logger().info(f'   Position gains: {self.default_position_gains}')
        self.get_logger().info(f'   Current gains: {self.default_current_gains}')
        
        # Crear mapeo de motor ID a USB y manager
        self.motor_to_usb_map = {}
        self.usb_to_manager_map = {}
        self.usb_to_lock_map = {}
        self.detected_motors = set()  # Cache de motores detectados
        
        # Connection monitoring and recovery
        self.connection_states = {}  # {usb_index: {'healthy': bool, 'last_error': time, 'error_count': int}}
        self.max_consecutive_errors = 3
        self.error_cooldown_time = 5.0  # seconds
        self.connection_check_timer = None
        
        # Real-time communication settings
        self.fast_fail_threshold = 3  # Quick fail after this many consecutive errors
        self.fast_recovery_time = 1.0  # Quick recovery attempt after 1 second
        
        # Lightweight connection health tracking (atomic operations)
        self.connection_health = {}  # {usb_index: bool} - simple healthy/unhealthy
        
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
                self.usb_to_lock_map[i] = threading.Lock()
                
                # Initialize connection state tracking
                self.connection_states[i] = {
                    'healthy': True,
                    'last_error': 0,
                    'error_count': 0,
                    'port': port
                }
                
                # Initialize simple health tracking for real-time
                self.connection_health[i] = True
                
                self.get_logger().info(f'✅ Manager de PyBear inicializado correctamente para {port}')
                self.get_logger().info(f'Puerto: {port}, Baudrate: {self.baudrate}')
                        
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
            self.get_logger().error('❌ No se pudo inicializar ningún manager USB')
            raise RuntimeError('No se pudo inicializar ningún manager USB')
        else:
            self.get_logger().info(f'✅ Sistema listo con {working_managers} USB(s) funcional(es)')
        
        # Detectar motores automáticamente si está habilitado
        if self.auto_detect:
            self.detect_and_map_motors()
        else:
            # Usar configuración manual si auto_detect está deshabilitado
            self.manual_motor_mapping()
        
        # Mostrar resumen del mapeo
        if self.motor_to_usb_map:
            summary = {}
            for usb_idx in set(m['usb_index'] for m in self.motor_to_usb_map.values()):
                ids = sorted([gid for gid, m in self.motor_to_usb_map.items() if m['usb_index'] == usb_idx])
                summary[f'USB{usb_idx}'] = ids
            self.get_logger().info(f'🗺️ Resumen: {summary}')
        else:
            self.get_logger().warning('⚠️ No hay motores mapeados')
        
        # Continuar con la inicialización de servicios
        try:
            self.setup_services()
        except Exception as e:
            self.get_logger().error(f'❌ Error al configurar servicios: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        # Start connection monitoring timer
        self.start_connection_monitoring()
    
    def setup_realtime_scheduling(self):
        """Configure realtime scheduling for the server process"""
        try:
            import os
            # Set high priority to the current process
            # SCHED_FIFO with configurable priority (range is 1-99, where 99 is highest)
            pid = os.getpid()
            # Try to set realtime scheduling - requires sudo privileges
            try:
                os.system(f'chrt -f -p {self.realtime_priority} {pid}')
                self.get_logger().info(f'🚀 Realtime scheduling configured for PID {pid} with FIFO priority {self.realtime_priority}')
            except Exception as e:
                self.get_logger().warning(f'⚠️ Could not set realtime scheduling (requires sudo): {str(e)}')
                self.get_logger().info('💡 Running with normal priority - consider running with sudo for realtime performance')
                
            # Set process nice value for higher priority (lower nice = higher priority)
            try:
                os.nice(self.nice_value)  # Use configurable nice value
                self.get_logger().info(f'✅ Process priority increased with nice {self.nice_value}')
            except Exception:
                try:
                    os.nice(-5)  # Try with less aggressive setting
                    self.get_logger().info('✅ Process priority increased with nice -5')
                except Exception:
                    self.get_logger().warning('⚠️ Could not increase process priority')
                    
        except Exception as e:
            self.get_logger().warning(f'⚠️ Realtime setup failed: {str(e)}')
            self.get_logger().info('📝 Continuing with normal scheduling')
    
    def detect_and_map_motors(self):
        """Detectar automáticamente motores conectados y crear mapeo inteligente"""
        self.get_logger().info('🔍 Detectando motores...')
        
        # Limpiar el cache de motores detectados
        self.detected_motors.clear()
        
        # Primero detectar todos los motores en todos los USBs
        all_detections = {}  # {usb_index: [motor_ids]}
        
        for usb_index, manager in enumerate(self.managers):
            if manager is None:
                continue
            
            lock = self.usb_to_lock_map.get(usb_index)
            if not lock:
                self.get_logger().warning(f"No se encontró lock para USB{usb_index}, saltando detección.")
                continue

            detected_motors = []

            try:
                with lock:
                    for motor_id in range(1, self.max_motor_scan_range + 1):  # Buscar IDs configurables
                        try:
                            result = manager.ping(motor_id)
                            if result and len(result) > 0:
                                if result[0] is not None:
                                    detected_motors.append(motor_id)
                        except Exception:
                            continue
            except Exception as e:
                self.get_logger().error(f"Error durante la detección en USB{usb_index}: {e}")
                continue

            if detected_motors:
                all_detections[usb_index] = detected_motors
                self.get_logger().info(f'USB{usb_index}: Detectados {len(detected_motors)} motor(es) con IDs locales: {detected_motors}')
        
        # Ahora asignar IDs globales, remapeando solo cuando hay conflictos
        used_global_ids = set()
        next_available_id = 1
        
        # Procesar USBs en orden (0, 1, 2, 3)
        for usb_index in sorted(all_detections.keys()):
            detected_motors = all_detections[usb_index]
            manager = self.managers[usb_index]
            
            for local_id in detected_motors:
                # Si el ID local no está en uso, usarlo como global
                if local_id not in used_global_ids:
                    global_id = local_id
                    used_global_ids.add(global_id)
                else:
                    # Si hay conflicto, buscar el siguiente ID disponible
                    while next_available_id in used_global_ids:
                        next_available_id += 1
                    global_id = next_available_id
                    used_global_ids.add(global_id)
                    next_available_id += 1
                    self.get_logger().info(f'  Conflicto detectado: Motor USB{usb_index} ID local {local_id} remapeado a ID global {global_id}')
                
                self.motor_to_usb_map[global_id] = {
                    'usb_index': usb_index,
                    'local_id': local_id,
                    'manager': manager
                }
                # Agregar al cache de motores detectados
                self.detected_motors.add(global_id)
        
        # Mostrar resumen final
        if self.motor_to_usb_map:
            total = len(self.motor_to_usb_map)
            self.get_logger().info(f'✅ Total: {total} motor(es) detectado(s)')
            # Mostrar mapeo por USB
            for usb_idx in sorted(set(m['usb_index'] for m in self.motor_to_usb_map.values())):
                mappings = [(gid, m['local_id']) for gid, m in self.motor_to_usb_map.items() if m['usb_index'] == usb_idx]
                mappings.sort()
                self.get_logger().info(f'  USB{usb_idx}: {[f"ID global {g} -> local {l}" for g, l in mappings]}')
        else:
            self.get_logger().warning('⚠️ No se detectaron motores en ningún USB')
    
    def manual_motor_mapping(self):
        """Usar configuración manual de motores desde parámetros"""
        self.get_logger().info('📝 Usando configuración manual de motores...')
        
        # Limpiar el cache
        self.detected_motors.clear()
        
        motor_configs = [
            (0, self.motor_ids_usb0),
            (1, self.motor_ids_usb1),
            (2, self.motor_ids_usb2),
            (3, self.motor_ids_usb3)
        ]
        
        total_motors = 0
        for usb_index, motor_ids in motor_configs:
            if usb_index < len(self.managers) and self.managers[usb_index] is not None and motor_ids:
                for i, global_id in enumerate(motor_ids):
                    # Los IDs locales empiezan desde 1
                    local_id = (i % 9) + 1  # Limitar a IDs 1-9
                    self.motor_to_usb_map[global_id] = {
                        'usb_index': usb_index,
                        'local_id': local_id,
                        'manager': self.managers[usb_index]
                    }
                    # Agregar al cache de motores detectados
                    self.detected_motors.add(global_id)
                    total_motors += 1
                self.get_logger().info(f'USB{usb_index}: Configurados {len(motor_ids)} motores con IDs globales {motor_ids}')
        
        if total_motors > 0:
            self.get_logger().info(f'✅ Total: {total_motors} motor(es) configurado(s) manualmente')
            self.get_logger().info(f'🗺️ Cache de motores: {sorted(list(self.detected_motors))}')
        else:
            self.get_logger().warning('⚠️ No hay motores configurados manualmente')
    
    def get_manager_for_motor(self, motor_id):
        """Obtener el manager y el lock correcto para un motor ID dado"""
        if motor_id in self.motor_to_usb_map:
            motor_info = self.motor_to_usb_map[motor_id]
            usb_index = motor_info['usb_index']
            local_id = motor_info['local_id']
            manager = motor_info['manager']
            lock = self.usb_to_lock_map.get(usb_index)
            return manager, local_id, lock
        return None, motor_id, None
    
    def ping_motor(self, motor_id):
        """Fast motor availability check optimized for real-time"""
        # For real-time operation, rely primarily on cache and connection health
        if motor_id in self.detected_motors:
            motor_info = self.motor_to_usb_map.get(motor_id)
            if motor_info and self.connection_health.get(motor_info['usb_index'], False):
                return True
        
        # If not cached or connection unhealthy, try direct ping
        return self._ping_motor_direct(motor_id)
    
    def _ping_motor_direct(self, motor_id):
        """Direct ping to motor with safe communication"""
        manager, local_id, lock = self.get_manager_for_motor(motor_id)
        if manager is None or lock is None:
            return False
        
        try:
            # Get USB index for this motor
            motor_info = self.motor_to_usb_map.get(motor_id)
            if not motor_info:
                return False
            
            usb_index = motor_info['usb_index']
            
            with self.safe_communication(usb_index, motor_id, "ping"):
                with lock:
                    result = safe_motor_operation(
                        lambda: manager.ping(local_id),
                        timeout=0.05  # 50ms timeout for ping
                    )
                if result and len(result) > 0:
                    if result[0] is not None:
                        # Si el ping es exitoso, agregar al cache
                        self.detected_motors.add(motor_id)
                        return True
            return False
        except Exception as e:
            # Remove from cache if ping fails
            self.detected_motors.discard(motor_id)
            return False
    
    def get_all_motor_ids(self):
        """Obtener todos los IDs de motores configurados"""
        return list(self.motor_to_usb_map.keys())
    
    @contextmanager
    def safe_communication(self, usb_index, motor_id, operation_name="operation"):
        """Lightweight context manager for real-time communication with minimal overhead"""
        try:
            # Quick health check - fail fast if connection is known bad
            if not self.connection_health.get(usb_index, False):
                raise RuntimeError(f"USB{usb_index} connection unhealthy")
            
            yield
            # Success - ensure connection is marked healthy
            self.connection_health[usb_index] = True
            if usb_index in self.connection_states:
                self.connection_states[usb_index]['error_count'] = 0
            
        except Exception as e:
            # Fast error handling for real-time
            error_str = str(e).lower()
            is_critical = any(pattern in error_str for pattern in [
                'bad file descriptor', 'errno 9', 'device disconnected', 'no such device'
            ])
            
            if is_critical:
                # Mark connection as unhealthy immediately
                self.connection_health[usb_index] = False
                self.get_logger().error(f"❌ USB{usb_index} marked unhealthy: {operation_name} on motor {motor_id}: {str(e)}")
                
                # Quick recovery attempt in background (non-blocking)
                if hasattr(self, '_last_recovery_attempt'):
                    if time.time() - self._last_recovery_attempt.get(usb_index, 0) > self.fast_recovery_time:
                        self._last_recovery_attempt[usb_index] = time.time()
                        # Schedule recovery attempt (don't block current operation)
                        threading.Thread(target=self._background_recovery, args=(usb_index,), daemon=True).start()
                else:
                    self._last_recovery_attempt = {usb_index: time.time()}
            
            # Update error count for less critical errors
            if usb_index in self.connection_states:
                self.connection_states[usb_index]['error_count'] += 1
                if self.connection_states[usb_index]['error_count'] >= self.fast_fail_threshold:
                    self.connection_health[usb_index] = False
            
            raise e
    
    def _background_recovery(self, usb_index):
        """Background recovery method that doesn't block real-time operations"""
        try:
            if self.attempt_connection_recovery(usb_index):
                self.connection_health[usb_index] = True
                self.get_logger().info(f"✅ Background recovery successful for USB{usb_index}")
        except Exception as e:
            self.get_logger().warning(f"⚠️ Background recovery failed for USB{usb_index}: {str(e)}")
    
    def is_connection_healthy(self, usb_index):
        """Fast connection health check for real-time operation"""
        return self.connection_health.get(usb_index, False)
    
    def mark_connection_success(self, usb_index):
        """Mark a successful communication, resetting error counters"""
        if usb_index in self.connection_states:
            self.connection_states[usb_index]['error_count'] = 0
            self.connection_states[usb_index]['healthy'] = True
    
    def handle_communication_error(self, usb_index, motor_id, error, operation_name):
        """Handle communication errors with appropriate recovery strategies"""
        current_time = time.time()
        
        # Update connection state
        if usb_index in self.connection_states:
            state = self.connection_states[usb_index]
            state['error_count'] += 1
            state['last_error'] = current_time
            
            # Check if this is a critical error
            is_critical = self.is_critical_error(error)
            
            if is_critical or state['error_count'] >= self.max_consecutive_errors:
                state['healthy'] = False
                self.get_logger().error(
                    f"❌ USB{usb_index} marked unhealthy after {state['error_count']} errors. "
                    f"Last error during {operation_name} on motor {motor_id}: {str(error)}"
                )
                
                # Attempt immediate recovery for critical errors
                if is_critical:
                    self.get_logger().warning(f"🔧 Attempting immediate recovery for critical error on USB{usb_index}")
                    return self.attempt_connection_recovery(usb_index)
            else:
                self.get_logger().warning(
                    f"⚠️ Communication error on USB{usb_index} motor {motor_id} during {operation_name}: {str(error)} "
                    f"(error {state['error_count']}/{self.max_consecutive_errors})"
                )
        
        return False  # Error not handled, let it propagate
    
    def is_critical_error(self, error):
        """Determine if an error is critical and requires immediate recovery"""
        error_str = str(error).lower()
        critical_patterns = [
            'bad file descriptor',
            'errno 9',
            'device disconnected',
            'no such device',
            'broken pipe',
            'connection reset'
        ]
        return any(pattern in error_str for pattern in critical_patterns)
    
    def attempt_connection_recovery(self, usb_index):
        """Attempt to recover a failed USB connection"""
        if usb_index not in self.connection_states:
            return False
        
        state = self.connection_states[usb_index]
        port = state['port']
        
        self.get_logger().info(f"🔧 Attempting recovery for USB{usb_index} ({port})")
        
        try:
            # Close existing manager if it exists
            if usb_index < len(self.managers) and self.managers[usb_index] is not None:
                try:
                    self.managers[usb_index].close()
                except:
                    pass  # Ignore errors during cleanup
            
            # Wait briefly before reconnection
            time.sleep(0.5)
            
            # Check if port still exists
            if not os.path.exists(port):
                self.get_logger().error(f"❌ Recovery failed: Port {port} no longer exists")
                return False
            
            # Create new manager
            new_manager = Manager(
                port=port,
                baudrate=self.baudrate,
                debug=self.debug
            )
            
            # Test communication
            test_ping = new_manager.ping(1)
            
            # Update manager references
            self.managers[usb_index] = new_manager
            self.usb_to_manager_map[usb_index] = new_manager
            
            # Reset connection state
            state['healthy'] = True
            state['error_count'] = 0
            state['last_error'] = 0
            
            self.get_logger().info(f"✅ Successfully recovered USB{usb_index} connection")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Recovery failed for USB{usb_index}: {str(e)}")
            return False
    
    def start_connection_monitoring(self):
        """Start lightweight connection health monitoring for real-time"""
        self.connection_check_timer = self.create_timer(
            5.0,  # More frequent checks for real-time
            self.check_connection_health
        )
        self.get_logger().info("🔍 Real-time connection monitoring started")
    
    def check_connection_health(self):
        """Lightweight periodic health check for real-time operation"""
        for usb_index in self.connection_health:
            if not self.connection_health[usb_index]:
                # Check if enough time has passed to attempt recovery
                current_time = time.time()
                if hasattr(self, '_last_recovery_attempt'):
                    last_attempt = self._last_recovery_attempt.get(usb_index, 0)
                    if current_time - last_attempt > self.fast_recovery_time * 2:  # Try recovery less frequently
                        self._last_recovery_attempt[usb_index] = current_time
                        # Background recovery attempt
                        threading.Thread(target=self._background_recovery, args=(usb_index,), daemon=True).start()
    
    def setup_services(self):
        """Configurar todos los servicios ROS2"""
        # Añadir el servicio para manejar múltiples motores con posiciones
        try:
            self.set_motor_id_and_target_service = self.create_service(
                SetMotorIdAndTarget,
                'westwood_motor/set_motor_id_and_target',
                self.handle_motor_ids_and_target
            )
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio multi-motor con arrays: {str(e)}')

        #Añadir servicio para manejar múltiples motores con velocidades
        try:
            self.set_motor_id_and_target_velocity_service = self.create_service(
                SetMotorIdAndTargetVelocity,
                'westwood_motor/set_motor_id_and_target_velocity',
                self.handle_motor_ids_and_target_velocity
            )
        except Exception as e:
            self.get_logger().error(f'Error al registrar servicio multi-motor con arrays de velocidad: {str(e)}')
        
        # Añadir servicio para obtener posiciones actuales de motores
        self.get_motor_positions_service = self.create_service(
            GetMotorPositions,
            'westwood_motor/get_motor_positions',
            self.handle_get_motor_positions
        )

        # Añadir servicio para obtener velocidades actuales de motores
        self.get_motor_velocities_service = self.create_service(
            GetMotorVelocities,
            'westwood_motor/get_motor_velocities',
            self.handle_get_motor_velocities
        )

        # Añadir servicio para obtener corrientes actuales de motores
        self.get_motor_currents_service = self.create_service(
            GetMotorCurrents,
            'westwood_motor/get_motor_currents',
            self.handle_get_motor_currents
        )
        
        # Añadir servicio para obtener IDs de motores disponibles
        self.get_available_motors_service = self.create_service(
            GetAvailableMotors,
            'westwood_motor/get_available_motors',
            self.handle_get_available_motors
        )
        
        # Añadir servicio para configurar ganancias de control de posición
        self.set_position_gains_service = self.create_service(
            SetGains,
            'westwood_motor/set_position_gains',
            self.handle_set_position_gains
        )
        
        # Añadir servicio para configurar ganancias de control de corriente
        self.set_current_gains_service = self.create_service(
            SetGains,
            'westwood_motor/set_current_gains',
            self.handle_set_current_gains
        )
        
        # Añadir servicio para configurar el modo de operación
        self.set_mode_service = self.create_service(
            SetMode,
            'westwood_motor/set_mode',
            self.handle_set_mode
        )
        
        # Añadir servicio para habilitar/deshabilitar el torque
        self.set_torque_enable_service = self.create_service(
            SetTorqueEnable,
            'westwood_motor/set_torque_enable',
            self.handle_set_torque_enable
        )
        
        # Añadir servicio para establecer corriente iq objetivo
        self.set_goal_iq_service = self.create_service(
            SetGoalIq,
            'westwood_motor/set_goal_iq',
            self.handle_set_goal_iq
        )

        self.set_motor_id_and_target_current_service = self.create_service(
            SetMotorIdAndTargetCurrent,
            'westwood_motor/set_motor_id_and_target_current',
            self.handle_set_motor_id_and_target_current
        )
        
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
            
            successful_motors = []
            failed_motor_ids = []
            previous_positions = []
            
            self.get_logger().info(f'🎯 Iniciando control de motores: {request.motor_ids} hacia posiciones: {request.target_positions}')
            
            # Verificar conexión de cada motor usando el cache
            for motor_id in request.motor_ids:
                try:
                    # Usar el método ping_motor que ahora usa cache
                    if self.ping_motor(motor_id):
                        self.get_logger().info(f'✅ Motor {motor_id} verificado y listo')
                    else:
                        failed_motor_ids.append(motor_id)
                        self.get_logger().warning(f'❌ Motor {motor_id} no está disponible')
                except Exception as e:
                    self.get_logger().error(f'❌ Error al verificar motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
            
            # Obtener motores disponibles (excluyendo los que ya fallaron)
            available_motors = [m for m in request.motor_ids if m not in failed_motor_ids]
            
            if available_motors:
                self.get_logger().info(f'🚀 Motores disponibles para control: {available_motors}')
            else:
                self.get_logger().warning(f'⚠️  Ningún motor respondió. Fallidos: {failed_motor_ids}')
                
            # Para cada motor disponible, configurar y mover
            for motor_id in available_motors:
                # Obtener el índice del motor en la lista original
                idx = request.motor_ids.index(motor_id)
                
                # Obtener el manager correcto para este motor
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    self.get_logger().error(f'No se encontró manager o lock para motor {motor_id}')
                    failed_motor_ids.append(motor_id)
                    previous_positions.append(0.0)
                    continue
                
                try:
                    # Get USB index for this motor
                    motor_info = self.motor_to_usb_map[motor_id]
                    usb_index = motor_info['usb_index']
                    
                    with self.safe_communication(usb_index, motor_id, "position_control"):
                        with lock:
                            # Leer posición actual con timeout protection
                            current_position_result = safe_motor_operation(
                                lambda: manager.get_present_position(local_id),
                                timeout=0.05  # 50ms timeout for real-time
                            )
                            if current_position_result and len(current_position_result) > 0:
                                current_position = float(current_position_result[0][0][0])
                                target_position = request.target_positions[idx]
                                
                                # Mientras idx < len(previous_positions), significa que ya hay posiciones guardadas
                                while len(previous_positions) <= idx:
                                    previous_positions.append(0.0)
                                previous_positions[idx] = current_position
                                
                                self.get_logger().info(f'🔧 Motor {motor_id}: posición actual {current_position:.3f} → objetivo {target_position:.3f}')
                                
                                # Configuración rápida con timeout protection
                                def configure_motor():
                                    # Configurar PID para el control de corriente
                                    manager.set_p_gain_iq((local_id, self.default_current_gains['p_gain_iq']))
                                    manager.set_i_gain_iq((local_id, self.default_current_gains['i_gain_iq']))
                                    manager.set_d_gain_iq((local_id, self.default_current_gains['d_gain_iq']))
                                    manager.set_p_gain_id((local_id, self.default_current_gains['p_gain_id']))
                                    manager.set_i_gain_id((local_id, self.default_current_gains['i_gain_id']))
                                    manager.set_d_gain_id((local_id, self.default_current_gains['d_gain_id']))

                                    # PID position mode
                                    manager.set_p_gain_position((local_id, self.default_position_gains['p_gain_position']))
                                    manager.set_i_gain_position((local_id, self.default_position_gains['i_gain_position']))
                                    manager.set_d_gain_position((local_id, self.default_position_gains['d_gain_position']))

                                    # Configurar modo y límites
                                    manager.set_mode((local_id, 2))  # Modo posición
                                    manager.set_limit_iq_max((local_id, self.default_position_gains['iq_max']))
                                    
                                    # Habilitar torque y mover
                                    manager.set_torque_enable((local_id, 1))
                                    manager.set_goal_position((local_id, target_position))
                                
                                # Execute configuration with timeout
                                safe_motor_operation(configure_motor, timeout=0.1)
                                
                                successful_motors.append(motor_id)
                                self.get_logger().info(f'✅ Motor {motor_id} configurado y moviendo')
                            else:
                                while len(previous_positions) <= idx:
                                    previous_positions.append(0.0)
                                failed_motor_ids.append(motor_id)
                                self.get_logger().warning(f'❌ No se pudo leer la posición actual del motor {motor_id}')

                except Exception as e:
                    self.get_logger().error(f'❌ Error al configurar/mover motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
                    while len(previous_positions) <= idx:
                        previous_positions.append(0.0)
            
            # Asegurar que previous_positions tiene la misma longitud que request.motor_ids
            while len(previous_positions) < len(request.motor_ids):
                previous_positions.append(0.0)
            
            # Preparar respuesta detallada
            total_requested = len(request.motor_ids)
            total_successful = len(successful_motors)
            total_failed = len(set(failed_motor_ids))  # Eliminar duplicados
            
            self.get_logger().info(f"📊 Estadísticas finales: solicitados={total_requested}, exitosos={total_successful}, fallidos={total_failed}")
            self.get_logger().info(f"📊 Motores exitosos: {successful_motors}")
            self.get_logger().info(f"📊 Motores fallidos: {list(set(failed_motor_ids))}")
            
            if total_successful > 0:
                response.success = True
                if total_failed == 0:
                    response.message = f"✅ Control exitoso de {total_successful} motores: {successful_motors}"
                    self.get_logger().info(f"🎉 Control completado exitosamente para todos los motores: {successful_motors}")
                else:
                    response.message = f"⚠️  Control parcial: {total_successful} motores exitosos {successful_motors}, {total_failed} fallaron {list(set(failed_motor_ids))}"
                    self.get_logger().warning(f"⚠️  Control parcial: exitosos {successful_motors}, fallidos {list(set(failed_motor_ids))}")
            else:
                response.success = False
                response.message = f"❌ No se pudieron controlar ningún motor. Fallidos: {list(set(failed_motor_ids))}"
                self.get_logger().error(f"❌ Control fallido para todos los motores solicitados: {request.motor_ids}")
            
            response.previous_positions = previous_positions
            
            # Debug final de la respuesta antes de enviarla
            self.get_logger().info(f"🔍 RESPUESTA FINAL: success={response.success}, message='{response.message}', previous_positions={response.previous_positions}")
            
            return response
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio multi-motor: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {str(e)}"
            response.previous_positions = []
            return response
        
    # Función principal para manejar IDs de motores y sus velocidades objetivo 
    def handle_motor_ids_and_target_velocity(self, request, response):
        """Callback para controlar múltiples motores con velocidades objetivo individuales"""
        try:
            # Si no hay motores especificados, no hay nada que hacer
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No se especificaron IDs de motores"
                return response
            
            # Si la cantidad de motores no coincide con la cantidad de velocidades
            if len(request.motor_ids) != len(request.target_velocities):
                response.success = False
                response.message = "La cantidad de IDs de motores no coincide con la cantidad de velocidades objetivo"
                return response
            
            successful_motors = []
            failed_motor_ids = []
            previous_velocities = []
            
            self.get_logger().info(f'🎯 Iniciando control de motores: {request.motor_ids} hacia velocidades: {request.target_velocities}')
            
            # Verificar conexión de cada motor usando el cache
            for motor_id in request.motor_ids:
                try:
                    # Usar el método ping_motor que ahora usa cache
                    if self.ping_motor(motor_id):
                        self.get_logger().info(f'✅ Motor {motor_id} verificado y listo')
                    else:
                        failed_motor_ids.append(motor_id)
                        self.get_logger().warning(f'❌ Motor {motor_id} no está disponible')
                except Exception as e:
                    self.get_logger().error(f'❌ Error al verificar motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
            
            # Obtener motores disponibles (excluyendo los que ya fallaron)
            available_motors = [m for m in request.motor_ids if m not in failed_motor_ids]
            
            if available_motors:
                self.get_logger().info(f'🚀 Motores disponibles para control: {available_motors}')
            else:
                self.get_logger().warning(f'⚠️  Ningún motor respondió. Fallidos: {failed_motor_ids}')
                
            # Para cada motor disponible, configurar y mover
            for motor_id in available_motors:
                # Obtener el índice del motor en la lista original
                idx = request.motor_ids.index(motor_id)
                
                # Obtener el manager correcto para este motor
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    self.get_logger().error(f'No se encontró para motor {motor_id}')
                    failed_motor_ids.append(motor_id)
                    previous_velocities.append(0.0)
                    continue

                try:
                    with lock:
                        # Leer velocidad actual y guardarla
                        current_velocity_result = manager.get_present_velocity(local_id)
                        if current_velocity_result and len(current_velocity_result) > 0:
                            current_velocity = float(current_velocity_result[0][0][0])
                            target_velocity = request.target_velocities[idx]
                            
                            # Mientras idx < len(previous_velocities), significa que ya hay velocidades guardadas
                            while len(previous_velocities) <= idx:
                                previous_velocities.append(0.0)
                            previous_velocities[idx] = current_velocity
                            
                            self.get_logger().info(f'🔧 Motor {motor_id}: velocidad actual {current_velocity:.3f} → objetivo {target_velocity:.3f}')
                            
                            # Configurar PID para el control de corriente (usando parámetros configurables)
                            manager.set_p_gain_iq((local_id, self.default_current_gains['p_gain_iq']))
                            manager.set_i_gain_iq((local_id, self.default_current_gains['i_gain_iq']))
                            manager.set_d_gain_iq((local_id, self.default_current_gains['d_gain_iq']))
                            manager.set_p_gain_id((local_id, self.default_current_gains['p_gain_id']))
                            manager.set_i_gain_id((local_id, self.default_current_gains['i_gain_id']))
                            manager.set_d_gain_id((local_id, self.default_current_gains['d_gain_id']))

                            # Configurar modo y límites
                            manager.set_mode((local_id, 1))  # Modo velocidad
                            manager.set_limit_iq_max((local_id, self.default_current_gains['iq_max']))  # Límite de corriente

                            # Habilitar torque y mover
                            manager.set_torque_enable((local_id, 1))
                            manager.set_goal_velocity((local_id, target_velocity))
                            
                            successful_motors.append(motor_id)
                            self.get_logger().info(f'✅ Motor {motor_id} configurado y moviendo')
                        else:
                            while len(previous_velocities) <= idx:
                                previous_velocities.append(0.0)
                            failed_motor_ids.append(motor_id)
                            self.get_logger().warning(f'❌ No se pudo leer la velocidad actual del motor {motor_id}')
                            
                except Exception as e:
                    self.get_logger().error(f'❌ Error al configurar/mover motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
                    while len(previous_velocities) <= idx:
                        previous_velocities.append(0.0)

            # Asegurar que previous_velocities tiene la misma longitud que request.motor_ids
            while len(previous_velocities) < len(request.motor_ids):
                previous_velocities.append(0.0)

            # Preparar respuesta detallada
            total_requested = len(request.motor_ids)
            total_successful = len(successful_motors)
            total_failed = len(set(failed_motor_ids))
            # Eliminar duplicados

            self.get_logger().info(f"📊 Estadísticas finales: solicitados={total_requested}, exitosos={total_successful}, fallidos={total_failed}")
            self.get_logger().info(f"📊 Motores exitosos: {successful_motors}")
            self.get_logger().info(f"📊 Motores fallidos: {list(set(failed_motor_ids))}")

            if total_successful > 0:
                response.success = True
                if total_failed == 0:
                    response.message = f"✅ Control exitoso de {total_successful} motores: {successful_motors}"
                    self.get_logger().info(f"🎉 Control completado exitosamente para todos los motores: {successful_motors}")
                else:
                    response.message = f"⚠️  Control parcial: {total_successful} motores exitosos {successful_motors}, {total_failed} fallaron {list(set(failed_motor_ids))}"
                    self.get_logger().warning(f"⚠️  Control parcial: exitosos {successful_motors}, fallidos {list(set(failed_motor_ids))}")
            else:
                response.success = False
                response.message = f"❌ No se pudieron controlar ningún motor. Fallidos: {list(set(failed_motor_ids))}"
                self.get_logger().error(f"❌ Control fallido para todos los motores solicitados: {request.motor_ids}")
            
            response.previous_velocities = previous_velocities

            # Debug final de la respuesta antes de enviarla
            self.get_logger().info(f"🔍 RESPUESTA FINAL: success={response.success},message='{response.message}', previous_velocities={response.previous_velocities}")

            return response

        except Exception as e:
            import traceback
            self.get_logger().error(f'Error en servicio multi-motor de velocidad: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {str(e)}"
            response.previous_velocities = []
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
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    failed_motor_ids.append(motor_id)
                    positions.append(0.0)
                    self.get_logger().warning(f'No se encontró manager o lock para motor {motor_id}')
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        
                        # Get USB index for this motor
                        motor_info = self.motor_to_usb_map[motor_id]
                        usb_index = motor_info['usb_index']
                        
                        with self.safe_communication(usb_index, motor_id, "position_read"):
                            with lock:
                                # Obtener posición actual con timeout
                                position_result = safe_motor_operation(
                                    lambda: manager.get_present_position(local_id),
                                    timeout=0.02  # 20ms timeout for position reading
                                )
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
            
            # Si no pudimos obtener posiciones reales, es un error
            if not positions:
                response.success = False
                response.message = "No se pudieron leer las posiciones de los motores"
                response.positions = []
                return response
            
            # Asegurarse de que tenemos una posición para cada motor solicitado
            while len(positions) < len(motor_ids):
                positions.append(0.0)
            
            # Preparar respuesta
            response.success = True
            if connected_motors:
                response.message = f"Posiciones leídas para {len(connected_motors)} motores"
            else:
                response.message = "No se pudieron leer posiciones"
            response.positions = positions
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error en servicio de lectura de posiciones: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            response.positions = []
            return response
        
    # Función para obtener las velocidades actuales de los motores
    def handle_get_motor_velocities(self, request, response):
        """Callback para obtener las velocidades actuales de los motores especificados"""
        try:
            # Si no hay motores especificados, utilizar todos los motores configurados
            motor_ids = request.motor_ids
            if not motor_ids or len(motor_ids) == 0:
                motor_ids = self.get_all_motor_ids()
                
            velocities = []
            connected_motors = []
            failed_motor_ids = []
            
            # Intentar obtener velocidades reales
            for motor_id in motor_ids:
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    failed_motor_ids.append(motor_id)
                    velocities.append(0.0)
                    self.get_logger().warning(f'No se encontró manager o lock para motor {motor_id}')
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        with lock:
                            # Obtener velocidad actual del motor
                            velocity_result = manager.get_present_velocity(local_id)
                        if velocity_result and len(velocity_result) > 0:
                            current_velocity = float(velocity_result[0][0][0])
                            velocities.append(current_velocity)
                            self.get_logger().info(f'Motor {motor_id} (local {local_id}): velocidad actual {current_velocity}')
                        else:
                            velocities.append(0.0)  # Valor por defecto si no se pudo leer
                            failed_motor_ids.append(motor_id)
                            self.get_logger().warning(f'No se pudo leer la velocidad del motor {motor_id}')
                    else:
                        failed_motor_ids.append(motor_id)
                        velocities.append(0.0)  # Valor por defecto para motores desconectados
                        self.get_logger().warning(f'Motor {motor_id} no responde')
                except Exception as e:
                    self.get_logger().error(f'Error al leer velocidad del motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
                    velocities.append(0.0)
            
            # Si no pudimos obtener velocidades reales, es un error
            if not velocities:
                response.success = False
                response.message = "No se pudieron leer las velocidades de los motores"
                response.velocities = []
                return response
            
            # Asegurarse de que tenemos una velocidad para cada motor solicitado
            while len(velocities) < len(motor_ids):
                velocities.append(0.0)

            # Preparar respuesta
            response.success = True
            if connected_motors:
                response.message = f"Velocidades leídas para {len(connected_motors)} motores"
            else:
                response.message = "No se pudieron leer velocidades"
            response.velocities = velocities
            return response
        
        except Exception as e:
            self.get_logger().error(f'Error en servicio de lectura de velocidades: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            response.velocities = []
            return response

    # Función para obtener las IDs de los motores disponibles
    def handle_get_available_motors(self, request, response):
        """Callback para obtener las IDs de los motores disponibles actualmente"""
        try:
            available_motors = []
            
            # Verificar si hay managers inicializados
            if not self.managers or all(m is None for m in self.managers):
                self.get_logger().error('Error: No hay managers inicializados')
                response.success = False
                response.message = "No hay managers USB inicializados"
                response.motor_ids = []
                return response
            
            # Obtener solo los motores que realmente están conectados
            try:
                configured_motor_ids = self.get_all_motor_ids()
                
                # Solo reportar los motores que están en el mapeo (ya verificados durante la detección)
                for motor_id in configured_motor_ids:
                    available_motors.append(motor_id)
            except Exception as e:
                self.get_logger().error(f'Error al escanear motores: {str(e)}')
            
            # Si no se encontraron motores, es un error
            if not available_motors:
                response.success = False
                response.message = "No se detectaron motores físicos"
                response.motor_ids = []
                self.get_logger().error('No se detectaron motores físicos')
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
            response.message = f"Error: {str(e)}"
            response.motor_ids = []
            
            return response
        
    # Función para obtener las corrientes actuales de los motores
    def handle_get_motor_currents(self, request, response):
        """Callback para obtener las corrientes actuales de los motores especificados"""
        try:
            # Si no hay motores especificados, utilizar todos los motores configurados
            motor_ids = request.motor_ids
            if not motor_ids or len(motor_ids) == 0:
                motor_ids = self.get_all_motor_ids()
                
            currents = []
            connected_motors = []
            failed_motor_ids = []
            
            # Intentar obtener corrientes reales
            for motor_id in motor_ids:
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    failed_motor_ids.append(motor_id)
                    currents.append(0.0)
                    self.get_logger().warning(f'No se encontró manager o lock para motor {motor_id}')
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        connected_motors.append(motor_id)
                        with lock:
                            # Obtener corriente actual del motor
                            current_result = manager.get_present_iq(local_id)
                        if current_result and len(current_result) > 0:
                            current_current = float(current_result[0][0][0])
                            currents.append(current_current)
                            self.get_logger().info(f'Motor {motor_id} (local {local_id}): corriente actual {current_current}')
                        else:
                            currents.append(0.0)  # Valor por defecto si no se pudo leer
                            failed_motor_ids.append(motor_id)
                            self.get_logger().warning(f'No se pudo leer la corriente del motor {motor_id}')
                    else:
                        failed_motor_ids.append(motor_id)
                        currents.append(0.0)  # Valor por defecto para motores desconectados
                        self.get_logger().warning(f'Motor {motor_id} no responde')
                except Exception as e:
                    self.get_logger().error(f'Error al leer corriente del motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)
                    currents.append(0.0)
            
            # Si no pudimos obtener corrientes reales, es un error
            if not currents:
                response.success = False
                response.message = "No se pudieron leer las corrientes de los motores"
                response.currents = []
                return response
            
            # Asegurarse de que tenemos una corriente para cada motor solicitado
            while len(currents) < len(motor_ids):
                currents.append(0.0)

            # Preparar respuesta
            response.success = True
            if connected_motors:
                response.message = f"Corrientes leídas para {len(connected_motors)} motores"
            else:
                response.message = "No se pudieron leer corrientes"
            response.currents = currents
            return response
        
        except Exception as e:
            self.get_logger().error(f'Error en servicio de lectura de corrientes: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            response.currents = []
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
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager o lock para motor {motor_id}')
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        with lock:
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
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager o lock para motor {motor_id}')
                    continue
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        with lock:
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
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager o lock para motor {motor_id}')
                    continue
                
                # Obtener el índice del motor en la lista original
                idx = request.motor_ids.index(motor_id)
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        with lock:
                          connected_motors.append(motor_id)
                          self.get_logger().info(f'Motor {motor_id} conectado y listo para configuración')

                          # Establecer el modo y las ganancias correspondientes
                          mode = request.modes[idx]
                          manager.set_mode((local_id, mode))
                        
                        # Configurar ganancias según el modo
                        if mode == 0:  # Modo corriente
                            # Configurar ganancias de corriente (iq/id)
                            manager.set_p_gain_iq((local_id, self.default_current_gains['p_gain_iq']))
                            manager.set_i_gain_iq((local_id, self.default_current_gains['i_gain_iq']))
                            manager.set_d_gain_iq((local_id, self.default_current_gains['d_gain_iq']))
                            manager.set_p_gain_id((local_id, self.default_current_gains['p_gain_id']))
                            manager.set_i_gain_id((local_id, self.default_current_gains['i_gain_id']))
                            manager.set_d_gain_id((local_id, self.default_current_gains['d_gain_id']))
                            # Desactivar ganancias de posición
                            manager.set_p_gain_position((local_id, 0.0))
                            manager.set_i_gain_position((local_id, 0.0))
                            manager.set_d_gain_position((local_id, 0.0))
                            # Desactivar ganancias de velocidad
                            manager.set_p_gain_velocity((local_id, 0.0))
                            manager.set_i_gain_velocity((local_id, 0.0))
                            manager.set_d_gain_velocity((local_id, 0.0))
                            # Desactivar ganancias de fuerza (para modo corriente puro)
                            if hasattr(manager, 'set_p_gain_force'):
                                manager.set_p_gain_force((local_id, 0.0))
                                manager.set_i_gain_force((local_id, 0.0))
                                manager.set_d_gain_force((local_id, 0.0))
                            self.get_logger().info(f'Modo 0 (corriente) y ganancias configuradas para motor {motor_id}')
                            
                        elif mode == 1:  # Modo velocidad
                            # Configurar ganancias de corriente
                            manager.set_p_gain_iq((local_id, self.default_current_gains['p_gain_iq']))
                            manager.set_i_gain_iq((local_id, self.default_current_gains['i_gain_iq']))
                            manager.set_d_gain_iq((local_id, self.default_current_gains['d_gain_iq']))
                            manager.set_p_gain_id((local_id, self.default_current_gains['p_gain_id']))
                            manager.set_i_gain_id((local_id, self.default_current_gains['i_gain_id']))
                            manager.set_d_gain_id((local_id, self.default_current_gains['d_gain_id']))
                            manager.set_limit_iq_max((local_id, self.default_current_gains['iq_max']))
                            # Desactivar ganancias de posición
                            manager.set_p_gain_position((local_id, 0.0))
                            manager.set_i_gain_position((local_id, 0.0))
                            manager.set_d_gain_position((local_id, 0.0))
                            # Desactivar ganancias de fuerza
                            if hasattr(manager, 'set_p_gain_force'):
                                manager.set_p_gain_force((local_id, 0.0))
                                manager.set_i_gain_force((local_id, 0.0))
                                manager.set_d_gain_force((local_id, 0.0))
                            self.get_logger().info(f'Modo 1 (velocidad) y ganancias configuradas para motor {motor_id}')
                            
                        elif mode == 2:  # Modo posición
                            # Configurar ganancias de corriente
                            manager.set_p_gain_iq((local_id, self.default_current_gains['p_gain_iq']))
                            manager.set_i_gain_iq((local_id, self.default_current_gains['i_gain_iq']))
                            manager.set_d_gain_iq((local_id, self.default_current_gains['d_gain_iq']))
                            manager.set_p_gain_id((local_id, self.default_current_gains['p_gain_id']))
                            manager.set_i_gain_id((local_id, self.default_current_gains['i_gain_id']))
                            manager.set_d_gain_id((local_id, self.default_current_gains['d_gain_id']))
                            # Configurar ganancias de posición
                            manager.set_p_gain_position((local_id, self.default_position_gains['p_gain_position']))
                            manager.set_i_gain_position((local_id, self.default_position_gains['i_gain_position']))
                            manager.set_d_gain_position((local_id, self.default_position_gains['d_gain_position']))
                            # Configurar límite de corriente para posición
                            manager.set_limit_iq_max((local_id, self.default_position_gains['iq_max']))

                            # Desactivar ganancias de fuerza
                            if hasattr(manager, 'set_p_gain_force'):
                                manager.set_p_gain_force((local_id, 0.0))
                                manager.set_i_gain_force((local_id, 0.0))
                                manager.set_d_gain_force((local_id, 0.0))
                            self.get_logger().info(f'Modo 2 (posición) y ganancias configuradas para motor {motor_id}')
                            
                        else:
                            self.get_logger().warning(f'Modo {mode} no reconocido para motor {motor_id}, solo se estableció el modo')
                            self.get_logger().info(f'Modo {mode} configurado para motor {motor_id} (local {local_id})')
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
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager o lock para motor {motor_id}')
                    continue
                
                # Obtener el índice del motor en la lista original
                idx = request.motor_ids.index(motor_id)
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        with lock:
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
                manager, local_id, lock = self.get_manager_for_motor(motor_id)
                
                if manager is None or lock is None:
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'No se encontró manager o lock para motor {motor_id}')
                    continue
                
                # Obtener el índice del motor en la lista original
                idx = request.motor_ids.index(motor_id)
                
                try:
                    ping_result = self.ping_motor(motor_id)
                    if ping_result:
                        with lock:
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

    def handle_set_motor_id_and_target_current(self, request, response):
        """Callback to control multiple motors with individual target currents"""
        try:
            if not request.motor_ids or len(request.motor_ids) == 0:
                response.success = False
                response.message = "No motor IDs specified"
                return response
            
            if len(request.motor_ids) != len(request.target_currents):
                response.success = False
                response.message = "The number of motor IDs does not match the number of target currents"
                return response

            successful_motors = []
            failed_motor_ids = []

            self.get_logger().info(f'Starting current control for motors: {request.motor_ids} to currents: {request.target_currents}')

            for motor_id in request.motor_ids:
                if not self.ping_motor(motor_id):
                    failed_motor_ids.append(motor_id)
                    self.get_logger().warning(f'Motor {motor_id} is not available')

            available_motors = [m for m in request.motor_ids if m not in failed_motor_ids]

            if available_motors:
                self.get_logger().info(f'Available motors for control: {available_motors}')
            else:
                self.get_logger().warning(f'No motors responded. Failed: {failed_motor_ids}')

            for motor_id in available_motors:
                idx = request.motor_ids.index(motor_id)
                manager, local_id, lock = self.get_manager_for_motor(motor_id)

                if manager is None or lock is None:
                    self.get_logger().error(f'No manager or lock found for motor {motor_id}')
                    failed_motor_ids.append(motor_id)
                    continue

                try:
                    with lock:
                        target_current = request.target_currents[idx]
                        self.get_logger().info(f'Motor {motor_id}: setting target current {target_current:.3f}')

                        # Set to current control mode (mode 0)
                        manager.set_mode((local_id, 0))
                        # Enable torque
                        manager.set_torque_enable((local_id, 1))
                        # Set goal current
                        manager.set_goal_iq((local_id, target_current))

                        successful_motors.append(motor_id)
                        self.get_logger().info(f'Motor {motor_id} configured for current control')

                except Exception as e:
                    self.get_logger().error(f'Error configuring/controlling motor {motor_id}: {str(e)}')
                    failed_motor_ids.append(motor_id)

            total_requested = len(request.motor_ids)
            total_successful = len(successful_motors)
            total_failed = len(set(failed_motor_ids))

            self.get_logger().info(f"Final stats: requested={total_requested}, successful={total_successful}, failed={total_failed}")

            if total_successful > 0:
                response.success = True
                response.message = f"Successfully controlled {total_successful} motors: {successful_motors}"
            else:
                response.success = False
                response.message = f"Failed to control any motors. Failed: {list(set(failed_motor_ids))}"

            return response

        except Exception as e:
            import traceback
            self.get_logger().error(f'Error in multi-motor current service: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {str(e)}"
            return response

def main():
    rclpy.init()
    node = WestwoodMotorServer()
    try:
        # Use MultiThreadedExecutor for concurrent service calls
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error en la ejecución: {str(e)}")
        import traceback
        print(traceback.format_exc())
    finally:
        # Stop connection monitoring timer
        if hasattr(node, 'connection_check_timer') and node.connection_check_timer is not None:
            try:
                node.connection_check_timer.cancel()
                print("Connection monitoring timer stopped")
            except Exception as e:
                print(f"Error stopping connection timer: {str(e)}")
        
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
        
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {str(e)}")
        
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error shutting down rclpy: {str(e)}")

if __name__ == '__main__':
    main()