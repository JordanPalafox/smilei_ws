#!/usr/bin/env python3
"""
Script de prueba para teleoperación remota SMILEi

Este script facilita las pruebas de comunicación UDP y configuración
antes de usar la teleoperación remota completa.

Uso:
    python3 test_remote_teleoperation.py --mode [server|client|config]
"""

import argparse
import socket
import struct
import time
import threading
import sys
from pathlib import Path

# Usar valores por defecto sin archivo de configuración
CONFIG_AVAILABLE = False

class UDPTester:
    """Clase para probar comunicación UDP"""
    
    def __init__(self):
        # Usar valores por defecto para máquina A
        self.local_ip = '192.168.4.241'
        self.remote_ip = '192.168.4.238'
        self.send_port = 4000
        self.receive_port = 5001
        
        self.running = False
        
    def start_server(self):
        """Inicia servidor UDP (recibe datos)"""
        print(f"🚀 Iniciando servidor UDP en {self.local_ip}:{self.receive_port}")
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((self.local_ip, self.receive_port))
            sock.settimeout(1.0)
            self.running = True
            
            print("✅ Servidor iniciado. Esperando datos...")
            print("   Presione Ctrl+C para terminar")
            
            packet_count = 0
            while self.running:
                try:
                    data, addr = sock.recvfrom(1024)
                    packet_count += 1
                    
                    # Intentar decodificar como datos de motores (8 floats)
                    if len(data) == 32:  # 8 floats * 4 bytes
                        positions = struct.unpack('<8f', data)
                        print(f"📦 Paquete {packet_count} de {addr}: {[f'{p:.3f}' for p in positions[:4]]}")
                    else:
                        print(f"📦 Paquete {packet_count} de {addr}: {data}")
                        
                except socket.timeout:
                    continue
                except KeyboardInterrupt:
                    break
                    
        except Exception as e:
            print(f"❌ Error en servidor: {e}")
        finally:
            sock.close()
            print("🔄 Servidor UDP cerrado")
    
    def start_client(self):
        """Inicia cliente UDP (envía datos)"""
        print(f"🚀 Iniciando cliente UDP hacia {self.remote_ip}:{self.send_port}")
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.running = True
            
            print("✅ Cliente iniciado. Enviando datos de prueba...")
            print("   Presione Ctrl+C para terminar")
            
            packet_count = 0
            start_time = time.time()
            
            while self.running:
                try:
                    # Simular datos de 8 motores
                    current_time = time.time() - start_time
                    positions = [
                        0.1 * np.sin(current_time),      # Motor 1
                        0.1 * np.cos(current_time),      # Motor 2  
                        0.05 * np.sin(2*current_time),   # Motor 3
                        0.05 * np.cos(2*current_time),   # Motor 4
                        0.0, 0.0, 0.0, 0.0              # Motores 5-8
                    ]
                    
                    # Empaquetar datos
                    data = struct.pack('<8f', *positions)
                    sock.sendto(data, (self.remote_ip, self.send_port))
                    
                    packet_count += 1
                    if packet_count % 100 == 0:
                        print(f"📤 Enviados {packet_count} paquetes (posición motor 1: {positions[0]:.3f})")
                    
                    time.sleep(0.001)  # 1kHz como en el sistema real
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"❌ Error enviando: {e}")
                    time.sleep(1)
                    
        except Exception as e:
            print(f"❌ Error en cliente: {e}")
        finally:
            sock.close()
            self.running = False
            print(f"🔄 Cliente UDP cerrado. Enviados {packet_count} paquetes")

def test_network_connectivity():
    """Prueba conectividad de red básica"""
    remote_ip = '192.168.4.238'  # IP por defecto de máquina B
    
    print(f"🌐 Probando conectividad hacia {remote_ip}...")
    
    import subprocess
    try:
        result = subprocess.run(['ping', '-c', '3', remote_ip], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("✅ Conectividad de red OK")
            return True
        else:
            print(f"❌ No se puede alcanzar {remote_ip}")
            print(result.stderr)
            return False
    except Exception as e:
        print(f"❌ Error probando conectividad: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Test de teleoperación remota SMILEi')
    parser.add_argument('--mode', choices=['server', 'client', 'config', 'ping'], 
                       required=True, help='Modo de operación')
    
    args = parser.parse_args()
    
    if args.mode == 'config':
        print("📋 CONFIGURACIÓN ACTUAL")
        print("   Configuración por defecto para máquina A:")
        print(f"   - IP Local: 192.168.4.241")
        print(f"   - IP Remota: 192.168.4.238")
        print(f"   - Puerto Envío: 4000")
        print(f"   - Puerto Recepción: 5001")
        print("   ⚠️ Para máquina B, intercambie las IPs y puertos")
            
    elif args.mode == 'ping':
        test_network_connectivity()
        
    elif args.mode == 'server':
        print("🖥️  MODO SERVIDOR")
        print("   Este modo recibe datos de la máquina remota")
        tester = UDPTester()
        try:
            tester.start_server()
        except KeyboardInterrupt:
            tester.running = False
            
    elif args.mode == 'client':
        print("📡 MODO CLIENTE") 
        print("   Este modo envía datos de prueba a la máquina remota")
        
        # Importar numpy si está disponible para las simulaciones
        try:
            import numpy as np
            globals()['np'] = np
        except ImportError:
            print("⚠️ numpy no disponible, usando funciones básicas")
            import math
            # Simular numpy.sin y numpy.cos
            class FakeMath:
                @staticmethod
                def sin(x): return math.sin(x)
                @staticmethod 
                def cos(x): return math.cos(x)
            globals()['np'] = FakeMath()
        
        tester = UDPTester()
        try:
            tester.start_client()
        except KeyboardInterrupt:
            tester.running = False
    
    print("\n✨ Test completado")

if __name__ == "__main__":
    main()