#!/usr/bin/env python3
"""
Test simple de UDP para verificar comunicación básica entre máquinas
"""

import socket
import struct
import sys
import time

def test_send():
    """Enviar datos UDP de prueba"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Configuración de red (usar las IPs de tu config)
    local_ip = '192.168.4.241'
    remote_ip = '192.168.4.238' 
    port = 4000
    
    print(f"🚀 Enviando desde {local_ip} hacia {remote_ip}:{port}")
    
    try:
        count = 0
        while True:
            # Simular datos de 8 motores
            positions = [0.1 * (count % 100) / 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            data = struct.pack('8f', *positions)
            
            sock.sendto(data, (remote_ip, port))
            print(f"📤 Paquete {count}: posición {positions[0]:.3f}")
            
            count += 1
            time.sleep(0.1)  # 10Hz para ver claramente
            
    except KeyboardInterrupt:
        print("\n🔄 Terminado por usuario")
    finally:
        sock.close()

def test_receive():
    """Recibir datos UDP de prueba"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Configuración de red - usar IP de esta máquina
    local_ip = '192.168.4.241'  # Tu máquina actual
    port = 4000
    
    print(f"🎧 Escuchando en {local_ip}:{port}")
    
    try:
        sock.bind((local_ip, port))
        sock.settimeout(2.0)
        
        count = 0
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) == 32:  # 8 floats
                    positions = struct.unpack('8f', data)
                    print(f"📥 Paquete {count} de {addr}: posición {positions[0]:.3f}")
                    count += 1
                else:
                    print(f"📦 Datos inesperados de {addr}: {len(data)} bytes")
                    
            except socket.timeout:
                print("⏰ Timeout - no se recibieron datos")
                
    except KeyboardInterrupt:
        print("\n🔄 Terminado por usuario")
    finally:
        sock.close()

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ['send', 'receive']:
        print("Uso:")
        print("  python3 test_udp_simple.py send      # Enviar datos")  
        print("  python3 test_udp_simple.py receive   # Recibir datos")
        return
        
    mode = sys.argv[1]
    
    if mode == 'send':
        test_send()
    else:
        test_receive()

if __name__ == '__main__':
    main()