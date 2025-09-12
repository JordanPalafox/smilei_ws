#!/usr/bin/env python3

import socket
import struct
import time
import sys

def test_udp_send(target_ip, target_port):
    """Test UDP send"""
    print(f"Enviando UDP a {target_ip}:{target_port}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Enviar paquetes con 2 floats
    data = struct.pack('2f', 1.234, 5.678)
    
    for i in range(10):
        try:
            sock.sendto(data, (target_ip, target_port))
            print(f"Enviado paquete {i+1}")
            time.sleep(1)
        except Exception as e:
            print(f"Error enviando: {e}")
    
    sock.close()

def test_udp_receive(listen_port):
    """Test UDP receive"""
    print(f"Escuchando UDP en puerto {listen_port}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', listen_port))
    sock.settimeout(1.0)
    
    print("Esperando paquetes (Ctrl+C para salir)...")
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                values = struct.unpack('2f', data)
                print(f"Recibido de {addr}: {values}")
            except socket.timeout:
                print(".", end="", flush=True)
    except KeyboardInterrupt:
        print("\nSaliendo...")
    
    sock.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso:")
        print("  python3 test_udp.py send <target_ip> <target_port>")
        print("  python3 test_udp.py receive <listen_port>")
        print()
        print("Ejemplo máquina A:")
        print("  python3 test_udp.py send 192.168.4.254 4000")
        print("  python3 test_udp.py receive 5005")
        print()
        print("Ejemplo máquina B:")
        print("  python3 test_udp.py send 192.168.4.241 5005")
        print("  python3 test_udp.py receive 4000")
        sys.exit(1)
    
    mode = sys.argv[1]
    
    if mode == "send":
        if len(sys.argv) != 4:
            print("Error: send requiere <target_ip> <target_port>")
            sys.exit(1)
        target_ip = sys.argv[2]
        target_port = int(sys.argv[3])
        test_udp_send(target_ip, target_port)
    
    elif mode == "receive":
        if len(sys.argv) != 3:
            print("Error: receive requiere <listen_port>")
            sys.exit(1)
        listen_port = int(sys.argv[2])
        test_udp_receive(listen_port)
    
    else:
        print(f"Error: modo '{mode}' no reconocido")
        sys.exit(1)