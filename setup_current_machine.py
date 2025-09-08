#!/usr/bin/env python3
"""
Script para configurar la máquina actual para teleoperación remota.
"""

import subprocess
import re

def get_local_ip():
    """Obtiene la IP local de la máquina actual"""
    try:
        result = subprocess.run(['ip', 'addr', 'show'], capture_output=True, text=True)
        # Buscar IP que no sea loopback ni docker
        for line in result.stdout.split('\n'):
            if 'inet ' in line and '127.0.0.1' not in line and 'docker' not in line:
                match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', line)
                if match and not match.group(1).startswith('172.'):
                    return match.group(1)
    except:
        pass
    return None

def main():
    print("🔧 CONFIGURACIÓN DE TELEOPERACIÓN REMOTA")
    print("=" * 50)
    
    # Obtener IP local
    local_ip = get_local_ip()
    if local_ip:
        print(f"✅ IP de esta máquina detectada: {local_ip}")
    else:
        print("❌ No se pudo detectar la IP automáticamente")
        local_ip = input("Ingrese la IP de esta máquina: ")
    
    # Solicitar IP remota
    print(f"\n📡 ¿Cuál es la IP de la máquina remota?")
    remote_ip = input("IP de la otra máquina: ")
    
    # Solicitar IDs de motores
    print(f"\n🔧 ¿Qué motores tienes conectados?")
    motor_input = input("IDs de motores (separados por coma, ej: 1,6): ").strip()
    motor_ids = [int(x.strip()) for x in motor_input.split(',')]
    
    # Generar configuración
    config_content = f"""
# Configuración automática generada
LOCAL_IP = '{local_ip}'
REMOTE_IP = '{remote_ip}'
SEND_PORT = 5005
RECEIVE_PORT = 4000
MOTOR_IDS = {motor_ids}
"""
    
    print(f"\n📝 CONFIGURACIÓN GENERADA:")
    print(config_content)
    
    # Actualizar archivo de configuración
    config_file = "src/smilei_state_machine/config/remote_teleop_config.py"
    
    # Leer archivo existente
    try:
        with open(config_file, 'r') as f:
            content = f.read()
        
        # Actualizar líneas específicas
        lines = content.split('\n')
        for i, line in enumerate(lines):
            if line.startswith('LOCAL_IP ='):
                lines[i] = f"LOCAL_IP = '{local_ip}'"
            elif line.startswith('REMOTE_IP ='):
                lines[i] = f"REMOTE_IP = '{remote_ip}'"
            elif line.startswith('MOTOR_IDS = '):
                lines[i] = f"MOTOR_IDS = {motor_ids}"
        
        # Escribir archivo actualizado
        with open(config_file, 'w') as f:
            f.write('\n'.join(lines))
        
        print(f"✅ Configuración actualizada en {config_file}")
        
    except Exception as e:
        print(f"❌ Error actualizando configuración: {e}")
        print("Por favor actualice manualmente el archivo de configuración")
    
    print(f"\n🚀 PASOS SIGUIENTES:")
    print(f"1. Copie esta configuración a la máquina remota ({remote_ip})")
    print(f"2. En la máquina remota, intercambie LOCAL_IP y REMOTE_IP")
    print(f"3. Ejecute 'ros2 topic pub /state_command std_msgs/String \"data: remote_teleoperation\" --once' en AMBAS máquinas")
    print(f"4. Mueva el motor manualmente - debería replicarse en la otra máquina")
    
    # Test de conectividad
    print(f"\n🌐 PROBANDO CONECTIVIDAD...")
    try:
        result = subprocess.run(['ping', '-c', '2', remote_ip], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✅ Conectividad OK hacia {remote_ip}")
        else:
            print(f"❌ No se puede alcanzar {remote_ip}")
            print("   Verifique que ambas máquinas estén en la misma red")
    except:
        print(f"❌ Error probando conectividad hacia {remote_ip}")

if __name__ == "__main__":
    main()