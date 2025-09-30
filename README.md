# SMILEi Workspace

Repositorio del workspace completo para el robot SMILEi, contiene los paquetes ROS2 necesarios para el control de motores Westwood y la máquina de estados del robot.

## Descripción

Este workspace está diseñado para el control del robot SMILEi, un robot humanoide desarrollado con motores Westwood. Integra controladores de bajo nivel para los motores junto con una arquitectura de máquina de estados para el control de alto nivel del robot.

## Estructura del Workspace

El workspace contiene los siguientes paquetes:

- **westwood_motor_control_sdk_wrapper**: Wrapper ROS2 para el SDK de control de motores Westwood
- **westwood_motor_interfaces**: Interfaces ROS2 personalizadas para el control de motores Westwood
- **smilei_state_machine**: Máquina de estados para el robot SMILEi basada en py_trees

## Requisitos del Sistema

- Ubuntu 22.04 o superior
- ROS2 Humble o superior
- Python 3.8+
- Conexión a los motores Westwood a través de USB

## Instalación

1. Clona el repositorio completo:

```bash
git clone https://github.com/JordanPalafox/smilei_ws.git
cd smilei_ws
```

2. Instala las dependencias del sistema:

```bash
sudo apt update
sudo apt install -y python3-pip python3-rosdep
pip3 install py_trees py_trees_ros
```

3. Inicializa y actualiza rosdep si no lo has hecho antes:

```bash
sudo rosdep init
rosdep update
```

4. Instala las dependencias de ROS:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

5. Compila el workspace:

```bash
colcon build
```

6. Configura el entorno:

```bash
source install/setup.bash
```

## Creacion de contenedor Docker (opcional)

Se proporciona un `Dockerfile` y un archivo de `docker-compose.yml` para crear un contenedor con todo el entorno configurado, listo para la teleoperación del robot SMILEi.

Para usar Docker, asegúrate de tener Docker y Docker Compose instalados en tu sistema.

**ℹ️ Nota:** Para usar un contenedor para una **Jetson** debes usar el Dockerfile específico para Jetson que se encuentra en la rama `jetson-container`. Hecho esto, puedes seguir los mismos pasos que se indican a continuación.

Puedes saber esto con los siguientes comandos:

```bash
docker --version
docker compose version
```

Si no los tienes instalados, puedes seguir las instrucciones oficiales:
- [Instalar Docker y Docker Compose](https://docs.docker.com/engine/install/ubuntu/)


### Pasos para crear y usar el contenedor Docker

1. Construye la imagen Docker:

```bash
    docker compose up --build
```

Esto creará una imagen llamada `smilei_image` y creara automáticamente un contenedor `smilei_ros2_container`.


**ℹ️ Nota:** Puede que al ejecutar el contenedor este puede quedarse congelado en la terminal, puedes parar el proceso con `CTRL + C` y luego iniciar el contenedor como se indica en el siguiente paso..


2. Inicia el contenedor:

```bash
    docker start smilei_ros2_container
```

Puedes mirar todos los contenedores activos con `docker ps`.

3. Accede al contenedor:

```bash
    docker exec -it smilei_ros2_container bash
```

Esto abrirá una terminal dentro del contenedor con el entorno ROS2 ya configurado.

4. Parar el contenedor:

```bash
    docker stop smilei_ros2_container
```

Esto detendrá el contenedor pero no lo eliminará. Puedes iniciarlo de nuevo con el comando del **paso 2**.

5. Elimina el contenedor (si es necesario):

```bash
    docker compose down
```

Esto detendrá y eliminará el contenedor `smilei_ros2_container`.

Para recrear el contenedor, simplemente ejecuta de nuevo el comando del **paso 1**.

## Uso

### 1. Iniciar el servidor de control de motores

Primero, inicia el servidor que controla los motores Westwood:

```bash
ros2 run westwood_motor_control_sdk_wrapper westwood_motor_server.py
```

### 2. Iniciar la máquina de estados

En otra terminal (después de hacer source del entorno):

```bash
ros2 run smilei_state_machine state_machine
```

### 3. Enviar comandos al robot

Para controlar el robot, envía comandos a la máquina de estados:

```bash
ros2 run smilei_state_machine send_state_command <comando>
```

Donde `<comando>` puede ser:
- `enable`: Habilitar los motores
- `disable`: Deshabilitar los motores
- `home`: Ir a posición home
- `zero`: Ir a posición cero
- `say_hello`: Ejecutar secuencia de saludo
- `teleop`: Activar modo de teleoperación local

## Descripción de los paquetes

### westwood_motor_control_sdk_wrapper

Este paquete proporciona un wrapper para el SDK de control de motores Westwood, permitiendo controlar los motores a través de servicios ROS2.

Servicios principales:

### Control de motores

- **/westwood_motor/set_motor_id_and_target** (`SetMotorIdAndTarget`): Establece la posición objetivo para uno o varios motores
- **/westwood_motor/get_motor_positions** (`GetMotorPositions`): Obtiene las posiciones actuales de los motores
- **/westwood_motor/get_available_motors** (`GetAvailableMotors`): Obtiene la lista de IDs de motores disponibles

### Configuración de control

- **/westwood_motor/set_position_gains** (`SetGains`): Configura las ganancias PID para el control de posición
- **/westwood_motor/set_current_gains** (`SetGains`): Configura las ganancias PID para el control de corriente
- **/westwood_motor/set_mode** (`SetMode`): Configura el modo de operación del motor
- **/westwood_motor/set_torque_enable** (`SetTorqueEnable`): Habilita o deshabilita el torque del motor
- **/westwood_motor/set_goal_iq** (`SetGoalIq`): Establece la corriente iq objetivo


[Más información](src/westwood_motor_control_sdk_wrapper/README.md)

### westwood_motor_interfaces

Este paquete contiene las definiciones de mensajes, servicios y acciones para interactuar con los motores Westwood a través de ROS2.

Interfaces principales:
- Servicios para control de posición, velocidad y corriente
- Servicios para configuración de parámetros PID
- Servicios para habilitación/deshabilitación de torque

[Más información](src/westwood_motor_interfaces/README.md)

### smilei_state_machine

Este paquete implementa una máquina de estados para el robot SMILEi basada en el framework py_trees. Permite controlar el robot en diferentes estados: home, zero, teleoperación, etc.

Comportamientos disponibles:
- `EnableRobot`: Habilita los motores
- `DisableRobot`: Deshabilita los motores
- `HomePosition`: Ir a posición home
- `ZeroPosition`: Ir a posición cero
- `SayHello`: Secuencia de saludo
- `LocalTeleoperation`: Modo teleoperación

[Más información](src/smilei_state_machine/README.md)

## Resolución de problemas

### El servidor no detecta los motores

- Verifica que el puerto USB esté correctamente conectado
- Comprueba los permisos del puerto: `sudo chmod 666 /dev/ttyUSB*`
- Verifica que los motores estén alimentados
- Prueba con diferentes valores de baudrate

### La máquina de estados no responde

- Asegúrate de que el servidor de motores esté activo
- Verifica en los logs si hay errores específicos
- Reinicia los nodos si es necesario

### Los motores no se mueven

- Asegúrate de haber habilitado los motores con el comando `enable`
- Verifica que no haya límites de articulaciones en conflicto
- Comprueba los valores PID en la configuración