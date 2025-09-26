ARG ROS_DISTRO=humble
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
# Set timezone to Coordinated Universal Time (UTC)
ENV TZ=Etc/UTC

# Install essential packages and colcon
RUN apt-get update \
    && apt-get install -y \
    # Install useful tools
    tzdata \
    nano \
    vim \
    tmux \
    git \
    wget \
    curl \
    pip \
    cmake \
    python3-colcon-common-extensions \
    # Install ros2 packages
    ros-${ROS_DISTRO}-foxglove-bridge \ 
    ros-${ROS_DISTRO}-py-trees \
    ros-${ROS_DISTRO}-py-trees-ros-interfaces \
    ros-${ROS_DISTRO}-py-trees-ros \
    ros-${ROS_DISTRO}-py-trees-ros-tutorials \
    ros-${ROS_DISTRO}-py-trees-ros-viewer \
    && rm -rf /var/lib/apt/lists/*


# Create a non-root user to avoid permission issues
ARG DOCKER_USER=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $DOCKER_USER \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $DOCKER_USER \
    && mkdir /home/$DOCKER_USER/.config && chown $USER_UID:$USER_GID /home/$DOCKER_USER/.config

# Configure passwordless sudo for the non-root user
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $DOCKER_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$DOCKER_USER\
    && chmod 0440 /etc/sudoers.d/$DOCKER_USER \
    && rm -rf /var/lib/apt/lists/*
  
# Ensure the non-root user has ownership of /usr/local (needed for some pip installs)
RUN chown -R ${DOCKER_USER} /usr/local
# Add user to dialout group to access serial ports
RUN usermod -aG dialout ${DOCKER_USER}

# Install PyBEAR
# Clonar solo la versión específica y con una profundidad mínima para ahorrar tiempo y espacio
RUN git clone --depth 1 --branch 0.1.3 https://github.com/Westwood-Robotics/PyBEAR.git /tmp/PyBEAR && \
    # Instalar el paquete directamente desde la carpeta clonada
    # pip se encargará de instalar las dependencias definidas en el paquete
    pip3 install /tmp/PyBEAR && \
    # --- Limpieza Crucial ---
    # Eliminar el código fuente que ya no es necesario
    rm -rf /tmp/PyBEAR && \
    # Limpiar el caché de apt para reducir el tamaño final de la imagen
    apt-get clean && rm -rf /var/lib/apt/lists/*
    
# Copy setup script and give execution permissions
COPY --chown=${DOCKER_USER}:${DOCKER_USER} ./setup.sh /home/${DOCKER_USER}/setup.sh
RUN chmod +x /home/${DOCKER_USER}/setup.sh

# Run automatic sourcing of colcon argcomplete
RUN echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> /home/${DOCKER_USER}/.bashrc && \
    # Add ROS setup to bashrc  
    echo 'source /opt/ros/humble/setup.bash' >> /home/${DOCKER_USER}/.bashrc && \
    # Set up tmux configuration
    echo 'set -g default-terminal "screen-256color"' >> /home/${DOCKER_USER}/.tmux.conf && \
    echo 'set -g mouse on' >> /home/${DOCKER_USER}/.tmux.conf

# Crear el directorio del workspace y asegurar que el propietario sea el usuario no-root
RUN mkdir -p /home/${DOCKER_USER}/smilei_ws/src && \
    chown -R ${DOCKER_USER}:${DOCKER_USER} /home/${DOCKER_USER}/smilei_ws

# Switch to non-root user
USER ${DOCKER_USER}
WORKDIR /home/${DOCKER_USER}

# ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
