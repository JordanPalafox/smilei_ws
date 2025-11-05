ARG ROS_DISTRO=humble
FROM dustynv/ros:humble-desktop-l4t-r36.4.0

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
# Set timezone to Coordinated Universal Time (UTC)
ENV TZ=Etc/UTC

# Update ROS2 GPG key and sources list
RUN rm -f /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update

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
    python3.10-venv \
    # Install gstreamer packages 
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    # Install ros2 packages
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-py-trees \
    ros-${ROS_DISTRO}-py-trees-ros-interfaces \
    ros-${ROS_DISTRO}-py-trees-ros \
    ros-${ROS_DISTRO}-py-trees-ros-tutorials \
    ros-${ROS_DISTRO}-py-trees-ros-viewer \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-example-interfaces \
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

# Add user to video group to access GPU
RUN usermod -aG video ${DOCKER_USER}
    
# Install PyBEAR
# Clonar solo la versión específica y con una profundidad mínima para ahorrar tiempo y espacio
RUN git clone --depth 1 --branch 0.1.3 https://github.com/Westwood-Robotics/PyBEAR.git /tmp/PyBEAR && \
# Instalar el paquete directamente desde la carpeta clonada usando PyPI estándar
pip3 install --index-url https://pypi.org/simple/ /tmp/PyBEAR && \
# --- Limpieza Crucial ---
    # Eliminar el código fuente que ya no es necesario
    rm -rf /tmp/PyBEAR && \
    # Limpiar el caché de apt para reducir el tamaño final de la imagen
    apt-get clean && rm -rf /var/lib/apt/lists/*
        
# Create the workspace directory and set up the Python virtual environment
RUN mkdir -p /home/${DOCKER_USER}/smilei_ws/src && \
    cd /home/${DOCKER_USER}/smilei_ws/src/ && \
    python3 -m venv audio_env && \
    chown -R ${DOCKER_USER}:${DOCKER_USER} /home/${DOCKER_USER}/smilei_ws

# Download PyTorch wheel file
RUN wget -O /home/${DOCKER_USER}/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl https://developer.download.nvidia.cn/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
    
# Copy and run the cuSparseLT installation script
COPY install_cusparselt.sh /home/${DOCKER_USER}/

RUN chmod +x /home/${DOCKER_USER}/install_cusparselt.sh && \
    su - ${DOCKER_USER} -c "/home/${DOCKER_USER}/install_cusparselt.sh" && \
    rm /home/${DOCKER_USER}/install_cusparselt.sh

# Install PyTorch and cleanup the wheel
RUN /home/${DOCKER_USER}/smilei_ws/src/audio_env/bin/pip install --ignore-installed --index-url https://pypi.org/simple/ /home/${DOCKER_USER}/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl && \
    rm /home/${DOCKER_USER}/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl

# Download and install TorchAudio
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    libportaudio2 \
    portaudio19-dev \
    ffmpeg libavformat-dev libavcodec-dev libavutil-dev libavdevice-dev libavfilter-dev && \
    /home/${DOCKER_USER}/smilei_ws/src/audio_env/bin/pip install --index-url https://pypi.org/simple/ setuptools && \
    git clone https://github.com/pytorch/audio.git /tmp/audio && \
    cd /tmp/audio && \
    git checkout ea5de177 && \
    USE_CUDA=1 /home/${DOCKER_USER}/smilei_ws/src/audio_env/bin/pip install --verbose --no-use-pep517 . && \
    cd / && \
    rm -rf /tmp/audio && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install depthai and other packages in system Python (where ROS 2 is)
RUN pip3 install \
    --index-url https://pypi.org/simple/ \
    'depthai>=2.25,<3.0'

# Install system packages for ROS 2 nodes
RUN apt-get update && \
    apt-get install -y \
    python3-cv-bridge \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Clone Retrieval-based Voice Conversion WebUI repository
RUN git clone --depth 1 --branch main https://github.com/RVC-Project/Retrieval-based-Voice-Conversion-WebUI.git /home/${DOCKER_USER}/smilei_ws/src/RVC_Project && \
chown -R ${DOCKER_USER}:${DOCKER_USER} /home/${DOCKER_USER}/smilei_ws/src/RVC_Project

# Copy model files
COPY --chown=${DOCKER_USER}:${DOCKER_USER} ./robot-voice-Arturo.pth /home/${DOCKER_USER}/smilei_ws/src/RVC_Project/robot-voice-Arturo.pth
COPY --chown=${DOCKER_USER}:${DOCKER_USER} ./trained_IVF601_Flat_nprobe_1_robot-voice-Arturo_v2.index /home/${DOCKER_USER}/smilei_ws/src/RVC_Project/trained_IVF601_Flat_nprobe_1_robot-voice-Arturo_v2.index

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

# Copy oak_publisher script
COPY ./oak_publisher.py /home/${DOCKER_USER}/

# Copy requirements.txt for venv
COPY requirements.txt /home/${DOCKER_USER}/smilei_ws/

# Install packages from requirements.txt in the venv
RUN /home/${DOCKER_USER}/smilei_ws/src/audio_env/bin/pip install --index-url https://pypi.org/simple/ -r /home/${DOCKER_USER}/smilei_ws/requirements.txt && \
    chown -R ${DOCKER_USER}:${DOCKER_USER} /home/${DOCKER_USER}/smilei_ws/src/audio_env

# Switch to non-root user
USER ${DOCKER_USER}
WORKDIR /home/${DOCKER_USER}

# ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
