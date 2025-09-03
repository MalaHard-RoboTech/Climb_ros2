FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    nano \
    git \
    wget \
    vim \
    tar xz-utils \
    libx11-6 libxcb1 libxau6 libgl1-mesa-dev \
    xvfb dbus-x11 x11-utils libxkbcommon-x11-0 \
    libavcodec-dev libavformat-dev libswscale-dev \
    python3-venv libraw1394-11 libmpfr6 libusb-1.0-0 \
    xsltproc \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    libxcb-xinerama0 \  
    python3-markdown \
    doxygen \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install pyzmq cbor2 \
    pyserial \
    flask \
    flask-ask-sdk \
    ask-sdk \
    notebook \ 
    pyyaml \
    xmlschema
# ROS workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src

ENV QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms
ENV QT_QPA_PLATFORM=xcb

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD ["bash"]