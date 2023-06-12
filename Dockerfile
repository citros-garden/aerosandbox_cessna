FROM ros:humble

ENV ROS_DISTRO humble

# install utils
RUN apt-get update && DEBIAN_FRONTEND=noninteractive  && apt-get install -y  --no-install-recommends\
    autoconf \
    automake \
    libtool \
    make \
    g++ \
    unzip \
    libprotobuf-dev \
    wget \
    openssh-server \
    curl \
    gnupg \
    git \
    build-essential \
    cmake \
    gdb-multiarch \
    default-jre \
    python3 \
    python3-setuptools \
    python3-pip \
    python3-venv \
    ros-humble-rosbridge-suite \    
    && rm -rf /var/lib/apt/lists/*

RUN pip install aerosandbox[full]
RUN apt-get update && apt-get install ffmpeg libsm6 libgl1 libxext6  -y

RUN sudo apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rosbag2-storage-mcap \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-ros2bag \
    ros-$ROS_DISTRO-rosbag2-transport \
    && rm -rf /var/lib/apt/lists/* 
RUN  pip install mcap-ros2-support

WORKDIR /workspaces/aerosandbox
COPY . .
RUN colcon build

RUN pip install citros==23.17.7

RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/workspaces/aerosandbox/ros2_entrypoint.sh"]

CMD ["bash"]
