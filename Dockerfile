FROM ros:humble

# system deps
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-pyqt6 \
    libxcb-cursor0 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libopencv-dev \
    python3-opencv \
    curl \
    git \
    x11-utils \
    ros-humble-micro-ros-msgs \
    && rm -rf /var/lib/apt/lists/*

# python packages
RUN pip3 install platformio ifcopenshell numpy

# micro-ROS agent
RUN mkdir -p /microros_ws/src && \
    cd /microros_ws/src && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git && \
    cd /microros_ws && \
    rosdep update && \
    apt-get update && \
    bash -c "source /opt/ros/humble/setup.bash && \
             rosdep install --from-paths src --ignore-src -y && \
             colcon build && \
             source install/local_setup.bash && \
             ros2 run micro_ros_setup create_agent_ws.sh && \
             ros2 run micro_ros_setup build_agent.sh" && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /risc_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /microros_ws/install/local_setup.bash" >> ~/.bashrc

RUN pio platform install teensy