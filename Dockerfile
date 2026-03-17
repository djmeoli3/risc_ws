FROM ros:humble

#Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    curl \
    git \
    ros-humble-micro-ros-msgs \
    && rm -rf /var/lib/apt/lists/*

#Install PlatformIO for firmware builds
RUN pip3 install platformio

#Build micro-ROS Agent from source
RUN mkdir -p /microros_ws/src && \
    cd /microros_ws/src && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git && \
    cd /microros_ws && \
    rosdep update && \
    apt-get update && \
    #Wrap the build in a bash shell to ensure ROS is sourced
    bash -c "source /opt/ros/humble/setup.bash && \
            rosdep install --from-paths src --ignore-src -y && \
            colcon build && \
            source install/local_setup.bash && \
            ros2 run micro_ros_setup create_agent_ws.sh && \
            ros2 run micro_ros_setup build_agent.sh" && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /risc_ws

# Ensure ROS 2 and Agent are sourced in every bash session
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /microros_ws/install/local_setup.bash" >> ~/.bashrc

RUN pio platform install teensy