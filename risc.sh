#!/bin/bash

CONTAINER_NAME="risc_main"

# ---------------------------------------------------------------------------
# port names -- set by udev rules in /etc/udev/rules.d/99-risc.rules
# these are stable symlinks regardless of plug order
# ---------------------------------------------------------------------------
XAXIS_PORT="/dev/risc_xaxis"
TOOLHEAD_PORT="/dev/risc_toolhead"
ZE_PORT="/dev/risc_ze"
ZL_PORT="/dev/risc_zl"

ROS_SOURCE="source /opt/ros/humble/setup.bash && source /risc_ws/install/local_setup.bash"
MICROROS_SOURCE="source /microros_ws/install/local_setup.bash"
COORD_PATH="/risc_ws/src/risc_control/bim_coordinator.py"
HMI_PATH="/risc_ws/scripts/hmi_main.py"

case "$1" in

  # ---------------------------------------------------------------------------
  # container
  # ---------------------------------------------------------------------------
  "up")
    docker rm -f $CONTAINER_NAME 2>/dev/null
    docker build -t risc_image .
    docker run -dit \
      --name $CONTAINER_NAME \
      --privileged \
      --network host \
      -v /dev:/dev \
      -v /dev/bus/usb:/dev/bus/usb \
      -v $(pwd):/risc_ws \
      -e RISC_WS=/risc_ws \
      risc_image /bin/bash
    echo "Container started."
    ;;

  "stop")
    docker stop $CONTAINER_NAME
    ;;

  # ---------------------------------------------------------------------------
  # dockerless -- run everything directly on host (no docker required)
  # requires micro-ROS agent built at ~/microros_ws
  # ---------------------------------------------------------------------------
  "install-agent")
    echo "Building micro-ROS agent on host..."
    mkdir -p ~/microros_ws/src
    cd ~/microros_ws/src
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
    cd ~/microros_ws
    source /opt/ros/humble/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    colcon build
    source install/local_setup.bash
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
    echo "Done. Restart terminal then run: ./risc.sh start-all"
    ;;

  "start-all")
    # start all agents + coordinator on host, no docker
    source ~/microros_ws/install/local_setup.bash
    echo "Starting agents..."
    ros2 run micro_ros_agent micro_ros_agent serial --dev $TOOLHEAD_PORT -v3 &
    ros2 run micro_ros_agent micro_ros_agent serial --dev $XAXIS_PORT   -v3 &
    ros2 run micro_ros_agent micro_ros_agent serial --dev $ZE_PORT      -v3 &
    ros2 run micro_ros_agent micro_ros_agent serial --dev $ZL_PORT      -v3 &
    sleep 3
    echo "Starting coordinator..."
    python3 ~/risc_ws/src/risc_control/bim_coordinator.py &
    echo "All started. Run: ./risc.sh hmi"
    ;;

  "stop-all")
    pkill -f micro_ros_agent || true
    pkill -f bim_coordinator  || true
    echo "All stopped."
    ;;

  "host-agent-tool")
    source ~/microros_ws/install/local_setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev $TOOLHEAD_PORT -v6
    ;;
  "host-agent-xaxis")
    source ~/microros_ws/install/local_setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev $XAXIS_PORT -v6
    ;;
  "host-agent-ze")
    source ~/microros_ws/install/local_setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev $ZE_PORT -v6
    ;;
  "host-agent-zl")
    source ~/microros_ws/install/local_setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev $ZL_PORT -v6
    ;;
  "host-coordinator")
    python3 ~/risc_ws/src/risc_control/bim_coordinator.py
    ;;


  "autostart")
    echo "Killing any existing agents..."
    docker exec $CONTAINER_NAME bash -c "pkill -f micro_ros_agent 2>/dev/null; pkill -f bim_coordinator 2>/dev/null; sleep 1"
    echo "Starting all agents..."
    docker exec -d $CONTAINER_NAME bash -c \
      "$ROS_SOURCE && $MICROROS_SOURCE && \
       ros2 run micro_ros_agent micro_ros_agent serial --dev $TOOLHEAD_PORT -v3"
    docker exec -d $CONTAINER_NAME bash -c \
      "$ROS_SOURCE && $MICROROS_SOURCE && \
       ros2 run micro_ros_agent micro_ros_agent serial --dev $XAXIS_PORT -v3"
    docker exec -d $CONTAINER_NAME bash -c \
      "$ROS_SOURCE && $MICROROS_SOURCE && \
       ros2 run micro_ros_agent micro_ros_agent serial --dev $ZE_PORT -v3"
    docker exec -d $CONTAINER_NAME bash -c \
      "$ROS_SOURCE && $MICROROS_SOURCE && \
       ros2 run micro_ros_agent micro_ros_agent serial --dev $ZL_PORT -v3"
    sleep 2
    echo "Starting coordinator..."
    docker exec -d $CONTAINER_NAME bash -c \
      "$ROS_SOURCE && python3 $COORD_PATH"
    echo "Autostart complete."
    ;;

  # ---------------------------------------------------------------------------
  # hmi -- launch hmi fullscreen on host (outside docker, needs display)
  # ---------------------------------------------------------------------------
  "hmi")
    xhost +local:root 2>/dev/null
    cd /home/risc/risc_ws/scripts && python3 hmi_main.py
    ;;

  # ---------------------------------------------------------------------------
  # flashing
  # ---------------------------------------------------------------------------
  "tool-flash")
    docker exec -it $CONTAINER_NAME bash -c "cd /risc_ws/firmware/toolhead && pio run -e tool_fsm --target upload"
    ;;
  "xaxis-flash")
    docker exec -it $CONTAINER_NAME bash -c "cd /risc_ws/firmware/xaxis && pio run -e xaxis_fsm --target upload"
    ;;
  "ze-flash")
    docker exec -it $CONTAINER_NAME bash -c "cd /risc_ws/firmware/zE && pio run -e ze_fsm --target upload"
    ;;
  "zl-flash")
    docker exec -it $CONTAINER_NAME bash -c "cd /risc_ws/firmware/zL && pio run -e zl_fsm --target upload"
    ;;

  # ---------------------------------------------------------------------------
  # agents (foreground, for debugging)
  # ---------------------------------------------------------------------------
  "agent-tool")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && $MICROROS_SOURCE && ros2 run micro_ros_agent micro_ros_agent serial --dev $TOOLHEAD_PORT -v6"
    ;;
  "agent-xaxis")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && $MICROROS_SOURCE && ros2 run micro_ros_agent micro_ros_agent serial --dev $XAXIS_PORT -v6"
    ;;
  "agent-ze")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && $MICROROS_SOURCE && ros2 run micro_ros_agent micro_ros_agent serial --dev $ZE_PORT -v6"
    ;;
  "agent-zl")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && $MICROROS_SOURCE && ros2 run micro_ros_agent micro_ros_agent serial --dev $ZL_PORT -v6"
    ;;

  # ---------------------------------------------------------------------------
  # coordinator (foreground, for debugging)
  # ---------------------------------------------------------------------------
  "coordinator")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && python3 $COORD_PATH"
    ;;

  # ---------------------------------------------------------------------------
  # monitoring
  # ---------------------------------------------------------------------------
  "monitor-tool")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 topic echo /toolhead_status"
    ;;
  "monitor-rotation")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 topic echo /toolhead_status --field data[2]"
    ;;
  "monitor-xaxis")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 topic echo /xaxis_status"
    ;;
  "monitor-ze")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 topic echo /ze_status"
    ;;
  "monitor-zl")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 topic echo /zl_status"
    ;;
  "monitor-commands")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 topic echo /risc_command"
    ;;
  "monitor-all")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 topic list"
    ;;

  "manual")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && python3 /risc_ws/src/risc_control/scripts/manual_control.py"
    ;;

  *)
    echo "RiSC 1.0 System Control"
    echo "------------------------"
    echo "Usage: ./risc.sh {command}"
    echo ""
    echo "CONTAINER:    up | stop"
    echo "AUTOSTART:    autostart          (agents + coordinator, background)"
    echo "HMI:          hmi               (fullscreen on host display)"
    echo "FLASHING:     tool-flash | xaxis-flash | ze-flash | zl-flash"
    echo "AGENTS:       agent-tool | agent-xaxis | agent-ze | agent-zl"
    echo "COORDINATOR:  coordinator        (foreground)"
    echo "MONITORING:   monitor-tool | monitor-xaxis | monitor-ze | monitor-zl | monitor-all"
    ;;
esac