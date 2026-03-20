#!/bin/bash

CONTAINER_NAME="risc_main"
XAXIS_PORT="/dev/ttyACM0"
TOOLHEAD_PORT="/dev/ttyACM1"
ZE_PORT="/dev/ttyACM2"
ZL_PORT="/dev/ttyACM3"

ROS_SOURCE="source /opt/ros/humble/setup.bash && source /risc_ws/install/local_setup.bash"

case "$1" in
  "up")
    docker rm -f $CONTAINER_NAME 2>/dev/null
    docker build -t risc_image .
    docker run -dit --name $CONTAINER_NAME --privileged -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb -v $(pwd):/risc_ws risc_image /bin/bash
    ;;

# --- FLASHING ---
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

  # --- AGENTS ---
  "agent-tool")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 run micro_ros_agent micro_ros_agent serial --dev $TOOLHEAD_PORT -v6"
    ;;
  "agent-xaxis")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 run micro_ros_agent micro_ros_agent serial --dev $XAXIS_PORT -v6"
    ;;
  "agent-ze")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 run micro_ros_agent micro_ros_agent serial --dev $ZE_PORT -v6"
    ;;
  "agent-zl")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 run micro_ros_agent micro_ros_agent serial --dev $ZL_PORT -v6"
    ;;

  # --- MONITORING ---
  "monitor-tool")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && ros2 topic echo /toolhead_status"
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

  # --- CONTROL ---
  "manual")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && python3 /risc_ws/src/risc_control/scripts/manual_control.py"
    ;;
  "coordinator")
    docker exec -it $CONTAINER_NAME bash -c "$ROS_SOURCE && python3 /risc_ws/src/risc_control/bim_coordinator.py"
  ;;

  "stop")
    docker stop $CONTAINER_NAME
    ;;

  *)
    echo "BrickBot System Control"
    echo "------------------------"
    echo "Usage: ./risc.sh {command}"
    echo ""
    echo "CONTAINER:"
    echo "  up             Build and start the Docker container"
    echo "  stop           Stop the Docker container"
    echo ""
    echo "FLASHING:"
    echo "  tool-flash     Upload firmware to Toolhead"
    echo "  xaxis-flash    Upload firmware to X-Axis"
    echo "  ze-flash       Upload firmware to Z-East pillar"
    echo "  zl-flash       Upload firmware to Z-Low pillar"
    echo ""
    echo "AGENTS (Background micro-ROS communication):"
    echo "  agent-tool     Start Agent for Toolhead ($TOOLHEAD_PORT)"
    echo "  agent-xaxis    Start Agent for X-Axis ($XAXIS_PORT)"
    echo "  agent-ze       Start Agent for Z-East ($ZE_PORT)"
    echo "  agent-zl       Start Agent for Z-Low ($ZL_PORT)"
    echo ""
    echo "MONITORING (Live data views):"
    echo "  monitor-tool   View live Toolhead status"
    echo "  monitor-xaxis  View live X-Axis status"
    echo "  monitor-ze     View live Z-East status"
    echo "  monitor-zl     View live Z-Low status"
    echo "  monitor-all    List all active ROS 2 topics"
    echo ""
    echo "CONTROL:"
    echo "  coordinator    Launch Automated FSM"
    echo "  manual         Launch manual terminal controller"
    ;;
esac