import rclpy
from rclpy.node import Node
from enum import Enum
import csv
import os
import time
from std_msgs.msg import Float32MultiArray

# FULL COMMAND BUS
class CMD:
    STATE_REQUEST     = 0
    TOOL_ROT_TARGET   = 1
    TOOL_SWING_TARGET = 2
    X_LEAD_TARGET     = 3
    ZL_TARGET         = 4
    ZE_TARGET         = 5
    WHEEL_FL_VEL      = 6
    WHEEL_FE_VEL      = 7
    WHEEL_RL_VEL      = 8
    WHEEL_RE_VEL      = 9
    GRIP_OPEN         = 10
    EXTRACTOR_ON      = 11
    CONVEYOR_ON       = 12
    ADHESIVE_ON       = 13
    TRIGGER_HOMING    = 14

# FULL STATUS BUS
class STAT:
    HW_ID            = 0
    POS_ALPHA        = 1
    POS_BETA         = 2
    GRIPPER_DETECT   = 3
    PROX_SENSOR      = 4
    TASK_COMPLETE    = 5
    LIMIT_MIN_HIT    = 6
    LIMIT_MAX_HIT    = 7

class RobotState(Enum):
    IDLE = 0            
    HOMING = 1           
    NAVIGATION = 2      
    FEEDING = 3         
    GRIP_ENGAGE = 4     
    SWING_DOWN = 5       
    ROTATE_TO_TARGET = 6 
    LOWER_AND_PLACE = 7  
    RELEASE_BRICK = 8
    LIFT_UP = 9
    ROTATE_RESET = 10    
    SWING_UP_RESET = 11  
    SAFETY_STOP = 99     

class BIMCoordinator(Node):
    def __init__(self):
        super().__init__('bim_coordinator')

        self.TARGET_SWING_UP = 248.0   
        self.TARGET_SWING_DOWN = 68.9  
        self.Z_HOVER_HEIGHT = 50.0   #placeholder
        self.Z_PLACE_HEIGHT = 150.0  #placeholder place height
        
        self.csv_path = '/risc_ws/src/risc_control/wall_design.csv'
        self.brick_queue = []
        self.current_brick = None
        self.load_csv()

        self.state = RobotState.IDLE
        self.status_data = {1: None, 2: None, 3: None, 4: None} 
        
        #crossbeam sync variables
        self.z_sync_start_time = None
        self.z_sync_timeout = 2.0 

        self.tool_pub = self.create_publisher(Float32MultiArray, 'risc_command', 10)
        self.xax_pub = self.create_publisher(Float32MultiArray, 'xaxis_command', 10)
        self.z_pub = self.create_publisher(Float32MultiArray, 'z_command', 10)
        
        self.create_subscription(Float32MultiArray, 'toolhead_status', lambda m: self.status_cb(m, 1), 10)
        self.create_subscription(Float32MultiArray, 'xaxis_status', lambda m: self.status_cb(m, 2), 10)
        self.create_subscription(Float32MultiArray, 'zl_status', lambda m: self.status_cb(m, 3), 10)
        self.create_subscription(Float32MultiArray, 'ze_status', lambda m: self.status_cb(m, 4), 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("BIM Coordinator Initialized with full Bus support.")

    def load_csv(self):
        if os.path.exists(self.csv_path):
            with open(self.csv_path, mode='r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.brick_queue.append({'x': float(row['x']), 'theta': float(row['theta'])})

    def status_cb(self, msg, hw_id):
        self.status_data[hw_id] = msg.data

    def is_ready(self, hw_id):
        if self.status_data[hw_id] is None: return False
        return self.status_data[hw_id][STAT.TASK_COMPLETE] > 0.5

    def wait_for_user(self, prompt_message):
        self.get_logger().info(f"\n>>> PAUSED: {prompt_message}")
        while True:
            user_input = input("Enter 'go' to continue or 'stop' to reset: ").lower()
            if user_input == "go": return True
            if user_input == "stop":
                self.state = RobotState.IDLE
                return False

    def control_loop(self):
        msg = Float32MultiArray()
        d = [0.0] * 15 
        d[CMD.STATE_REQUEST] = float(self.state.value)
        d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_UP
        
        #persist targets
        if self.current_brick:
            d[CMD.X_LEAD_TARGET] = self.current_brick['x']
            d[CMD.ZL_TARGET] = self.Z_HOVER_HEIGHT
            d[CMD.ZE_TARGET] = self.Z_HOVER_HEIGHT

        # --- FSM LOGIC ---
        if self.state == RobotState.IDLE:
            if self.wait_for_user("Ready to start HOMING?"):
                self.state = RobotState.HOMING

        elif self.state == RobotState.HOMING:
            d[CMD.X_LEAD_TARGET] = -1000.0 
            d[CMD.ZL_TARGET] = -1000.0
            d[CMD.ZE_TARGET] = -1000.0
            
            #crossbeam sync logic
            zl_hit = self.status_data[3] and self.status_data[3][STAT.LIMIT_MIN_HIT] > 0.5
            ze_hit = self.status_data[4] and self.status_data[4][STAT.LIMIT_MIN_HIT] > 0.5
            
            if (zl_hit or ze_hit) and not (zl_hit and ze_hit):
                if self.z_sync_start_time is None:
                    self.z_sync_start_time = time.time()
                elif (time.time() - self.z_sync_start_time) > self.z_sync_timeout:
                    self.get_logger().error("Z-SYNC FAILURE: Crossbeam out of alignment!")
                    self.state = RobotState.SAFETY_STOP
            
            if zl_hit and ze_hit and self.status_data[2] and self.status_data[2][STAT.LIMIT_MIN_HIT] > 0.5:
                self.z_sync_start_time = None
                if self.wait_for_user("Homed. Ready to load brick and NAVIGATE?"):
                    if self.brick_queue:
                        self.current_brick = self.brick_queue.pop(0)
                        self.state = RobotState.NAVIGATION
                    else: self.state = RobotState.IDLE

        elif self.state == RobotState.NAVIGATION:
            if self.is_ready(1) and self.is_ready(2) and self.is_ready(3) and self.is_ready(4):
                if self.status_data[2] and self.status_data[2][STAT.PROX_SENSOR] > 0.5:
                    if self.wait_for_user("Brick detected. Ready for FEEDING?"):
                        self.state = RobotState.FEEDING

        elif self.state == RobotState.FEEDING:
            d[CMD.GRIP_OPEN] = 1.0; d[CMD.CONVEYOR_ON] = 1.0; d[CMD.EXTRACTOR_ON] = 1.0
            self.tool_pub.publish(msg)
            if self.status_data[1] and self.status_data[1][STAT.GRIPPER_DETECT] > 0.5:
                #shutdown feed motors
                d[CMD.CONVEYOR_ON] = 0.0; d[CMD.EXTRACTOR_ON] = 0.0
                if self.wait_for_user("Brick at gripper switch. Ready to GRIP?"):
                    self.state = RobotState.GRIP_ENGAGE

        elif self.state == RobotState.GRIP_ENGAGE:
            d[CMD.GRIP_OPEN] = 0.0 
            if self.is_ready(1):
                if self.wait_for_user("Grip secure. Ready to SWING DOWN?"):
                    self.state = RobotState.SWING_DOWN

        elif self.state == RobotState.SWING_DOWN:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
            if self.is_ready(1):
                if self.wait_for_user("Down. Ready to ROTATE TO TARGET?"):
                    self.state = RobotState.ROTATE_TO_TARGET

        elif self.state == RobotState.ROTATE_TO_TARGET:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
            d[CMD.TOOL_ROT_TARGET]   = self.current_brick['theta']
            if self.is_ready(1):
                if self.wait_for_user("Aligned. Ready to LOWER AND PLACE?"):
                    self.state = RobotState.LOWER_AND_PLACE

        elif self.state == RobotState.LOWER_AND_PLACE:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
            d[CMD.TOOL_ROT_TARGET]   = self.current_brick['theta']
            d[CMD.ZL_TARGET] = self.Z_PLACE_HEIGHT
            d[CMD.ZE_TARGET] = self.Z_PLACE_HEIGHT
            if self.is_ready(3) and self.is_ready(4):
                if self.wait_for_user("Placed. Ready to RELEASE?"):
                    self.state = RobotState.RELEASE_BRICK

        elif self.state == RobotState.RELEASE_BRICK:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
            d[CMD.TOOL_ROT_TARGET]   = self.current_brick['theta']
            d[CMD.ZL_TARGET] = self.Z_PLACE_HEIGHT
            d[CMD.ZE_TARGET] = self.Z_PLACE_HEIGHT
            d[CMD.GRIP_OPEN] = 1.0 
            if self.is_ready(1):
                if self.wait_for_user("Released. Ready to LIFT UP?"):
                    self.state = RobotState.LIFT_UP

        elif self.state == RobotState.LIFT_UP:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
            d[CMD.TOOL_ROT_TARGET]   = self.current_brick['theta']
            d[CMD.ZL_TARGET] = self.Z_HOVER_HEIGHT
            d[CMD.ZE_TARGET] = self.Z_HOVER_HEIGHT
            if self.is_ready(3) and self.is_ready(4):
                if self.wait_for_user("Clear of brick. Ready to ROTATE RESET?"):
                    self.state = RobotState.ROTATE_RESET

        elif self.state == RobotState.ROTATE_RESET:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
            d[CMD.TOOL_ROT_TARGET]   = 0.0 
            if self.is_ready(1):
                if self.wait_for_user("Rotated to Home. Ready to SWING UP?"):
                    self.state = RobotState.SWING_UP_RESET

        elif self.state == RobotState.SWING_UP_RESET:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_UP
            if self.is_ready(1):
                if self.brick_queue:
                    self.current_brick = self.brick_queue.pop(0)
                    self.state = RobotState.NAVIGATION
                else:
                    self.state = RobotState.IDLE

        elif self.state == RobotState.SAFETY_STOP:
            d = [0.0] * 15 #kill all
            self.get_logger().error("Robot in SAFETY_STOP. Resolve hardware alignment.")

        msg.data = d
        self.tool_pub.publish(msg)
        self.xax_pub.publish(msg)
        self.z_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BIMCoordinator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()