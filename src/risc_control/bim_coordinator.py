import rclpy
from rclpy.node import Node
from enum import Enum
import csv
import os
import time
from std_msgs.msg import Float32MultiArray

# MASTER BUS INDEX DEFINITIONS
class CMD:
    STATE_REQUEST     = 0
    TOOL_ROT_TARGET   = 1
    TOOL_SWING_TARGET = 2
    X_LEAD_TARGET     = 3
    ZL_TARGET         = 4
    ZE_TARGET         = 5
    WHEEL_FL_VEL      = 6   
    WHEEL_FE_VEL      = 7   
    WHEEL_RL_VEL      = 8   #unused, left for 4 wheel drive potential
    WHEEL_RE_VEL      = 9   #unused, left for 4 wheel drive potential
    WHEEL_FL_VEL      = 6   
    WHEEL_FE_VEL      = 7   
    WHEEL_RL_VEL      = 8   
    WHEEL_RE_VEL      = 9   
    GRIP_OPEN         = 10
    EXTRACTOR_ON      = 11
    CONVEYOR_ON       = 12
    ADHESIVE_ON       = 13

class STAT:
    HW_ID            = 0
    POS_ALPHA        = 1
    TASK_COMPLETE    = 5
    LIMIT_MIN_HIT    = 6
    LIMIT_MAX_HIT    = 7
    PROX_SENSOR      = 4
    GRIPPER_DETECT   = 3

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

        #CALIBRATED CONSTANTS
        self.TARGET_SWING_UP = 248.0   
        self.TARGET_SWING_DOWN = 68.9  
        self.Z_SAFETY_MARGIN = 60.0  #mm above current layer
        self.Z_SAFETY_MARGIN = 60.0  
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.csv_path = os.path.join(script_dir, 'wall_design.csv')
        self.brick_queue = []
        self.current_brick = None
        self.load_csv()

        self.state = RobotState.IDLE
        self.status_data = {2: None, 3: None, 4: None, 1: None} 
        
        #HOMING LATCHES
        self.zl_homed = False
        self.ze_homed = False
        self.xax_homed = False
        
        self.state_entry_time = 0
        self.latch_ready = False 
        self.nav_x_done = False
        self.nav_zl_done = False
        self.nav_ze_done = False
        self.debug_counter = 0

        self.tool_pub = self.create_publisher(Float32MultiArray, 'risc_command', 10)
        self.xax_pub = self.create_publisher(Float32MultiArray, 'xaxis_command', 10)
        self.z_pub = self.create_publisher(Float32MultiArray, 'z_command', 10)
        
        self.create_subscription(Float32MultiArray, 'toolhead_status', lambda m: self.status_cb(m, 1), 10)
        self.create_subscription(Float32MultiArray, 'xaxis_status', lambda m: self.status_cb(m, 2), 10)
        self.create_subscription(Float32MultiArray, 'zl_status', lambda m: self.status_cb(m, 3), 10)
        self.create_subscription(Float32MultiArray, 'ze_status', lambda m: self.status_cb(m, 4), 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("BIM Coordinator: Release-to-Lift dwell added.")

    def load_csv(self):
        if os.path.exists(self.csv_path):
            with open(self.csv_path, mode='r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.brick_queue.append(row)
                self.get_logger().info(f"Loaded {len(self.brick_queue)} bricks.")

    def status_cb(self, msg, hw_id):
        self.status_data[hw_id] = msg.data

    def change_state(self, new_state):
        self.get_logger().info(f"Transitioning: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_entry_time = time.time()
        self.latch_ready = False 
        self.nav_x_done = False
        self.nav_zl_done = False
        self.nav_ze_done = False

    def is_ready(self, hw_id):
        if self.status_data.get(hw_id) is None: return False
        if not self.latch_ready:
            if self.status_data[hw_id][STAT.TASK_COMPLETE] < 0.5:
                self.latch_ready = True
            return False
        return self.status_data[hw_id][STAT.TASK_COMPLETE] > 0.5

    def control_loop(self):
        msg = Float32MultiArray()
        d = [0.0] * 15 
        d[CMD.STATE_REQUEST] = float(self.state.value)
        
        states_where_arm_is_down = [
            RobotState.SWING_DOWN, RobotState.ROTATE_TO_TARGET, 
            RobotState.LOWER_AND_PLACE, RobotState.RELEASE_BRICK, 
            RobotState.LIFT_UP, RobotState.ROTATE_RESET
        ]
        
        if self.state in states_where_arm_is_down:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
        else:
            d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_UP
        
        if self.current_brick:
            d[CMD.X_LEAD_TARGET] = float(self.current_brick['x'])
            y_target = float(self.current_brick['y'])
            d[CMD.WHEEL_FL_VEL] = y_target; d[CMD.WHEEL_FE_VEL] = y_target 

            current_z_layer = float(self.current_brick['z'])
            dynamic_hover = current_z_layer + self.Z_SAFETY_MARGIN
            
            if self.state == RobotState.LOWER_AND_PLACE or self.state == RobotState.RELEASE_BRICK:
                target_z = current_z_layer + float(self.current_brick.get('drop_offset', 0.0))
            else:
                target_z = dynamic_hover
                
            d[CMD.ZL_TARGET] = target_z; d[CMD.ZE_TARGET] = target_z

        elapsed = time.time() - self.state_entry_time

        if self.state == RobotState.IDLE:
            if self.brick_queue: self.change_state(RobotState.HOMING)

        elif self.state == RobotState.HOMING:
            d[CMD.X_LEAD_TARGET] = -1000.0; d[CMD.ZL_TARGET] = -1000.0; d[CMD.ZE_TARGET] = -1000.0
            zl_msg = self.status_data.get(3); ze_msg = self.status_data.get(4); xax_msg = self.status_data.get(2)
            if zl_msg and zl_msg[STAT.LIMIT_MIN_HIT] > 0.5: self.zl_homed = True
            if ze_msg and ze_msg[STAT.LIMIT_MIN_HIT] > 0.5: self.ze_homed = True
            if xax_msg and xax_msg[STAT.LIMIT_MIN_HIT] > 0.5: self.xax_homed = True
            if self.zl_homed and self.ze_homed and self.xax_homed:
                if self.brick_queue:
                    self.current_brick = self.brick_queue.pop(0)
                    self.change_state(RobotState.NAVIGATION)

        elif self.state == RobotState.NAVIGATION:
            #move to targeets, check for completion
            if self.is_ready(2): self.nav_x_done = True
            if self.is_ready(3): self.nav_zl_done = True
            if self.is_ready(4): self.nav_ze_done = True
            if self.nav_x_done and self.nav_zl_done and self.nav_ze_done:
                if self.status_data[2] and self.status_data[2][STAT.PROX_SENSOR] > 0.5:
                    self.change_state(RobotState.FEEDING)

        elif self.state == RobotState.FEEDING:
            d[CMD.GRIP_OPEN] = 1.0; d[CMD.CONVEYOR_ON] = 1.0; d[CMD.EXTRACTOR_ON] = 1.0
            if self.status_data[1] and self.status_data[1][STAT.GRIPPER_DETECT] > 0.5:
                if elapsed > 2.0: 
                    d[CMD.CONVEYOR_ON] = 0.0; d[CMD.EXTRACTOR_ON] = 0.0
                    self.change_state(RobotState.GRIP_ENGAGE)

        elif self.state == RobotState.GRIP_ENGAGE:
            d[CMD.GRIP_OPEN] = 0.0 
            if elapsed > 1.0: self.change_state(RobotState.SWING_DOWN)

        elif self.state == RobotState.SWING_DOWN:
            if self.is_ready(1): self.change_state(RobotState.ROTATE_TO_TARGET)

        elif self.state == RobotState.ROTATE_TO_TARGET:
            d[CMD.TOOL_ROT_TARGET] = float(self.current_brick['theta'])
            if self.is_ready(1) or elapsed > 1.5: self.change_state(RobotState.LOWER_AND_PLACE)

        elif self.state == RobotState.LOWER_AND_PLACE:
            d[CMD.TOOL_ROT_TARGET] = float(self.current_brick['theta'])
            if self.is_ready(3) and self.is_ready(4):
                if elapsed > 2.0: self.change_state(RobotState.RELEASE_BRICK)

        elif self.state == RobotState.RELEASE_BRICK:
            d[CMD.TOOL_ROT_TARGET] = float(self.current_brick['theta'])
            d[CMD.GRIP_OPEN] = 1.0 
            if elapsed > 1.0:
                self.change_state(RobotState.LIFT_UP)

        elif self.state == RobotState.LIFT_UP:
            d[CMD.GRIP_OPEN] = 1.0 
            d[CMD.TOOL_ROT_TARGET] = float(self.current_brick['theta'])
            lift_z = float(self.current_brick['z']) + self.Z_SAFETY_MARGIN
            d[CMD.ZL_TARGET] = lift_z; d[CMD.ZE_TARGET] = lift_z
            if self.is_ready(3) and self.is_ready(4):
                self.change_state(RobotState.ROTATE_RESET)

        elif self.state == RobotState.ROTATE_RESET:
            d[CMD.GRIP_OPEN] = 0.0
            d[CMD.TOOL_ROT_TARGET] = 0.0 
            if self.is_ready(1) or elapsed > 1.5: self.change_state(RobotState.SWING_UP_RESET)

        elif self.state == RobotState.SWING_UP_RESET:
            if self.is_ready(1):
                if self.brick_queue:
                    self.current_brick = self.brick_queue.pop(0)
                    self.change_state(RobotState.NAVIGATION)
                else: self.change_state(RobotState.IDLE)

        msg.data = d
        self.tool_pub.publish(msg); self.xax_pub.publish(msg); self.z_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BIMCoordinator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()