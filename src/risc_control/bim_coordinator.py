import rclpy
from rclpy.node import Node
from enum import Enum
import csv
import os
import time
from std_msgs.msg import Float32MultiArray, Bool

# ---------------------------------------------------------------------------
# bus index definitions
# ---------------------------------------------------------------------------
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
    CONV_CHECK        = 15 
    LIFT_RAMP_READY   = 16

class STAT:
    HW_ID            = 0
    POS_ALPHA        = 1
    POS_BETA         = 2  
    GRIPPER_DETECT   = 3
    PROX_SENSOR      = 4
    TASK_COMPLETE    = 5
    LIMIT_MIN_HIT    = 6
    LIMIT_MAX_HIT    = 7
    ESTOP_HIT        = 8 
    RAMP_CONTACT     = 9
    BRICK_PRESENT    = 10

class RobotState(Enum):
    IDLE = 0            
    HOMING = 1           
    NAV_AND_FEED = 2      
    GRIP_ENGAGE = 3       
    SWING_TO_PUMP = 4
    PUMP_DELIVERY = 12     
    SWING_TO_FINAL = 13          
    ROTATE_TO_TARGET = 5 
    LOWER_AND_PLACE = 6
    X_OVERSHOOT = 11 
    RELEASE_BRICK = 7
    LIFT_UP = 8
    ROTATE_RESET = 9    
    SWING_UP_RESET = 10  
    PAUSED = 98
    SAFETY_STOP = 99     

class BIMCoordinator(Node):
    def __init__(self):
        super().__init__('bim_coordinator')

        # calibrated constants
        self.TARGET_SWING_UP = 248.0
        self.TARGET_SWING_PUMP = 266.0
        self.TARGET_SWING_PUMP_END = 275.5
        self.TARGET_SWING_DOWN = 68.9  
        self.ROTATE_HOME_RAW = 268.0
        self.Z_SAFETY_MARGIN = 60.0  
        
        # CSV written by HMI at <ws>/scripts/output_csvs/current_build.csv
        ws_root = os.environ.get('RISC_WS', '/risc_ws')
        self.csv_path = os.path.join(ws_root, 'scripts', 'output_csvs', 'current_build.csv')
        
        self.brick_queue   = []
        self.current_brick = None
        self.bricks_placed = 0
        self.total_bricks  = 0
        self.state = RobotState.IDLE
        self.state_entry_time = time.time()
        self.status_data = {1: None, 2: None, 3: None, 4: None} 
        self.latches = {1: False, 2: False, 3: False, 4: False}
        self.in_manual_mode  = False
        self.last_tool_rot   = self.ROTATE_HOME_RAW
        self.last_tool_swing = self.TARGET_SWING_UP
        self.paused_state = None
        self.paused_tool_rot   = 0.0   # toolhead targets held during pause
        self.paused_tool_swing = 0.0
        self.paused_elapsed_offset = 0.0  # elapsed time accumulated before pause
        self.pause_entry_time = None
        self.anchors = {'x': 0.0, 'z': 0.0}

        # publishers
        self.tool_pub = self.create_publisher(Float32MultiArray, 'risc_command', 10)
        self.xax_pub  = self.create_publisher(Float32MultiArray, 'xaxis_command', 10)
        self.z_pub    = self.create_publisher(Float32MultiArray, 'z_command', 10)
        self.ir_pub        = self.create_publisher(Bool, 'ir_signal', 10)
        self.fsm_state_pub = self.create_publisher(Float32MultiArray, 'coordinator_state', 10)
        
        # hmi subscriptions
        self.create_subscription(Float32MultiArray, 'hmi_manual_cmd', self.manual_cmd_cb, 10)
        self.create_subscription(Bool, 'hmi_start_trigger', self.hmi_start_cb, 10)
        self.create_subscription(Bool, 'hmi_pause_trigger', self.hmi_pause_cb, 10)
        self.create_subscription(Bool, 'hmi_cancel_trigger', self.hmi_cancel_cb, 10)
        
        # hardware status subscriptions
        self.create_subscription(Float32MultiArray, 'toolhead_status', lambda m: self.status_cb(m, 1), 10)
        self.create_subscription(Float32MultiArray, 'xaxis_status', lambda m: self.status_cb(m, 2), 10)
        self.create_subscription(Float32MultiArray, 'zl_status', lambda m: self.status_cb(m, 3), 10)
        self.create_subscription(Float32MultiArray, 'ze_status', lambda m: self.status_cb(m, 4), 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("BIM Coordinator: Relay Architecture Active.")

    def manual_cmd_cb(self, msg):

        self.in_manual_mode = True
        self.anchors['x'] = msg.data[CMD.X_LEAD_TARGET]
        self.anchors['z'] = msg.data[CMD.ZL_TARGET]

        self.xax_pub.publish(msg)
        self.z_pub.publish(msg)

    def hmi_cancel_cb(self, msg):
        if msg.data:
            self.get_logger().info("CANCEL received -- returning to IDLE")
            self.brick_queue   = []
            self.current_brick = None
            self.paused_state  = None
            self.bricks_placed = 0
            self.total_bricks  = 0
            self.change_state(RobotState.IDLE)

    def hmi_start_cb(self, msg):
        if msg.data:
            self.get_logger().info("START TRIGGER received.")
            self.in_manual_mode = False
            self.bricks_placed = 0
            self.load_csv()
            if self.brick_queue:
                self.total_bricks = len(self.brick_queue)
                self.get_logger().info(f"Starting build: {self.total_bricks} bricks.")
                self.change_state(RobotState.HOMING)
            else:
                self.get_logger().warn(f"CSV not found or empty at: {self.csv_path}")
    
    def hmi_pause_cb(self, msg):
        if msg.data and self.state not in [RobotState.IDLE, RobotState.PAUSED, RobotState.SAFETY_STOP]:
            self.paused_elapsed_offset = time.time() - self.state_entry_time
            self.pause_entry_time = time.time()
            self.paused_state = self.state

            self.paused_tool_rot   = self.last_tool_rot
            self.paused_tool_swing = self.last_tool_swing
            self.change_state(RobotState.PAUSED)
        elif not msg.data and self.state == RobotState.PAUSED:

            self.state = self.paused_state
            self.state_entry_time = time.time() - self.paused_elapsed_offset
            self.get_logger().info(f"Resumed from PAUSED -> {self.state.name}")

    def load_csv(self):
        if os.path.exists(self.csv_path):
            self.brick_queue = []
            with open(self.csv_path, mode='r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    row.update({k: float(v) for k, v in row.items() if k in ['x','y','z','theta','drop_offset','x_overshoot']})
                    self.brick_queue.append(row)
            self.get_logger().info(f"Loaded {len(self.brick_queue)} bricks.")

    def change_state(self, new_state):
        self.state = new_state
        self.state_entry_time = time.time()
        for hw_id in self.latches: self.latches[hw_id] = False

    def is_ready(self, hw_id):
        if self.status_data.get(hw_id) is None: return False
        if self.status_data[hw_id][STAT.TASK_COMPLETE] > 0.5:
            self.latches[hw_id] = True
            return True
        return False

    def status_cb(self, msg, hw_id): self.status_data[hw_id] = msg.data

    def control_loop(self):
        msg = Float32MultiArray()
        d = [0.0] * 17 
        d[CMD.STATE_REQUEST] = float(self.state.value)
        
        # lift dispatch
        if self.status_data[2]:
            ramp_is_closed = self.status_data[2][STAT.RAMP_CONTACT] > 0.5
            d[CMD.LIFT_RAMP_READY] = 1.0 if ramp_is_closed else 0.0
            belt_is_idle = (self.status_data[2][STAT.PROX_SENSOR] + 
                            self.status_data[2][STAT.BRICK_PRESENT]) < 0.5 
            d[CMD.CONV_CHECK] = 1.0 if belt_is_idle else 0.0

            # forward IR to ZEbox lightstack
            if len(self.status_data[2]) > 11:
                ir_msg = Bool()
                ir_msg.data = self.status_data[2][11] > 0.5
                self.ir_pub.publish(ir_msg)
        
        elapsed = time.time() - self.state_entry_time

        # pre-calculate brick targets
        x_target = 0.0
        z_target = 0.0
        y_val = 0.0
        
        if self.current_brick:
            offset = self.current_brick['drop_offset']
            y_val = self.current_brick['y']
            

            if self.state in [RobotState.X_OVERSHOOT, RobotState.RELEASE_BRICK]:
                x_target = self.current_brick['x']
                z_target = self.current_brick['z'] + offset
            elif self.state == RobotState.LOWER_AND_PLACE:
                x_target = self.current_brick['x'] + offset
                z_target = self.current_brick['z'] + offset
            else:
                x_target = self.current_brick['x'] + offset
                z_target = self.current_brick['z'] + self.Z_SAFETY_MARGIN

        # ---------------------------------------------------------------------------
        # state machine
        # ---------------------------------------------------------------------------
        match self.state:
            case RobotState.IDLE:
                d[CMD.TOOL_ROT_TARGET] = self.ROTATE_HOME_RAW
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_UP
                if self.in_manual_mode:
                    d[CMD.X_LEAD_TARGET] = self.anchors['x']
                    d[CMD.ZL_TARGET] = self.anchors['z']
                    d[CMD.ZE_TARGET] = self.anchors['z']
                else:
                    pass
                if self.brick_queue: self.change_state(RobotState.HOMING)

            case RobotState.HOMING:
                d[CMD.TOOL_ROT_TARGET] = self.ROTATE_HOME_RAW
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_UP
                if elapsed < 3.0:
                    # buzzer phase -- hold all axes
                    d[CMD.TRIGGER_HOMING] = 1.0
                    d[CMD.X_LEAD_TARGET]  = 0.0
                    d[CMD.ZL_TARGET]      = 0.0
                    d[CMD.ZE_TARGET]      = 0.0
                    d[CMD.WHEEL_FL_VEL]   = 0.0
                    d[CMD.WHEEL_FE_VEL]   = 0.0
                else:
                    # homing phase
                    d[CMD.TRIGGER_HOMING] = 0.0
                    d[CMD.X_LEAD_TARGET]  = -1000.0
                    d[CMD.ZL_TARGET]      = -1000.0
                    d[CMD.ZE_TARGET]      = -1000.0
                    d[CMD.WHEEL_FL_VEL]   = 0.0
                    d[CMD.WHEEL_FE_VEL]   = 0.0

                    if elapsed > 3.5:
                        s2 = self.status_data[2]
                        s3 = self.status_data[3]
                        s4 = self.status_data[4]
                        x_ok  = s2 is not None and s2[STAT.LIMIT_MIN_HIT] > 0.5
                        zl_ok = s3 is not None and s3[STAT.LIMIT_MIN_HIT] > 0.5
                        ze_ok = s4 is not None and s4[STAT.LIMIT_MIN_HIT] > 0.5

                        if int(elapsed * 2) % 2 == 0:
                            self.get_logger().info(
                                f"Homing check -- X:{x_ok}(data={'yes' if s2 else 'NO'}) "
                                f"ZL:{zl_ok}(data={'yes' if s3 else 'NO'}) "
                                f"ZE:{ze_ok}(data={'yes' if s4 else 'NO'})"
                            )
                        if x_ok and zl_ok and ze_ok and self.brick_queue:
                            self.current_brick = self.brick_queue.pop(0)
                            self.change_state(RobotState.NAV_AND_FEED)

            case RobotState.NAV_AND_FEED:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_UP
                if self.status_data[2] and self.status_data[2][STAT.PROX_SENSOR] > 0.5:
                    d[CMD.EXTRACTOR_ON] = 1.0; d[CMD.GRIP_OPEN] = 1.0     
                if self.status_data[1] and self.status_data[1][STAT.GRIPPER_DETECT] > 0.5:
                    self.change_state(RobotState.GRIP_ENGAGE)

            case RobotState.GRIP_ENGAGE:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_UP
                d[CMD.GRIP_OPEN] = 0.0
                if elapsed > 1.2 and self.is_ready(2) and self.is_ready(3) and self.is_ready(4):
                    self.change_state(RobotState.SWING_TO_PUMP)

            case RobotState.SWING_TO_PUMP:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_PUMP
                if self.is_ready(1): 
                    self.change_state(RobotState.PUMP_DELIVERY)

            case RobotState.PUMP_DELIVERY:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_PUMP_END
                d[CMD.ADHESIVE_ON] = 1.0
                if self.is_ready(1):
                    self.change_state(RobotState.SWING_TO_FINAL)

            case RobotState.SWING_TO_FINAL:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
                d[CMD.ADHESIVE_ON] = 0.0

                if self.is_ready(1) and elapsed > 1.0:
                    self.change_state(RobotState.ROTATE_TO_TARGET)

            case RobotState.ROTATE_TO_TARGET:
                if elapsed > 1.0:
                    d[CMD.TOOL_ROT_TARGET] = self.current_brick['theta']
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
                if self.is_ready(1) and elapsed > 2.0:
                    self.change_state(RobotState.LOWER_AND_PLACE)

            case RobotState.LOWER_AND_PLACE:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
                d[CMD.TOOL_ROT_TARGET] = self.current_brick['theta']
                # dwell until Z ready
                if self.is_ready(3) and self.is_ready(4) and elapsed > 1.5:
                    if self.current_brick['x_overshoot'] > 0.5:
                        self.change_state(RobotState.X_OVERSHOOT)
                    else:
                        self.change_state(RobotState.RELEASE_BRICK)

            case RobotState.X_OVERSHOOT:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
                d[CMD.TOOL_ROT_TARGET] = self.current_brick['theta']
                if self.is_ready(2) and elapsed > 3.0: 
                    self.change_state(RobotState.RELEASE_BRICK)

            case RobotState.RELEASE_BRICK:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
                d[CMD.TOOL_ROT_TARGET] = self.current_brick['theta']
                d[CMD.GRIP_OPEN] = 1.0 
                if elapsed > 1.0: self.change_state(RobotState.LIFT_UP)

            case RobotState.LIFT_UP:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
                d[CMD.TOOL_ROT_TARGET] = self.current_brick['theta']
                d[CMD.GRIP_OPEN] = 1.0
                # wait for Z up + clearance
                if self.is_ready(3) and self.is_ready(4) and elapsed > 1.0:
                    self.change_state(RobotState.ROTATE_RESET)

            case RobotState.ROTATE_RESET:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_DOWN
                d[CMD.TOOL_ROT_TARGET] = 0.0

                if self.is_ready(1) and elapsed > 0.5:
                    self.change_state(RobotState.SWING_UP_RESET)

            case RobotState.SWING_UP_RESET:
                d[CMD.TOOL_SWING_TARGET] = self.TARGET_SWING_UP
                if (self.is_ready(1) or elapsed >= 1.5) and elapsed >= 1.0:
                    if self.brick_queue:
                        self.bricks_placed += 1
                        self.current_brick = self.brick_queue.pop(0)
                        self.change_state(RobotState.NAV_AND_FEED)
                    else:
                        self.bricks_placed += 1
                        self.change_state(RobotState.IDLE)

            case RobotState.PAUSED:
                # hold last toolhead position
                d[CMD.STATE_REQUEST]    = float(RobotState.PAUSED.value)
                d[CMD.TOOL_ROT_TARGET]  = self.paused_tool_rot
                d[CMD.TOOL_SWING_TARGET]= self.paused_tool_swing

            case RobotState.SAFETY_STOP:
                d = [0.0] * 17; d[CMD.STATE_REQUEST] = 99.0

        # final assignment
        if self.state not in [RobotState.HOMING, RobotState.PAUSED, RobotState.SAFETY_STOP]:
            d[CMD.X_LEAD_TARGET] = x_target
            d[CMD.ZL_TARGET] = z_target
            d[CMD.ZE_TARGET] = z_target
            d[CMD.WHEEL_FL_VEL] = y_val; d[CMD.WHEEL_FE_VEL] = y_val
        
        # track toolhead targets for pause
        if self.state not in [RobotState.PAUSED, RobotState.IDLE]:
            self.last_tool_rot   = d[CMD.TOOL_ROT_TARGET]
            self.last_tool_swing = d[CMD.TOOL_SWING_TARGET]
        msg.data = d
        # publish fsm state
        state_msg = Float32MultiArray()
        state_msg.data = [float(self.state.value), float(self.bricks_placed), float(self.total_bricks)]
        self.fsm_state_pub.publish(state_msg)
        # toolhead always published, x/z only during active build
        self.tool_pub.publish(msg)
        if self.state != RobotState.IDLE:
            self.xax_pub.publish(msg)
            self.z_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BIMCoordinator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()