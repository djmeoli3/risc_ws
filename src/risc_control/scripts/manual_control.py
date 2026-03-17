import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class BrickBotController(Node):
    def __init__(self):
        super().__init__('brickbot_terminal')
        
        #publishers for all 4 Teensy Nodes
        self.xaxis_pub = self.create_publisher(String, 'xaxis_cmd', 10)
        self.toolhead_pub = self.create_publisher(String, 'toolhead_cmd', 10)
        self.ze_pub = self.create_publisher(String, 'ze_cmd', 10)
        self.zl_pub = self.create_publisher(String, 'zl_cmd', 10)
        
        print("BrickBot Manual Control Initialized")
        print("-" * 40)
        print("X-AXIS:   m[mm], f(conveyor), h(home), b(bounce)")
        print("TOOLHEAD: s[1-up, 0-down], r[deg], e[pwm], g[1-open, 0-closed]")
        print("Z-POSTS:  z[mm] (vertical move for both)")
        print("GLOBAL:   x (stop all), exit")
        print("-" * 40)

    def run(self):
        while rclpy.ok():
            try:
                user_input = input("Enter Command: ").strip().lower()
                if not user_input: continue
                
                msg = String()
                msg.data = user_input

                # 1.Z-Axis Sync (Broadcast to ze and zl)
                if user_input.startswith('z'):
                    self.ze_pub.publish(msg)
                    self.zl_pub.publish(msg)
                    print(f"Sent Synchronized Z-Move: {user_input}")

                # 2.X-Axis Routing
                elif user_input.startswith(('m', 'f', 'h', 'b')):
                    self.xaxis_pub.publish(msg)

                # 3.Toolhead Routing
                elif user_input.startswith(('s', 'r', 'e', 'g')):
                    self.toolhead_pub.publish(msg)

                # 4.Global Stop (Broadcast to ALL)
                elif user_input == 'x':
                    self.xaxis_pub.publish(msg)
                    self.toolhead_pub.publish(msg)
                    self.ze_pub.publish(msg)
                    self.zl_pub.publish(msg)
                    print("!!! GLOBAL EMERGENCY STOP !!!")

                elif user_input == 'exit':
                    break
                else:
                    print("Unknown Command Prefix")
                    
            except EOFError:
                break

def main():
    rclpy.init()
    node = BrickBotController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()