#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# Setări pentru terminal (pentru a citi tastele fără a apăsa Enter)
settings = termios.tcgetattr(sys.stdin)

msg = """
--------------------------------------------------
Control UR5 End-Effector (6DOF)
--------------------------------------------------
Taste pentru control:

Translație (Liniar):
   W / S :  +X / -X (Înainte / Înapoi)
   A / D :  +Y / -Y (Stânga / Dreapta)
   Q / E :  +Z / -Z (Sus / Jos)

Rotație (Angular):
   U / O :  +Roll / -Roll (Ruliu)
   I / K :  +Pitch / -Pitch (Tangaj)
   J / L :  +Yaw / -Yaw (Girație)

Altele:
   1 / 2 : Crește / Scade viteza liniară
   3 / 4 : Crește / Scade viteza unghiulară
   SPACE : STOP (Viteze zero)
   CTRL-C: Ieșire
--------------------------------------------------
"""

move_bindings = {
    'w': (1, 0, 0, 0, 0, 0), 's': (-1, 0, 0, 0, 0, 0),
    'a': (0, 1, 0, 0, 0, 0), 'd': (0, -1, 0, 0, 0, 0),
    'q': (0, 0, 1, 0, 0, 0), 'e': (0, 0, -1, 0, 0, 0),
    'u': (0, 0, 0, 1, 0, 0), 'o': (0, 0, 0, -1, 0, 0),
    'i': (0, 0, 0, 0, 1, 0), 'k': (0, 0, 0, 0, -1, 0),
    'j': (0, 0, 0, 0, 0, 1), 'l': (0, 0, 0, 0, 0, -1),
}

speed_bindings = {
    '1': (1.1, 1.0), '2': (0.9, 1.0),
    '3': (1.0, 1.1), '4': (1.0, 0.9),
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class UR5KeyboardController(Node):
    def __init__(self):
        super().__init__('ur5_keyboard_control')
        
        # Parametri configurabili
        self.declare_parameter('topic_name', '/servo_node/delta_twist_cmds')
        self.declare_parameter('frame_id', 'base_link') # Schimbă în 'base' dacă nu merge
        
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(TwistStamped, topic_name, 10)
        
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        
        self.x = 0.0; self.y = 0.0; self.z = 0.0
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
        
        self.get_logger().info(f"Nod pornit. Publicare pe: {topic_name}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")
        self.get_logger().info("Dacă folosești Gazebo, nu uita: --ros-args -p use_sim_time:=true")
        print(msg)

    def run(self):
        try:
            while rclpy.ok():
                key = get_key()
                if key in move_bindings.keys():
                    self.x = move_bindings[key][0]
                    self.y = move_bindings[key][1]
                    self.z = move_bindings[key][2]
                    self.roll = move_bindings[key][3]
                    self.pitch = move_bindings[key][4]
                    self.yaw = move_bindings[key][5]
                elif key in speed_bindings.keys():
                    self.linear_speed *= speed_bindings[key][0]
                    self.angular_speed *= speed_bindings[key][1]
                    print(f"Viteze: Lin={self.linear_speed:.2f}, Ang={self.angular_speed:.2f}")
                elif key == ' ':
                    self.x = 0.0; self.y = 0.0; self.z = 0.0
                    self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
                elif key == '\x03': # CTRL+C
                    break
                # NOTA: Nu resetam la 0 automat aici pentru a permite miscarea continua 
                # cat timp nu se apasa SPACE, similar cu teleop_twist_keyboard standard.
                # Daca vrei "dead-man switch" (sa se opreasca cand iei mana), 
                # decomenteaza liniile de mai jos, dar va fi sacadat in terminal simplu.
                # else:
                #    self.x = 0.0; self.y = 0.0; self.z = 0.0
                #    self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0

                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = self.frame_id

                twist_msg.twist.linear.x = self.x * self.linear_speed
                twist_msg.twist.linear.y = self.y * self.linear_speed
                twist_msg.twist.linear.z = self.z * self.linear_speed
                twist_msg.twist.angular.x = self.roll * self.angular_speed
                twist_msg.twist.angular.y = self.pitch * self.angular_speed
                twist_msg.twist.angular.z = self.yaw * self.angular_speed

                self.publisher_.publish(twist_msg)

        except Exception as e:
            print(e)

        finally:
            twist_msg = TwistStamped()
            twist_msg.header.frame_id = self.frame_id
            self.publisher_.publish(twist_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = UR5KeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()