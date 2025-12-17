import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, tty, termios

MOVE_STEP = 0.1  # rad per key press

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

KEY_MAPPING = {
    'q': (0, MOVE_STEP),
    'a': (0, -MOVE_STEP),
    'w': (1, MOVE_STEP),
    's': (1, -MOVE_STEP),
    'e': (2, MOVE_STEP),
    'd': (2, -MOVE_STEP),
    'r': (3, MOVE_STEP),
    'f': (3, -MOVE_STEP),
    't': (4, MOVE_STEP),
    'g': (4, -MOVE_STEP),
    'y': (5, MOVE_STEP),
    'h': (5, -MOVE_STEP),
}

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key

class JointTeleop(Node):
    def __init__(self):
        super().__init__('ur5_joint_teleop')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.current_positions = [0.0] * 6
        self.get_logger().info("UR5 joint teleop started")

    def send_joint_delta(self, joint_index, delta):
        self.current_positions[joint_index] += delta
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointTeleop()

    print("""
Keyboard joint control:
-----------------------
q/a : shoulder_pan +/-
w/s : shoulder_lift +/-
e/d : elbow +/-
r/f : wrist_1 +/-
t/g : wrist_2 +/-
y/h : wrist_3 +/-
Ctrl+C to quit
""")

    try:
        while rclpy.ok():
            key = get_key()
            if key in KEY_MAPPING:
                idx, delta = KEY_MAPPING[key]
                node.send_joint_delta(idx, delta)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
