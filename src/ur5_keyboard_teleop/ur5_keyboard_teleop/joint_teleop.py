import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
import threading
import time
import tkinter as tk
from math import cos, sin, pi

# --- UR5e Kinematic Parameters (DH) ---
# d: offset along z, a: length along x, alpha: twist x
# Units: Meters
d = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
a = np.array([0.0, -0.425, -0.3922, 0.0, 0.0, 0.0])
alpha = np.array([pi/2, 0.0, 0.0, pi/2, -pi/2, 0.0])

class UR5eKinematics:
    @staticmethod
    def get_t_mat(theta, d, a, alpha):
        """Standard DH Transformation Matrix"""
        ca, sa = cos(alpha), sin(alpha)
        ct, st = cos(theta), sin(theta)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    @staticmethod
    def forward_kinematics(q):
        """Returns T0_6 (End Effector Pose)"""
        T = np.eye(4)
        for i in range(6):
            T = T @ UR5eKinematics.get_t_mat(q[i], d[i], a[i], alpha[i])
        return T

    @staticmethod
    def compute_jacobian(q):
        """Computes the 6x6 Geometric Jacobian"""
        J = np.zeros((6, 6))
        T = np.eye(4)
        z_prev = np.array([0, 0, 1]) # Base z-axis
        p_prev = np.array([0, 0, 0]) # Base position
        
        # We need the end-effector position
        T_final = UR5eKinematics.forward_kinematics(q)
        p_ee = T_final[:3, 3]

        transforms = []
        for i in range(6):
            transforms.append(T)
            T = T @ UR5eKinematics.get_t_mat(q[i], d[i], a[i], alpha[i])

        for i in range(6):
            T_curr = transforms[i]
            z_curr = T_curr[:3, 2] # z-axis of current frame
            p_curr = T_curr[:3, 3] # origin of current frame

            # Linear velocity component (v = w x r)
            J[:3, i] = np.cross(z_curr, p_ee - p_curr)
            # Angular velocity component
            J[3:, i] = z_curr
            
        return J

class VirtualJoystickGUI:
    def __init__(self, update_callback):
        self.root = tk.Tk()
        self.root.title("UR5e Virtual Joystick")
        self.root.geometry("400x350")
        self.update_callback = update_callback
        
        # Velocity State [vx, vy, vz, wx, wy, wz]
        self.cmd_vel = np.zeros(6)

        # Instructions
        tk.Label(self.root, text="Click & Drag Blue/Red dots to move").pack(pady=5)

        # Frame for Joysticks
        frame = tk.Frame(self.root)
        frame.pack()

        # XY Plane Joystick
        self.cv1 = tk.Canvas(frame, width=150, height=150, bg="white", highlightthickness=1, highlightbackground="#ccc")
        self.cv1.grid(row=0, column=0, padx=10)
        self.cv1.create_line(75, 0, 75, 150, fill="#eee")
        self.cv1.create_line(0, 75, 150, 75, fill="#eee")
        self.dot1 = self.cv1.create_oval(65, 65, 85, 85, fill="blue")
        tk.Label(frame, text="XY Position").grid(row=1, column=0)

        # Z / Yaw Joystick
        self.cv2 = tk.Canvas(frame, width=150, height=150, bg="white", highlightthickness=1, highlightbackground="#ccc")
        self.cv2.grid(row=0, column=1, padx=10)
        self.cv2.create_line(75, 0, 75, 150, fill="#eee")
        self.cv2.create_line(0, 75, 150, 75, fill="#eee")
        self.dot2 = self.cv2.create_oval(65, 65, 85, 85, fill="red")
        tk.Label(frame, text="Z Height (Up/Down) / Yaw (L/R)").grid(row=1, column=1)

        # Bindings
        self.cv1.bind("<B1-Motion>", self.on_move_xy)
        self.cv1.bind("<ButtonRelease-1>", self.on_release_xy)
        self.cv2.bind("<B1-Motion>", self.on_move_z_yaw)
        self.cv2.bind("<ButtonRelease-1>", self.on_release_z_yaw)

        # Stop Button
        tk.Button(self.root, text="EMERGENCY STOP (Space)", command=self.stop_all, bg="red", fg="white").pack(pady=20)
        self.root.bind("<space>", lambda e: self.stop_all())

        # Poll loop
        self.root.after(50, self.loop)

    def stop_all(self):
        self.cmd_vel = np.zeros(6)
        self.update_ui()

    def on_move_xy(self, event):
        x = (event.x - 75) / 75.0
        y = -(event.y - 75) / 75.0 # Inverted Y for screen coords
        # Limit speed
        x = np.clip(x, -1, 1) * 0.2  # Max 0.2 m/s
        y = np.clip(y, -1, 1) * 0.2
        self.cmd_vel[0] = y  # Robot X is usually Forward
        self.cmd_vel[1] = -x # Robot Y is usually Left
        self.update_ui(event.x, event.y, None, None)

    def on_move_z_yaw(self, event):
        yaw = -(event.x - 75) / 75.0
        z = -(event.y - 75) / 75.0
        yaw = np.clip(yaw, -1, 1) * 0.5 # Max 0.5 rad/s
        z = np.clip(z, -1, 1) * 0.2
        self.cmd_vel[2] = z
        self.cmd_vel[5] = yaw # Rz
        self.update_ui(None, None, event.x, event.y)

    def on_release_xy(self, event):
        self.cmd_vel[0] = 0.0
        self.cmd_vel[1] = 0.0
        self.update_ui(75, 75, None, None)

    def on_release_z_yaw(self, event):
        self.cmd_vel[2] = 0.0
        self.cmd_vel[5] = 0.0
        self.update_ui(None, None, 75, 75)

    def update_ui(self, x1=None, y1=None, x2=None, y2=None):
        if x1 is not None: self.cv1.coords(self.dot1, x1-10, y1-10, x1+10, y1+10)
        if x2 is not None: self.cv2.coords(self.dot2, x2-10, y2-10, x2+10, y2+10)

    def loop(self):
        self.update_callback(self.cmd_vel)
        self.root.after(50, self.loop)

class UR5TeleopNode(Node):
    def __init__(self):
        super().__init__('ur5_ik_teleop')
        
        # Publishers
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        
        self.current_joints = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0]) # Home guess
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        self.lock = threading.Lock()
        
        # Initial guess valid?
        self.joints_received = False

    def joint_state_cb(self, msg):
        # Map joint names to our order (msg order is not guaranteed)
        with self.lock:
            pos_map = {}
            for i, name in enumerate(msg.name):
                pos_map[name] = msg.position[i]
            
            try:
                self.current_joints = np.array([pos_map[n] for n in self.joint_names])
                self.joints_received = True
            except KeyError:
                pass # Wait for full message

    def control_loop(self, velocity_cmd):
        if not self.joints_received:
            # self.get_logger().warn("Waiting for joint states...")
            return

        # 1. Compute Jacobian at current config
        with self.lock:
            q = self.current_joints.copy()
        
        J = UR5eKinematics.compute_jacobian(q)

        # 2. Damped Least Squares (DLS) Inverse: J_dls = J^T * (J*J^T + k^2*I)^-1
        # This prevents the robot from going crazy at singularities
        damping = 0.01
        lambda_sq = damping ** 2
        J_inv = J.T @ np.linalg.inv(J @ J.T + lambda_sq * np.eye(6))

        # 3. Compute Joint Velocities: dq = J_inv * v
        dq = J_inv @ velocity_cmd

        # 4. Integrate to get next Position (dt = 0.05s from GUI loop)
        dt = 0.05
        q_next = q + dq * dt

        # 5. Publish
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = q_next.tolist()
        point.time_from_start.nanosec = int(dt * 1e9) # Reach point in dt
        msg.points.append(point)
        self.traj_pub.publish(msg)

def main():
    rclpy.init()
    node = UR5TeleopNode()
    
    # Run ROS in a separate thread so Tkinter can own the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Callback wrapper
    def send_cmd(vel):
        node.control_loop(vel)

    # Start GUI
    try:
        gui = VirtualJoystickGUI(send_cmd)
        gui.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()