import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk
from math import cos, sin, pi, sqrt, atan2

# --- Modern Color Palette (Nord-ish) ---
COLORS = {
    "bg": "#2E3440",         # Dark Slate Background
    "panel_bg": "#3B4252",   # Slightly Lighter Panel
    "text": "#ECEFF4",       # Off-White Text
    "subtext": "#D8DEE9",    # Dimmer Text
    "accent": "#88C0D0",     # Cyan/Blue Accent
    "btn_primary": "#81A1C1",# Muted Blue Button
    "btn_success": "#A3BE8C",# Muted Green Button
    "btn_danger": "#BF616A", # Muted Red Button
    "input_bg": "#4C566A",   # Input Field Background
    "joy_track": "#434C5E",  # Joystick Background
    "joy_dot": "#ECEFF4"     # Joystick Knob
}

FONT_HEADER = ("Segoe UI", 16, "bold")
FONT_SUB = ("Segoe UI", 12, "bold")
FONT_BODY = ("Segoe UI", 10)

# --- UR5e Kinematics Parameters ---
d_dh = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
a_dh = np.array([0.0, -0.425, -0.3922, 0.0, 0.0, 0.0])
alpha_dh = np.array([pi/2, 0.0, 0.0, pi/2, -pi/2, 0.0])

class Kinematics:
    @staticmethod
    def get_t_mat(theta, d, a, alpha):
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
        T = np.eye(4)
        for i in range(6):
            T = T @ Kinematics.get_t_mat(q[i], d_dh[i], a_dh[i], alpha_dh[i])
        return T

    @staticmethod
    def compute_jacobian(q):
        J = np.zeros((6, 6))
        T = np.eye(4)
        T_final = Kinematics.forward_kinematics(q)
        p_ee = T_final[:3, 3]
        transforms = []
        for i in range(6):
            transforms.append(T)
            T = T @ Kinematics.get_t_mat(q[i], d_dh[i], a_dh[i], alpha_dh[i])
        for i in range(6):
            T_curr = transforms[i]
            z_curr = T_curr[:3, 2]
            p_curr = T_curr[:3, 3]
            J[:3, i] = np.cross(z_curr, p_ee - p_curr)
            J[3:, i] = z_curr
        return J

    @staticmethod
    def rotation_matrix(roll, pitch, yaw):
        Rx = np.array([[1,0,0],[0,cos(roll),-sin(roll)],[0,sin(roll),cos(roll)]])
        Ry = np.array([[cos(pitch),0,sin(pitch)],[0,1,0],[-sin(pitch),0,cos(pitch)]])
        Rz = np.array([[cos(yaw),-sin(yaw),0],[sin(yaw),cos(yaw),0],[0,0,1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def solve_ik(target_pos, target_rpy, q_guess):
        q = q_guess.copy()
        R_des = Kinematics.rotation_matrix(*target_rpy)
        for _ in range(20):
            T_curr = Kinematics.forward_kinematics(q)
            p_curr = T_curr[:3, 3]
            R_curr = T_curr[:3, :3]
            e_pos = target_pos - p_curr
            R_err = R_des @ R_curr.T
            rx = (R_err[2,1] - R_err[1,2]) / 2.0
            ry = (R_err[0,2] - R_err[2,0]) / 2.0
            rz = (R_err[1,0] - R_err[0,1]) / 2.0
            e_rot = np.array([rx, ry, rz])
            error = np.hstack((e_pos, e_rot))
            if np.linalg.norm(error) < 1e-4: return q
            J = Kinematics.compute_jacobian(q)
            dq = J.T @ np.linalg.inv(J @ J.T + 0.001 * np.eye(6)) @ error
            q += dq
        return q

class ModernGUI:
    def __init__(self, vel_callback, pose_callback, home_callback):
        self.root = tk.Tk()
        self.root.title("UR5e Teleoperation Interface")
        self.root.geometry("900x500")
        self.root.configure(bg=COLORS["bg"])
        
        self.vel_callback = vel_callback
        self.pose_callback = pose_callback
        self.home_callback = home_callback

        # State
        self.cmd_vel = np.zeros(6)
        self.stick_left = [0, 0]
        self.stick_right = [0, 0]
        self.pitch_val = 0.0
        self.roll_val = 0.0

        # --- Layout ---
        # Main Container
        container = tk.Frame(self.root, bg=COLORS["bg"])
        container.pack(fill="both", expand=True, padx=20, pady=20)

        # Left Column: Velocity Control
        self.joy_frame = tk.Frame(container, bg=COLORS["bg"])
        self.joy_frame.pack(side="left", fill="both", expand=True, padx=(0, 20))
        
        # Right Column: Absolute Control
        self.panel_frame = tk.Frame(container, bg=COLORS["panel_bg"], width=320)
        self.panel_frame.pack(side="right", fill="y")
        self.panel_frame.pack_propagate(False) # Force width

        self.setup_joysticks()
        self.setup_control_panel()
        
        self.root.after(50, self.loop)

    def setup_joysticks(self):
        # Header
        tk.Label(self.joy_frame, text="Inverse Kinematics (Velocity Control)", 
                 fg=COLORS["accent"], bg=COLORS["bg"], font=FONT_HEADER).pack(anchor="w", pady=(0, 20))

        # Joystick Container
        sticks_cont = tk.Frame(self.joy_frame, bg=COLORS["bg"])
        sticks_cont.pack(pady=10)

        # Left Stick
        f1 = tk.Frame(sticks_cont, bg=COLORS["bg"])
        f1.pack(side="left", padx=30)
        tk.Label(f1, text="Vertical / Yaw", fg=COLORS["subtext"], bg=COLORS["bg"], font=FONT_BODY).pack(pady=5)
        self.cv_l = self.create_stick(f1)

        # Right Stick
        f2 = tk.Frame(sticks_cont, bg=COLORS["bg"])
        f2.pack(side="left", padx=30)
        tk.Label(f2, text="Planar (X / Y)", fg=COLORS["subtext"], bg=COLORS["bg"], font=FONT_BODY).pack(pady=5)
        self.cv_r = self.create_stick(f2)

        # Bindings
        self.cv_l.bind("<B1-Motion>", lambda e: self.on_move(e, "left"))
        self.cv_l.bind("<ButtonRelease-1>", lambda e: self.on_release(e, "left"))
        self.cv_r.bind("<B1-Motion>", lambda e: self.on_move(e, "right"))
        self.cv_r.bind("<ButtonRelease-1>", lambda e: self.on_release(e, "right"))

        # Sliders
        sliders_frame = tk.Frame(self.joy_frame, bg=COLORS["bg"])
        sliders_frame.pack(fill="x", pady=30)
        
        tk.Label(sliders_frame, text="Fine Adjustments", fg=COLORS["subtext"], bg=COLORS["bg"], font=FONT_SUB).pack(anchor="w")
        
        self.create_slider(sliders_frame, "Pitch (Wrist Tilt)", self.update_vel, "p")
        self.create_slider(sliders_frame, "Roll (Wrist Rotate)", self.update_vel, "r")

    def create_stick(self, parent):
        # Draw a nicer looking joystick
        size = 160
        center = size // 2
        cv = tk.Canvas(parent, width=size, height=size, bg=COLORS["bg"], highlightthickness=0)
        cv.pack()
        # Background track
        cv.create_oval(10, 10, size-10, size-10, fill=COLORS["joy_track"], outline="")
        # Crosshairs
        cv.create_line(center, 10, center, size-10, fill=COLORS["input_bg"], width=2)
        cv.create_line(10, center, size-10, center, fill=COLORS["input_bg"], width=2)
        # Handle knob
        cv.create_oval(center-15, center-15, center+15, center+15, fill=COLORS["joy_dot"], outline="", tags="dot")
        return cv

    def create_slider(self, parent, label, cmd, tag):
        f = tk.Frame(parent, bg=COLORS["bg"])
        f.pack(fill="x", pady=5)
        tk.Label(f, text=label, fg=COLORS["text"], bg=COLORS["bg"], width=15, anchor="w", font=FONT_BODY).pack(side="left")
        s = tk.Scale(f, from_=-1, to=1, orient="horizontal", resolution=0.1, 
                     bg=COLORS["bg"], fg=COLORS["text"], troughcolor=COLORS["input_bg"],
                     activebackground=COLORS["accent"], highlightthickness=0, bd=0, command=cmd)
        s.pack(side="right", fill="x", expand=True)
        if tag == "p": self.scale_p = s
        else: self.scale_r = s

    def setup_control_panel(self):
        p = self.panel_frame
        pad_x = 20
        
        tk.Label(p, text="Absolute Control", font=FONT_HEADER, bg=COLORS["panel_bg"], fg=COLORS["text"]).pack(pady=(20, 10))

        # Home Button
        btn_home = tk.Button(p, text="GO HOME", bg=COLORS["btn_success"], fg=COLORS["bg"], 
                             font=FONT_SUB, relief="flat", padx=10, pady=5, cursor="hand2", command=self.home_callback)
        btn_home.pack(fill="x", padx=pad_x, pady=10)

        # Divider
        tk.Frame(p, height=1, bg=COLORS["input_bg"]).pack(fill="x", padx=10, pady=10)

        # Inputs
        tk.Label(p, text="Target Coordinates", bg=COLORS["panel_bg"], fg=COLORS["accent"], font=FONT_SUB).pack(pady=(10, 5))
        
        self.entries = {}
        labels = ["X (m)", "Y (m)", "Z (m)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
        defaults = ["0.4", "0.0", "0.4", "180.0", "0.0", "0.0"]

        grid_frame = tk.Frame(p, bg=COLORS["panel_bg"])
        grid_frame.pack(fill="x", padx=pad_x)

        for i, lab in enumerate(labels):
            row = tk.Frame(grid_frame, bg=COLORS["panel_bg"])
            row.pack(fill="x", pady=4)
            tk.Label(row, text=lab, width=10, anchor="w", bg=COLORS["panel_bg"], fg=COLORS["subtext"], font=FONT_BODY).pack(side="left")
            e = tk.Entry(row, bg=COLORS["input_bg"], fg=COLORS["text"], insertbackground="white", 
                         relief="flat", font=FONT_BODY)
            e.insert(0, defaults[i])
            e.pack(side="right", fill="x", expand=True, ipady=3)
            self.entries[lab] = e

        # Go Button
        btn_go = tk.Button(p, text="EXECUTE MOVE", bg=COLORS["btn_primary"], fg=COLORS["panel_bg"], 
                           font=FONT_SUB, relief="flat", padx=10, pady=5, cursor="hand2", command=self.trigger_move)
        btn_go.pack(fill="x", padx=pad_x, pady=20)

    def on_move(self, event, side):
        size = 160; center = size // 2; radius = 70
        
        # Calculate offset from center
        dx = event.x - center
        dy = event.y - center
        dist = sqrt(dx*dx + dy*dy)
        
        # Clamp to radius
        if dist > radius:
            scale = radius / dist
            dx *= scale
            dy *= scale
        
        # Normalized output (-1 to 1)
        nx = dx / radius
        ny = -(dy / radius) # Invert Y

        # Visual update
        vis_x = center + dx
        vis_y = center + dy

        if side == "left":
            self.cv_l.coords("dot", vis_x-15, vis_y-15, vis_x+15, vis_y+15)
            self.stick_left = [nx, ny]
        else:
            self.cv_r.coords("dot", vis_x-15, vis_y-15, vis_x+15, vis_y+15)
            self.stick_right = [nx, ny]
        self.update_vel()

    def on_release(self, event, side):
        c = 80 # Center of 160
        if side == "left":
            self.cv_l.coords("dot", c-15, c-15, c+15, c+15)
            self.stick_left = [0, 0]
        else:
            self.cv_r.coords("dot", c-15, c-15, c+15, c+15)
            self.stick_right = [0, 0]
        self.update_vel()

    def update_vel(self, _=None):
        vz = self.stick_left[1] * 0.25
        wz = -self.stick_left[0] * 0.5
        vx = self.stick_right[1] * 0.25
        vy = -self.stick_right[0] * 0.25
        wy = -self.scale_p.get() * 0.5
        wx = self.scale_r.get() * 0.5
        self.cmd_vel = np.array([vx, vy, vz, wx, wy, wz])

    def trigger_move(self):
        try:
            vals = [float(self.entries[k].get()) for k in self.entries]
            pos = np.array(vals[:3])
            rpy_deg = np.array(vals[3:])
            rpy_rad = np.radians(rpy_deg) 
            self.pose_callback(pos, rpy_rad)
        except ValueError:
            print("Invalid Input")

    def loop(self):
        self.vel_callback(self.cmd_vel)
        self.root.after(50, self.loop)

class UR5TeleopNode(Node):
    def __init__(self):
        super().__init__('ur5_teleop')
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.current_joints = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.lock = threading.Lock()
        self.joints_received = False
        self.mode = "VELOCITY"

    def joint_state_cb(self, msg):
        with self.lock:
            p_map = {n: p for n, p in zip(msg.name, msg.position)}
            try:
                self.current_joints = np.array([p_map[n] for n in self.joint_names])
                self.joints_received = True
            except KeyError: pass

    def send_velocity(self, vel_cmd):
        if not self.joints_received or self.mode == "MOVING": 
            if np.linalg.norm(vel_cmd) > 0.01: self.mode = "VELOCITY" 
            else: return

        with self.lock: q = self.current_joints.copy()
        J = Kinematics.compute_jacobian(q)
        J_inv = J.T @ np.linalg.inv(J @ J.T + 0.0001 * np.eye(6))
        dq = J_inv @ vel_cmd
        dt = 0.05
        q_next = q + dq * dt
        self.publish_traj(q_next, dt)

    def go_pose(self, pos, rpy):
        self.mode = "MOVING"
        self.get_logger().info(f"Moving to: {pos}")
        with self.lock: q_start = self.current_joints.copy()
        q_target = Kinematics.solve_ik(pos, rpy, q_start)
        self.publish_traj(q_target, 4.0)

    def go_home(self):
        self.mode = "MOVING"
        q_home = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]) 
        self.publish_traj(q_home, 5.0)

    def publish_traj(self, q, duration):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration))*1e9)
        msg.points.append(pt)
        self.traj_pub.publish(msg)

def main():
    rclpy.init()
    node = UR5TeleopNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        gui = ModernGUI(node.send_velocity, node.go_pose, node.go_home)
        gui.root.mainloop()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()