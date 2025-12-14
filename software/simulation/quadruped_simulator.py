"""
Nova-SM3 Quadruped Robot Simulator
Full robot simulation with keyboard control

Controls:
- Arrow Up/Down: Walk forward/backward
- Arrow Left/Right: Turn left/right
- Space: Stop
- W/A/S/D: Alternative movement controls
- Q: Quit

Features:
- Top-down view of robot
- Real-time IK for all 4 legs
- Gait coordination
- Joint angle visualization for all legs

Author: Quadruped Simulator
Date: 2025-12-12
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle, Rectangle, FancyArrow
import tkinter as tk
from tkinter import ttk
import sys
sys.path.append('c:\\Users\\valer\\OneDrive\\Desktop\\Robot\\dog_robot\\software\\python')

from forward_kinematics import forward_kinematics as FK
from inverse_kinematics import inverse_kinematics as IK

# Link lengths (mm)
L1 = 60.0
L2 = 105.0
L3 = 132.0


class GaitController:
    """Control walking gait for quadruped robot"""

    def __init__(self):
        # Gait parameters
        self.step_height = 40.0  # mm
        self.step_length = 80.0  # mm
        self.step_duration = 0.8  # seconds
        self.stride_length = 80.0  # Total stride

        # Phase offsets for trot gait
        # FR and RL move together, FL and RR move together
        self.phase_offsets = {
            'FL': 0.0,    # Front Left
            'FR': 0.5,    # Front Right (opposite phase)
            'RL': 0.5,    # Rear Left (opposite phase)
            'RR': 0.0     # Rear Right
        }

        # Current gait time
        self.gait_time = 0.0

    def get_foot_position(self, leg_name, velocity_x, velocity_y, angular_velocity, dt):
        """
        Calculate foot position for a leg based on velocities

        Args:
            leg_name: 'FL', 'FR', 'RL', 'RR'
            velocity_x: Forward velocity (mm/s)
            velocity_y: Lateral velocity (mm/s)
            angular_velocity: Yaw rate (rad/s)
            dt: Time step (s)

        Returns:
            (x, y, z) foot position relative to leg origin
        """
        # Check if robot is moving
        velocity_mag = np.sqrt(velocity_x**2 + velocity_y**2)
        is_moving = velocity_mag > 1.0 or abs(angular_velocity) > 0.01  # Threshold

        # Update gait time only when moving
        if is_moving:
            self.gait_time += dt

        # Get phase for this leg
        phase_offset = self.phase_offsets[leg_name]
        phase = ((self.gait_time / self.step_duration) + phase_offset) % 1.0

        # Default stance position (leg pointing backward with elbow back)
        x_default = -52.5
        z_default = -222.93

        # Y coordinate depends on leg side:
        # Left legs (FL, RL): y = +60 (outward to left)
        # Right legs (FR, RR): y = -60 (outward to right, MIRRORED!)
        if leg_name in ['FL', 'RL']:
            y_default = 60.0  # Left side
        else:  # FR, RR
            y_default = -60.0  # Right side (mirrored)

        # If not moving, return static stance position
        if not is_moving:
            return x_default, y_default, z_default

        # Calculate desired stride based on velocity
        stride_x = velocity_x * self.step_duration
        stride_y = velocity_y * self.step_duration

        # Limit stride to reasonable values
        max_stride = 100.0
        stride_mag = np.sqrt(stride_x**2 + stride_y**2)
        if stride_mag > max_stride:
            stride_x = stride_x / stride_mag * max_stride
            stride_y = stride_y / stride_mag * max_stride

        # Calculate foot position based on gait phase
        if phase < 0.5:
            # Swing phase - lift and move forward
            swing_progress = phase * 2  # Map to [0, 1]

            # Move from back to front
            x = x_default - stride_x/2 + stride_x * swing_progress
            y = y_default + stride_y/2 - stride_y * swing_progress

            # Lift foot
            z = z_default + self.step_height * np.sin(np.pi * swing_progress)
        else:
            # Stance phase - push backward
            stance_progress = (phase - 0.5) * 2  # Map to [0, 1]

            # Move from front to back
            x = x_default + stride_x/2 - stride_x * stance_progress
            y = y_default - stride_y/2 + stride_y * stance_progress

            # On ground
            z = z_default

        return x, y, z


class QuadrupedRobot:
    """Quadruped robot model"""

    def __init__(self, body_length=600.0, body_width=200.0):
        """
        Initialize quadruped robot

        Args:
            body_length (H): Distance between front and rear legs (mm)
            body_width (W): Distance between left and right legs (mm)
        """
        # Robot body dimensions (mm)
        self.body_length = body_length  # H: Front to rear distance
        self.body_width = body_width    # W: Left to right distance

        # Hip positions relative to body center (x, y)
        # Origin is at center of body
        self.leg_positions = {
            'FL': (self.body_length/2, self.body_width/2),    # Front Left: (+H/2, +W/2)
            'FR': (self.body_length/2, -self.body_width/2),   # Front Right: (+H/2, -W/2)
            'RL': (-self.body_length/2, self.body_width/2),   # Rear Left: (-H/2, +W/2)
            'RR': (-self.body_length/2, -self.body_width/2)   # Rear Right: (-H/2, -W/2)
        }

        # Robot pose in world frame (x, y, yaw)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # radians

        # Velocities
        self.velocity_x = 0.0  # mm/s (forward)
        self.velocity_y = 0.0  # mm/s (lateral)
        self.angular_velocity = 0.0  # rad/s (yaw rate)

        # Joint angles for all legs (theta1, theta2, theta3)
        self.joint_angles = {
            'FL': np.array([0, np.radians(30), np.radians(-30)]),
            'FR': np.array([0, np.radians(30), np.radians(-30)]),
            'RL': np.array([0, np.radians(30), np.radians(-30)]),
            'RR': np.array([0, np.radians(30), np.radians(-30)])
        }

        # Gait controller
        self.gait = GaitController()

    def update(self, dt):
        """Update robot state"""
        # Update position based on velocities
        dx = self.velocity_x * dt * np.cos(self.yaw) - self.velocity_y * dt * np.sin(self.yaw)
        dy = self.velocity_x * dt * np.sin(self.yaw) + self.velocity_y * dt * np.cos(self.yaw)
        dyaw = self.angular_velocity * dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw

        # Keep yaw in [-pi, pi]
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))

        # Update leg positions using gait controller
        for leg_name in ['FL', 'FR', 'RL', 'RR']:
            # Get desired foot position from gait
            x_foot, y_foot, z_foot = self.gait.get_foot_position(
                leg_name, self.velocity_x, self.velocity_y,
                self.angular_velocity, dt
            )

            # Calculate IK
            result = self.select_best_ik_solution(
                x_foot, y_foot, z_foot,
                self.joint_angles[leg_name],
                leg_name
            )

            if result:
                self.joint_angles[leg_name] = np.array(result)

    def select_best_ik_solution(self, x, y, z, current_angles, leg_name):
        """
        Select best IK solution based on minimum joint angle change

        Note: Right legs (FR, RR) are mirrored, so we need to:
        1. Mirror y coordinate for IK: y → -y
        2. Mirror theta1 result: theta1 → -theta1
        """
        # Mirror Y coordinate for right legs
        if leg_name in ['FR', 'RR']:
            y_ik = -y  # Mirror
        else:
            y_ik = y

        all_solutions = IK(x, y_ik, z, return_all=True)

        if not all_solutions or len(all_solutions) == 0:
            return None

        best_solution = None
        min_distance = float('inf')

        for theta1, theta2, theta3, error in all_solutions:
            # Mirror theta1 back for right legs
            if leg_name in ['FR', 'RR']:
                theta1 = -theta1

            distance = (abs(theta1 - current_angles[0]) +
                       abs(theta2 - current_angles[1]) +
                       abs(theta3 - current_angles[2]))

            if distance < min_distance:
                min_distance = distance
                best_solution = (theta1, theta2, theta3)

        return best_solution

    def get_foot_world_position(self, leg_name):
        """Get foot position in world frame"""
        # Get leg base position in body frame
        leg_base_x, leg_base_y = self.leg_positions[leg_name]

        # Get joint angles
        theta1, theta2, theta3 = self.joint_angles[leg_name]

        # Calculate foot position in leg frame
        x_leg, y_leg, z_leg = FK(theta1, theta2, theta3)

        # Transform to body frame
        x_body = leg_base_x + x_leg
        y_body = leg_base_y + y_leg

        # Transform to world frame
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)

        x_world = self.x + x_body * cos_yaw - y_body * sin_yaw
        y_world = self.y + x_body * sin_yaw + y_body * cos_yaw

        return x_world, y_world


class QuadrupedSimulatorGUI:
    """GUI for quadruped robot simulator"""

    def __init__(self, root):
        self.root = root
        self.root.title("Nova-SM3 Quadruped Robot Simulator")
        self.root.geometry("1600x900")

        # Body dimensions (can be configured)
        self.body_length = 600.0  # H: Front-rear distance (mm)
        self.body_width = 200.0   # W: Left-right distance (mm)

        # Robot
        self.robot = QuadrupedRobot(self.body_length, self.body_width)

        # Simulation parameters
        self.dt = 0.05  # 20 Hz
        self.running = False
        self.update_timer = None

        # Command velocities
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_angular = 0.0

        # Velocity limits
        self.max_linear_vel = 150.0  # mm/s
        self.max_angular_vel = np.radians(60)  # rad/s

        # Acceleration
        self.linear_accel = 300.0  # mm/s^2
        self.angular_accel = np.radians(120)  # rad/s^2

        # Track which keys are currently pressed
        self.keys_pressed = set()

        self.setup_ui()
        self.bind_keys()
        self.start_simulation()

    def setup_ui(self):
        """Setup user interface"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel - Controls & Info
        control_frame = ttk.Frame(main_frame, width=300)
        control_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        control_frame.grid_propagate(False)

        # Title
        title_label = ttk.Label(control_frame, text="Nova-SM3 Simulator",
                               font=("Arial", 14, "bold"))
        title_label.pack(pady=10)

        # Controls help
        controls_frame = ttk.LabelFrame(control_frame, text="Keyboard Controls", padding=10)
        controls_frame.pack(fill=tk.X, pady=5)

        controls_text = """
↑ / W : Forward
↓ / S : Backward
← / A : Turn Left
→ / D : Turn Right
Space : Stop
Q : Quit
"""
        ttk.Label(controls_frame, text=controls_text, font=("Courier", 9),
                 justify=tk.LEFT).pack()

        # Body dimensions
        body_frame = ttk.LabelFrame(control_frame, text="Body Dimensions (mm)", padding=10)
        body_frame.pack(fill=tk.X, pady=5)

        # H (Front-Rear distance)
        ttk.Label(body_frame, text="H (Front-Rear):", font=("Arial", 9)).grid(row=0, column=0, sticky="w", pady=2)
        self.h_entry = ttk.Entry(body_frame, width=10, font=("Courier", 9))
        self.h_entry.grid(row=0, column=1, sticky="w", padx=5)
        self.h_entry.insert(0, str(int(self.body_length)))
        ttk.Label(body_frame, text="mm", font=("Arial", 9)).grid(row=0, column=2, sticky="w")

        # W (Left-Right distance)
        ttk.Label(body_frame, text="W (Left-Right):", font=("Arial", 9)).grid(row=1, column=0, sticky="w", pady=2)
        self.w_entry = ttk.Entry(body_frame, width=10, font=("Courier", 9))
        self.w_entry.grid(row=1, column=1, sticky="w", padx=5)
        self.w_entry.insert(0, str(int(self.body_width)))
        ttk.Label(body_frame, text="mm", font=("Arial", 9)).grid(row=1, column=2, sticky="w")

        # Apply button
        apply_btn = ttk.Button(body_frame, text="Apply Dimensions", command=self.apply_body_dimensions)
        apply_btn.grid(row=2, column=0, columnspan=3, pady=(5, 5))

        # Hip positions info
        self.hip_info_label = ttk.Label(body_frame, text="", font=("Courier", 7),
                                       foreground="gray", justify=tk.LEFT)
        self.hip_info_label.grid(row=3, column=0, columnspan=3, sticky="w")
        self.update_hip_info()

        # Robot state
        state_frame = ttk.LabelFrame(control_frame, text="Robot State", padding=10)
        state_frame.pack(fill=tk.X, pady=5)

        self.state_label = ttk.Label(state_frame, text="", font=("Courier", 9),
                                    justify=tk.LEFT)
        self.state_label.pack()

        # Command velocities
        cmd_frame = ttk.LabelFrame(control_frame, text="Command Velocities", padding=10)
        cmd_frame.pack(fill=tk.X, pady=5)

        self.cmd_label = ttk.Label(cmd_frame, text="", font=("Courier", 9),
                                  justify=tk.LEFT)
        self.cmd_label.pack()

        # Joint angles display
        joints_frame = ttk.LabelFrame(control_frame, text="Joint Angles (deg)", padding=10)
        joints_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.joints_text = tk.Text(joints_frame, height=12, width=35,
                                  font=("Courier", 8))
        self.joints_text.pack(fill=tk.BOTH, expand=True)

        # Right panel - Visualization
        viz_frame = ttk.Frame(main_frame)
        viz_frame.grid(row=0, column=1, sticky="nsew")

        # Create matplotlib figure with 4 rows
        self.fig = plt.Figure(figsize=(14, 14))

        # Row 1: Top-down world view (large)
        self.ax_top = self.fig.add_subplot(4, 1, 1)
        self.ax_top.set_title("Top View (World Frame)", fontweight='bold', fontsize=12)
        self.ax_top.set_xlabel("X (mm)")
        self.ax_top.set_ylabel("Y (mm)")
        self.ax_top.grid(True, alpha=0.3)
        self.ax_top.set_aspect('equal')

        # Row 2: XY projection plots (leg local frames) - FL, FR, RL, RR
        self.ax_xy_fl = self.fig.add_subplot(4, 4, 5)
        self.ax_xy_fl.set_title("FL - XY Projection", fontsize=9, fontweight='bold')
        self.ax_xy_fl.set_xlabel("X (mm)", fontsize=7)
        self.ax_xy_fl.set_ylabel("Y (mm)", fontsize=7)
        self.ax_xy_fl.grid(True, alpha=0.3)
        self.ax_xy_fl.set_aspect('equal')
        self.ax_xy_fl.tick_params(labelsize=6)

        self.ax_xy_fr = self.fig.add_subplot(4, 4, 6)
        self.ax_xy_fr.set_title("FR - XY Projection", fontsize=9, fontweight='bold')
        self.ax_xy_fr.set_xlabel("X (mm)", fontsize=7)
        self.ax_xy_fr.set_ylabel("Y (mm)", fontsize=7)
        self.ax_xy_fr.grid(True, alpha=0.3)
        self.ax_xy_fr.set_aspect('equal')
        self.ax_xy_fr.tick_params(labelsize=6)

        self.ax_xy_rl = self.fig.add_subplot(4, 4, 7)
        self.ax_xy_rl.set_title("RL - XY Projection", fontsize=9, fontweight='bold')
        self.ax_xy_rl.set_xlabel("X (mm)", fontsize=7)
        self.ax_xy_rl.set_ylabel("Y (mm)", fontsize=7)
        self.ax_xy_rl.grid(True, alpha=0.3)
        self.ax_xy_rl.set_aspect('equal')
        self.ax_xy_rl.tick_params(labelsize=6)

        self.ax_xy_rr = self.fig.add_subplot(4, 4, 8)
        self.ax_xy_rr.set_title("RR - XY Projection", fontsize=9, fontweight='bold')
        self.ax_xy_rr.set_xlabel("X (mm)", fontsize=7)
        self.ax_xy_rr.set_ylabel("Y (mm)", fontsize=7)
        self.ax_xy_rr.grid(True, alpha=0.3)
        self.ax_xy_rr.set_aspect('equal')
        self.ax_xy_rr.tick_params(labelsize=6)

        # Row 3: XZ projection plots (leg local frames) - FL, FR, RL, RR
        self.ax_xz_fl = self.fig.add_subplot(4, 4, 9)
        self.ax_xz_fl.set_title("FL - XZ Projection", fontsize=9, fontweight='bold')
        self.ax_xz_fl.set_xlabel("X (mm)", fontsize=7)
        self.ax_xz_fl.set_ylabel("Z (mm)", fontsize=7)
        self.ax_xz_fl.grid(True, alpha=0.3)
        self.ax_xz_fl.set_aspect('equal')
        self.ax_xz_fl.tick_params(labelsize=6)

        self.ax_xz_fr = self.fig.add_subplot(4, 4, 10)
        self.ax_xz_fr.set_title("FR - XZ Projection", fontsize=9, fontweight='bold')
        self.ax_xz_fr.set_xlabel("X (mm)", fontsize=7)
        self.ax_xz_fr.set_ylabel("Z (mm)", fontsize=7)
        self.ax_xz_fr.grid(True, alpha=0.3)
        self.ax_xz_fr.set_aspect('equal')
        self.ax_xz_fr.tick_params(labelsize=6)

        self.ax_xz_rl = self.fig.add_subplot(4, 4, 11)
        self.ax_xz_rl.set_title("RL - XZ Projection", fontsize=9, fontweight='bold')
        self.ax_xz_rl.set_xlabel("X (mm)", fontsize=7)
        self.ax_xz_rl.set_ylabel("Z (mm)", fontsize=7)
        self.ax_xz_rl.grid(True, alpha=0.3)
        self.ax_xz_rl.set_aspect('equal')
        self.ax_xz_rl.tick_params(labelsize=6)

        self.ax_xz_rr = self.fig.add_subplot(4, 4, 12)
        self.ax_xz_rr.set_title("RR - XZ Projection", fontsize=9, fontweight='bold')
        self.ax_xz_rr.set_xlabel("X (mm)", fontsize=7)
        self.ax_xz_rr.set_ylabel("Z (mm)", fontsize=7)
        self.ax_xz_rr.grid(True, alpha=0.3)
        self.ax_xz_rr.set_aspect('equal')
        self.ax_xz_rr.tick_params(labelsize=6)

        # Row 4: Joint angle plots (FL, FR, RL, RR)
        self.ax_angles_fl = self.fig.add_subplot(4, 4, 13)
        self.ax_angles_fl.set_title("FL - Joint Angles", fontsize=9, fontweight='bold')
        self.ax_angles_fl.set_ylim(-180, 180)
        self.ax_angles_fl.set_ylabel("Angle (deg)", fontsize=7)
        self.ax_angles_fl.grid(True, alpha=0.3)
        self.ax_angles_fl.tick_params(labelsize=6)

        self.ax_angles_fr = self.fig.add_subplot(4, 4, 14)
        self.ax_angles_fr.set_title("FR - Joint Angles", fontsize=9, fontweight='bold')
        self.ax_angles_fr.set_ylim(-180, 180)
        self.ax_angles_fr.grid(True, alpha=0.3)
        self.ax_angles_fr.tick_params(labelsize=6)

        self.ax_angles_rl = self.fig.add_subplot(4, 4, 15)
        self.ax_angles_rl.set_title("RL - Joint Angles", fontsize=9, fontweight='bold')
        self.ax_angles_rl.set_ylim(-180, 180)
        self.ax_angles_rl.grid(True, alpha=0.3)
        self.ax_angles_rl.tick_params(labelsize=6)

        self.ax_angles_rr = self.fig.add_subplot(4, 4, 16)
        self.ax_angles_rr.set_title("RR - Joint Angles", fontsize=9, fontweight='bold')
        self.ax_angles_rr.set_ylim(-180, 180)
        self.ax_angles_rr.grid(True, alpha=0.3)
        self.ax_angles_rr.tick_params(labelsize=6)

        self.fig.tight_layout(pad=2.0)

        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Configure grid weights
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # Joint angle history for plotting
        self.angle_history = {
            'FL': {'theta1': [], 'theta2': [], 'theta3': []},
            'FR': {'theta1': [], 'theta2': [], 'theta3': []},
            'RL': {'theta1': [], 'theta2': [], 'theta3': []},
            'RR': {'theta1': [], 'theta2': [], 'theta3': []}
        }

        # Foot position history for XY projection plots (in leg local frame)
        self.foot_xy_history = {
            'FL': {'x': [], 'y': []},
            'FR': {'x': [], 'y': []},
            'RL': {'x': [], 'y': []},
            'RR': {'x': [], 'y': []}
        }

        # Foot position history for XZ projection plots (in leg local frame)
        self.foot_xz_history = {
            'FL': {'x': [], 'z': []},
            'FR': {'x': [], 'z': []},
            'RL': {'x': [], 'z': []},
            'RR': {'x': [], 'z': []}
        }
        self.max_history = 100

    def update_hip_info(self):
        """Update hip positions display"""
        hip_info = f"Hip Positions:\n"
        hip_info += f"FL:(+{self.body_length/2:.0f},+{self.body_width/2:.0f})  "
        hip_info += f"FR:(+{self.body_length/2:.0f},-{self.body_width/2:.0f})\n"
        hip_info += f"RL:(-{self.body_length/2:.0f},+{self.body_width/2:.0f})  "
        hip_info += f"RR:(-{self.body_length/2:.0f},-{self.body_width/2:.0f})"
        self.hip_info_label.config(text=hip_info)

    def apply_body_dimensions(self):
        """Apply new body dimensions"""
        try:
            # Get new values from entry widgets
            new_h = float(self.h_entry.get())
            new_w = float(self.w_entry.get())

            # Validate inputs
            if new_h <= 0 or new_w <= 0:
                return

            # Update robot body dimensions
            self.body_length = new_h
            self.body_width = new_w
            self.robot.body_length = new_h
            self.robot.body_width = new_w

            # Update hip positions
            self.robot.leg_positions = {
                'FL': (new_h/2, new_w/2),
                'FR': (new_h/2, -new_w/2),
                'RL': (-new_h/2, new_w/2),
                'RR': (-new_h/2, -new_w/2)
            }

            # Update display
            self.update_hip_info()

        except ValueError:
            pass  # Invalid input, ignore

    def bind_keys(self):
        """Bind keyboard controls"""
        # Key press events
        self.root.bind('<KeyPress-Up>', lambda e: self.on_key_press('forward'))
        self.root.bind('<KeyPress-Down>', lambda e: self.on_key_press('backward'))
        self.root.bind('<KeyPress-Left>', lambda e: self.on_key_press('left'))
        self.root.bind('<KeyPress-Right>', lambda e: self.on_key_press('right'))
        self.root.bind('<KeyPress-space>', lambda e: self.on_key_press('stop'))

        self.root.bind('<KeyPress-w>', lambda e: self.on_key_press('forward'))
        self.root.bind('<KeyPress-s>', lambda e: self.on_key_press('backward'))
        self.root.bind('<KeyPress-a>', lambda e: self.on_key_press('left'))
        self.root.bind('<KeyPress-d>', lambda e: self.on_key_press('right'))
        self.root.bind('<KeyPress-q>', lambda e: self.root.quit())

        # Key release events
        self.root.bind('<KeyRelease-Up>', lambda e: self.on_key_release('forward'))
        self.root.bind('<KeyRelease-Down>', lambda e: self.on_key_release('backward'))
        self.root.bind('<KeyRelease-Left>', lambda e: self.on_key_release('left'))
        self.root.bind('<KeyRelease-Right>', lambda e: self.on_key_release('right'))

        self.root.bind('<KeyRelease-w>', lambda e: self.on_key_release('forward'))
        self.root.bind('<KeyRelease-s>', lambda e: self.on_key_release('backward'))
        self.root.bind('<KeyRelease-a>', lambda e: self.on_key_release('left'))
        self.root.bind('<KeyRelease-d>', lambda e: self.on_key_release('right'))

    def on_key_press(self, command):
        """Handle key press events"""
        if command == 'stop':
            self.keys_pressed.clear()
            self.cmd_vel_x = 0.0
            self.cmd_vel_y = 0.0
            self.cmd_angular = 0.0
        else:
            self.keys_pressed.add(command)
            self.update_command_from_keys()

    def on_key_release(self, command):
        """Handle key release events"""
        if command in self.keys_pressed:
            self.keys_pressed.remove(command)
        self.update_command_from_keys()

    def update_command_from_keys(self):
        """Update command velocities based on currently pressed keys"""
        # Reset commands
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_angular = 0.0

        # Apply commands for all pressed keys
        if 'forward' in self.keys_pressed:
            self.cmd_vel_x = self.max_linear_vel
        if 'backward' in self.keys_pressed:
            self.cmd_vel_x = -self.max_linear_vel
        if 'left' in self.keys_pressed:
            self.cmd_angular = self.max_angular_vel
        if 'right' in self.keys_pressed:
            self.cmd_angular = -self.max_angular_vel

    def update_velocities(self):
        """Smoothly update robot velocities with acceleration limits"""
        # Linear velocity
        vel_error = self.cmd_vel_x - self.robot.velocity_x
        max_change = self.linear_accel * self.dt
        if abs(vel_error) < max_change:
            self.robot.velocity_x = self.cmd_vel_x
        else:
            self.robot.velocity_x += np.sign(vel_error) * max_change

        # Lateral velocity
        vel_error = self.cmd_vel_y - self.robot.velocity_y
        if abs(vel_error) < max_change:
            self.robot.velocity_y = self.cmd_vel_y
        else:
            self.robot.velocity_y += np.sign(vel_error) * max_change

        # Angular velocity
        ang_error = self.cmd_angular - self.robot.angular_velocity
        max_ang_change = self.angular_accel * self.dt
        if abs(ang_error) < max_ang_change:
            self.robot.angular_velocity = self.cmd_angular
        else:
            self.robot.angular_velocity += np.sign(ang_error) * max_ang_change

    def start_simulation(self):
        """Start simulation loop"""
        self.running = True
        self.update_simulation()

    def update_simulation(self):
        """Update simulation state"""
        if not self.running:
            return

        # Update velocities
        self.update_velocities()

        # Update robot
        self.robot.update(self.dt)

        # Update displays
        self.update_state_display()
        self.update_visualization()

        # Schedule next update
        self.update_timer = self.root.after(int(self.dt * 1000), self.update_simulation)

    def update_state_display(self):
        """Update state text display"""
        state_text = f"""Position:
  X: {self.robot.x:8.1f} mm
  Y: {self.robot.y:8.1f} mm
  Yaw: {np.degrees(self.robot.yaw):6.1f}°

Velocity:
  Vx: {self.robot.velocity_x:7.1f} mm/s
  Vy: {self.robot.velocity_y:7.1f} mm/s
  Vyaw: {np.degrees(self.robot.angular_velocity):5.1f}°/s"""

        self.state_label.config(text=state_text)

        cmd_text = f"""Linear X: {self.cmd_vel_x:7.1f} mm/s
Linear Y: {self.cmd_vel_y:7.1f} mm/s
Angular: {np.degrees(self.cmd_angular):6.1f}°/s"""

        self.cmd_label.config(text=cmd_text)

        # Update joint angles display
        joints_text = ""
        for leg_name in ['FL', 'FR', 'RL', 'RR']:
            angles = self.robot.joint_angles[leg_name]
            joints_text += f"{leg_name}: θ1={np.degrees(angles[0]):6.1f}° "
            joints_text += f"θ2={np.degrees(angles[1]):6.1f}° "
            joints_text += f"θ3={np.degrees(angles[2]):6.1f}°\n"

        self.joints_text.delete('1.0', tk.END)
        self.joints_text.insert('1.0', joints_text)

    def update_visualization(self):
        """Update visualization plots"""
        # Clear top view
        self.ax_top.clear()
        self.ax_top.set_title("Top View (World Frame)", fontweight='bold')
        self.ax_top.set_xlabel("X (mm)")
        self.ax_top.set_ylabel("Y (mm)")
        self.ax_top.grid(True, alpha=0.3)
        self.ax_top.set_aspect('equal')

        # Draw robot body
        body_corners_local = np.array([
            [self.robot.body_length/2, self.robot.body_width/2],
            [self.robot.body_length/2, -self.robot.body_width/2],
            [-self.robot.body_length/2, -self.robot.body_width/2],
            [-self.robot.body_length/2, self.robot.body_width/2],
            [self.robot.body_length/2, self.robot.body_width/2]
        ])

        # Rotate and translate
        cos_yaw = np.cos(self.robot.yaw)
        sin_yaw = np.sin(self.robot.yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

        body_corners_world = np.dot(body_corners_local, rotation_matrix.T)
        body_corners_world[:, 0] += self.robot.x
        body_corners_world[:, 1] += self.robot.y

        self.ax_top.plot(body_corners_world[:, 0], body_corners_world[:, 1],
                        'b-', linewidth=3, label='Body')

        # Draw heading indicator
        arrow_length = 100
        arrow_dx = arrow_length * cos_yaw
        arrow_dy = arrow_length * sin_yaw
        self.ax_top.arrow(self.robot.x, self.robot.y, arrow_dx, arrow_dy,
                         head_width=30, head_length=20, fc='red', ec='red',
                         linewidth=2, label='Heading')

        # Draw legs and feet
        colors = {'FL': 'green', 'FR': 'blue', 'RL': 'orange', 'RR': 'purple'}

        for leg_name in ['FL', 'FR', 'RL', 'RR']:
            # Leg base position in world frame
            leg_base_x, leg_base_y = self.robot.leg_positions[leg_name]
            leg_base_world_x = self.robot.x + leg_base_x * cos_yaw - leg_base_y * sin_yaw
            leg_base_world_y = self.robot.y + leg_base_x * sin_yaw + leg_base_y * cos_yaw

            # Foot position in world frame
            foot_x, foot_y = self.robot.get_foot_world_position(leg_name)

            # Draw leg
            self.ax_top.plot([leg_base_world_x, foot_x],
                           [leg_base_world_y, foot_y],
                           color=colors[leg_name], linewidth=2, alpha=0.7)

            # Draw foot
            self.ax_top.scatter([foot_x], [foot_y], s=100,
                              color=colors[leg_name], marker='o',
                              edgecolors='black', linewidths=1.5,
                              label=leg_name, zorder=5)

        # Set view limits
        view_range = 500
        self.ax_top.set_xlim(self.robot.x - view_range, self.robot.x + view_range)
        self.ax_top.set_ylim(self.robot.y - view_range, self.robot.y + view_range)
        self.ax_top.legend(loc='upper right', fontsize=8)

        # Update XY projection plots (leg local frames)
        colors = {'FL': 'green', 'FR': 'blue', 'RL': 'orange', 'RR': 'purple'}

        for leg_name, ax in [('FL', self.ax_xy_fl), ('FR', self.ax_xy_fr),
                            ('RL', self.ax_xy_rl), ('RR', self.ax_xy_rr)]:
            # Get foot position in leg local frame using FK
            theta1, theta2, theta3 = self.robot.joint_angles[leg_name]
            x, y, z = FK(theta1, theta2, theta3)

            # Add to history
            self.foot_xy_history[leg_name]['x'].append(x)
            self.foot_xy_history[leg_name]['y'].append(y)

            # Keep only recent history
            if len(self.foot_xy_history[leg_name]['x']) > self.max_history:
                self.foot_xy_history[leg_name]['x'].pop(0)
                self.foot_xy_history[leg_name]['y'].pop(0)

            # Clear and redraw
            ax.clear()

            # Draw trajectory history (light color)
            if len(self.foot_xy_history[leg_name]['x']) > 1:
                ax.plot(self.foot_xy_history[leg_name]['x'],
                       self.foot_xy_history[leg_name]['y'],
                       color=colors[leg_name], alpha=0.3, linewidth=1)

            # Draw current foot position (bright)
            ax.scatter([x], [y], s=100, color=colors[leg_name],
                      marker='o', edgecolors='black', linewidths=1.5,
                      zorder=5)

            # Draw hip position at origin
            ax.scatter([0], [0], s=80, color='red', marker='x',
                      linewidths=2, zorder=5, label='Hip')

            # Configure plot
            ax.set_xlim(-250, 100)
            ax.set_ylim(-150, 150)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_title(f"{leg_name} - XY Projection", fontsize=9, fontweight='bold')
            ax.set_xlabel("X (mm)", fontsize=7)
            ax.set_ylabel("Y (mm)", fontsize=7)
            ax.tick_params(labelsize=6)
            ax.legend(loc='upper right', fontsize=6)

        # Update XZ projection plots (leg local frames)
        for leg_name, ax in [('FL', self.ax_xz_fl), ('FR', self.ax_xz_fr),
                            ('RL', self.ax_xz_rl), ('RR', self.ax_xz_rr)]:
            # Get foot position in leg local frame using FK
            theta1, theta2, theta3 = self.robot.joint_angles[leg_name]
            x, y, z = FK(theta1, theta2, theta3)

            # Add to history
            self.foot_xz_history[leg_name]['x'].append(x)
            self.foot_xz_history[leg_name]['z'].append(z)

            # Keep only recent history
            if len(self.foot_xz_history[leg_name]['x']) > self.max_history:
                self.foot_xz_history[leg_name]['x'].pop(0)
                self.foot_xz_history[leg_name]['z'].pop(0)

            # Clear and redraw
            ax.clear()

            # Draw trajectory history (light color)
            if len(self.foot_xz_history[leg_name]['x']) > 1:
                ax.plot(self.foot_xz_history[leg_name]['x'],
                       self.foot_xz_history[leg_name]['z'],
                       color=colors[leg_name], alpha=0.3, linewidth=1)

            # Draw current foot position (bright)
            ax.scatter([x], [z], s=100, color=colors[leg_name],
                      marker='o', edgecolors='black', linewidths=1.5,
                      zorder=5)

            # Draw hip position at origin
            ax.scatter([0], [0], s=80, color='red', marker='x',
                      linewidths=2, zorder=5, label='Hip')

            # Configure plot
            ax.set_xlim(-250, 100)
            ax.set_ylim(-300, 50)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_title(f"{leg_name} - XZ Projection", fontsize=9, fontweight='bold')
            ax.set_xlabel("X (mm)", fontsize=7)
            ax.set_ylabel("Z (mm)", fontsize=7)
            ax.tick_params(labelsize=6)
            ax.legend(loc='upper right', fontsize=6)

        # Update joint angle plots
        for leg_name, ax in [('FL', self.ax_angles_fl), ('FR', self.ax_angles_fr),
                            ('RL', self.ax_angles_rl), ('RR', self.ax_angles_rr)]:
            angles = self.robot.joint_angles[leg_name]

            # Add to history
            self.angle_history[leg_name]['theta1'].append(np.degrees(angles[0]))
            self.angle_history[leg_name]['theta2'].append(np.degrees(angles[1]))
            self.angle_history[leg_name]['theta3'].append(np.degrees(angles[2]))

            # Keep only recent history
            for key in ['theta1', 'theta2', 'theta3']:
                if len(self.angle_history[leg_name][key]) > self.max_history:
                    self.angle_history[leg_name][key].pop(0)

            # Plot
            ax.clear()
            ax.plot(self.angle_history[leg_name]['theta1'], 'r-', label='θ1', linewidth=1.5)
            ax.plot(self.angle_history[leg_name]['theta2'], 'g-', label='θ2', linewidth=1.5)
            ax.plot(self.angle_history[leg_name]['theta3'], 'b-', label='θ3', linewidth=1.5)
            ax.set_ylim(-180, 180)
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize=6)
            ax.set_title(f"{leg_name} - Joint Angles", fontsize=9, fontweight='bold')
            ax.set_ylabel("Angle (deg)", fontsize=7)
            ax.tick_params(labelsize=6)

            if leg_name == 'RR':
                ax.set_xlabel("Sample", fontsize=7)

        self.canvas.draw()

    def on_closing(self):
        """Handle window closing"""
        self.running = False
        if self.update_timer:
            self.root.after_cancel(self.update_timer)
        self.root.destroy()


def main():
    root = tk.Tk()
    app = QuadrupedSimulatorGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
