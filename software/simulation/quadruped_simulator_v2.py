"""
Nova-SM3 Quadruped Robot Simulator V2
Full robot simulation with detailed visualization for each leg

Controls:
- Arrow Up/Down: Walk forward/backward
- Arrow Left/Right: Turn left/right
- Space: Stop
- Q: Quit

Author: Quadruped Simulator V2
Date: 2025-12-13
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
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


class SimplifiedGaitController:
    """Simplified gait controller for debugging"""

    def __init__(self):
        self.step_height = 30.0  # mm
        self.step_length = 60.0  # mm
        self.step_duration = 1.0  # seconds

        # Phase offsets for trot gait
        self.phase_offsets = {
            'FL': 0.0,
            'FR': 0.5,
            'RL': 0.5,
            'RR': 0.0
        }

        self.gait_time = 0.0

    def get_foot_position(self, leg_name, is_moving, dt):
        """
        Get foot position for a leg in its LOCAL frame

        Args:
            leg_name: 'FL', 'FR', 'RL', 'RR'
            is_moving: Whether robot is moving
            dt: Time step

        Returns:
            (x, y, z) foot position relative to leg origin (hip joint)
        """
        if is_moving:
            self.gait_time += dt

        # Get phase for this leg
        phase_offset = self.phase_offsets[leg_name]
        phase = ((self.gait_time / self.step_duration) + phase_offset) % 1.0

        # Default stance position in LEG LOCAL FRAME
        # Initial config: (0, 30°, -30°) → FK = (-52.5, 60.0, -222.93)
        x_center = -52.5
        z_default = -222.93

        # Y coordinate depends on leg side:
        # Left legs (FL, RL): y = +60 (outward to left)
        # Right legs (FR, RR): y = -60 (outward to right, MIRRORED!)
        if leg_name in ['FL', 'RL']:
            y_default = 60.0  # Left side
        else:  # FR, RR
            y_default = -60.0  # Right side (mirrored)

        if phase < 0.5:
            # Swing phase - lift and move forward
            swing_progress = phase * 2
            x = x_center - self.step_length/2 + self.step_length * swing_progress
            z = z_default + self.step_height * np.sin(np.pi * swing_progress)
        else:
            # Stance phase - move backward on ground
            stance_progress = (phase - 0.5) * 2
            x = x_center + self.step_length/2 - self.step_length * stance_progress
            z = z_default

        return x, y_default, z


class QuadrupedRobotSimple:
    """Simplified quadruped robot"""

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

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Velocities
        self.velocity_x = 0.0
        self.angular_velocity = 0.0

        # Joint angles - initial configuration
        initial_config = np.array([0, np.radians(30), np.radians(-30)])
        self.joint_angles = {
            'FL': initial_config.copy(),
            'FR': initial_config.copy(),
            'RL': initial_config.copy(),
            'RR': initial_config.copy()
        }

        # Foot positions in leg frame (for visualization)
        # Left legs: y = +60, Right legs: y = -60
        self.foot_positions_leg_frame = {
            'FL': (-52.5, 60.0, -222.93),   # Left
            'FR': (-52.5, -60.0, -222.93),  # Right (mirrored)
            'RL': (-52.5, 60.0, -222.93),   # Left
            'RR': (-52.5, -60.0, -222.93)   # Right (mirrored)
        }

        # Gait controller
        self.gait = SimplifiedGaitController()

        # Track if IK failed
        self.ik_failures = {leg: 0 for leg in ['FL', 'FR', 'RL', 'RR']}

    def update(self, dt):
        """Update robot state"""
        # Update position
        dx = self.velocity_x * dt * np.cos(self.yaw)
        dy = self.velocity_x * dt * np.sin(self.yaw)
        dyaw = self.angular_velocity * dt

        self.x += dx
        self.y += dy
        self.yaw += dyaw
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))

        # Update leg positions
        is_moving = abs(self.velocity_x) > 1.0 or abs(self.angular_velocity) > 0.01

        for leg_name in ['FL', 'FR', 'RL', 'RR']:
            # Get desired foot position
            x_foot, y_foot, z_foot = self.gait.get_foot_position(leg_name, is_moving, dt)

            # Store for visualization
            self.foot_positions_leg_frame[leg_name] = (x_foot, y_foot, z_foot)

            # Calculate IK
            result = self.select_best_ik_solution(
                x_foot, y_foot, z_foot,
                self.joint_angles[leg_name],
                leg_name
            )

            if result:
                self.joint_angles[leg_name] = np.array(result)
                self.ik_failures[leg_name] = 0
            else:
                self.ik_failures[leg_name] += 1
                # Keep previous angles if IK fails

    def select_best_ik_solution(self, x, y, z, current_angles, leg_name):
        """
        Select best IK solution

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

        if not all_solutions:
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


class QuadrupedSimulatorGUIV2:
    """GUI with full visualization for each leg"""

    def __init__(self, root):
        self.root = root
        self.root.title("Nova-SM3 Quadruped Simulator V2")
        self.root.geometry("1800x1000")

        # Body dimensions (can be configured)
        self.body_length = 600.0  # H: Front-rear distance (mm)
        self.body_width = 200.0   # W: Left-right distance (mm)

        self.robot = QuadrupedRobotSimple(self.body_length, self.body_width)

        self.dt = 0.05
        self.running = False
        self.update_timer = None

        # Command velocities
        self.cmd_vel_x = 0.0
        self.cmd_angular = 0.0

        # Limits
        self.max_linear_vel = 100.0  # mm/s (slower for stability)
        self.max_angular_vel = np.radians(45)  # rad/s

        # Acceleration
        self.linear_accel = 200.0
        self.angular_accel = np.radians(90)

        # Keys pressed
        self.keys_pressed = set()

        # History for plotting
        self.angle_history = {
            leg: {'theta1': [], 'theta2': [], 'theta3': [], 'time': []}
            for leg in ['FL', 'FR', 'RL', 'RR']
        }
        self.max_history = 100
        self.time_elapsed = 0.0

        self.setup_ui()
        self.bind_keys()
        self.start_simulation()

    def setup_ui(self):
        """Setup UI"""
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left panel - Controls
        control_frame = ttk.Frame(main_frame, width=250)
        control_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        control_frame.grid_propagate(False)

        ttk.Label(control_frame, text="Quadruped Simulator V2",
                 font=("Arial", 12, "bold")).pack(pady=5)

        # Controls
        ctrl_frame = ttk.LabelFrame(control_frame, text="Controls", padding=5)
        ctrl_frame.pack(fill=tk.X, pady=5)

        ttk.Label(ctrl_frame, text="↑/W: Forward\n↓/S: Backward\n←/A: Left\n→/D: Right\nSpace: Stop",
                 font=("Courier", 8), justify=tk.LEFT).pack()

        # State
        state_frame = ttk.LabelFrame(control_frame, text="State", padding=5)
        state_frame.pack(fill=tk.X, pady=5)

        self.state_label = ttk.Label(state_frame, text="", font=("Courier", 8))
        self.state_label.pack()

        # IK status
        ik_frame = ttk.LabelFrame(control_frame, text="IK Status", padding=5)
        ik_frame.pack(fill=tk.X, pady=5)

        self.ik_label = ttk.Label(ik_frame, text="", font=("Courier", 8))
        self.ik_label.pack()

        # Right panel - Visualizations
        viz_frame = ttk.Frame(main_frame)
        viz_frame.grid(row=0, column=1, sticky="nsew")

        # Create figure with subplots for each leg
        self.fig = plt.Figure(figsize=(17, 10))

        # Layout: 4 rows (one per leg), 4 columns (3D, XZ, XY, Angles)
        legs = ['FL', 'FR', 'RL', 'RR']
        self.axes = {}

        for i, leg in enumerate(legs):
            row = i

            # 3D view
            ax_3d = self.fig.add_subplot(4, 4, row*4 + 1, projection='3d')
            ax_3d.set_title(f"{leg} - 3D View", fontsize=9, fontweight='bold')
            ax_3d.set_xlabel("X", fontsize=7)
            ax_3d.set_ylabel("Y", fontsize=7)
            ax_3d.set_zlabel("Z", fontsize=7)
            ax_3d.tick_params(labelsize=6)

            # XZ projection
            ax_xz = self.fig.add_subplot(4, 4, row*4 + 2)
            ax_xz.set_title(f"{leg} - XZ", fontsize=9, fontweight='bold')
            ax_xz.set_xlabel("X (mm)", fontsize=7)
            ax_xz.set_ylabel("Z (mm)", fontsize=7)
            ax_xz.grid(True, alpha=0.3)
            ax_xz.set_aspect('equal')
            ax_xz.tick_params(labelsize=6)

            # XY projection
            ax_xy = self.fig.add_subplot(4, 4, row*4 + 3)
            ax_xy.set_title(f"{leg} - XY", fontsize=9, fontweight='bold')
            ax_xy.set_xlabel("X (mm)", fontsize=7)
            ax_xy.set_ylabel("Y (mm)", fontsize=7)
            ax_xy.grid(True, alpha=0.3)
            ax_xy.set_aspect('equal')
            ax_xy.tick_params(labelsize=6)

            # Angles
            ax_ang = self.fig.add_subplot(4, 4, row*4 + 4)
            ax_ang.set_title(f"{leg} - Angles", fontsize=9, fontweight='bold')
            ax_ang.set_ylabel("Angle (deg)", fontsize=7)
            ax_ang.set_ylim(-180, 180)
            ax_ang.grid(True, alpha=0.3)
            ax_ang.tick_params(labelsize=6)

            self.axes[leg] = {
                '3d': ax_3d,
                'xz': ax_xz,
                'xy': ax_xy,
                'angles': ax_ang
            }

        self.fig.tight_layout(pad=1.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

    def bind_keys(self):
        """Bind keys"""
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

        self.root.bind('<KeyRelease-Up>', lambda e: self.on_key_release('forward'))
        self.root.bind('<KeyRelease-Down>', lambda e: self.on_key_release('backward'))
        self.root.bind('<KeyRelease-Left>', lambda e: self.on_key_release('left'))
        self.root.bind('<KeyRelease-Right>', lambda e: self.on_key_release('right'))
        self.root.bind('<KeyRelease-w>', lambda e: self.on_key_release('forward'))
        self.root.bind('<KeyRelease-s>', lambda e: self.on_key_release('backward'))
        self.root.bind('<KeyRelease-a>', lambda e: self.on_key_release('left'))
        self.root.bind('<KeyRelease-d>', lambda e: self.on_key_release('right'))

    def on_key_press(self, command):
        """Handle key press"""
        if command == 'stop':
            self.keys_pressed.clear()
            self.cmd_vel_x = 0.0
            self.cmd_angular = 0.0
        else:
            self.keys_pressed.add(command)
            self.update_command_from_keys()

    def on_key_release(self, command):
        """Handle key release"""
        if command in self.keys_pressed:
            self.keys_pressed.remove(command)
        self.update_command_from_keys()

    def update_command_from_keys(self):
        """Update commands from keys"""
        self.cmd_vel_x = 0.0
        self.cmd_angular = 0.0

        if 'forward' in self.keys_pressed:
            self.cmd_vel_x = self.max_linear_vel
        if 'backward' in self.keys_pressed:
            self.cmd_vel_x = -self.max_linear_vel
        if 'left' in self.keys_pressed:
            self.cmd_angular = self.max_angular_vel
        if 'right' in self.keys_pressed:
            self.cmd_angular = -self.max_angular_vel

    def update_velocities(self):
        """Update velocities with acceleration"""
        vel_error = self.cmd_vel_x - self.robot.velocity_x
        max_change = self.linear_accel * self.dt
        if abs(vel_error) < max_change:
            self.robot.velocity_x = self.cmd_vel_x
        else:
            self.robot.velocity_x += np.sign(vel_error) * max_change

        ang_error = self.cmd_angular - self.robot.angular_velocity
        max_ang_change = self.angular_accel * self.dt
        if abs(ang_error) < max_ang_change:
            self.robot.angular_velocity = self.cmd_angular
        else:
            self.robot.angular_velocity += np.sign(ang_error) * max_ang_change

    def start_simulation(self):
        """Start simulation"""
        self.running = True
        self.update_simulation()

    def update_simulation(self):
        """Update simulation"""
        if not self.running:
            return

        self.update_velocities()
        self.robot.update(self.dt)
        self.time_elapsed += self.dt

        # Update history
        for leg in ['FL', 'FR', 'RL', 'RR']:
            angles = self.robot.joint_angles[leg]
            self.angle_history[leg]['theta1'].append(np.degrees(angles[0]))
            self.angle_history[leg]['theta2'].append(np.degrees(angles[1]))
            self.angle_history[leg]['theta3'].append(np.degrees(angles[2]))
            self.angle_history[leg]['time'].append(self.time_elapsed)

            # Keep only recent
            for key in ['theta1', 'theta2', 'theta3', 'time']:
                if len(self.angle_history[leg][key]) > self.max_history:
                    self.angle_history[leg][key].pop(0)

        self.update_displays()
        self.update_timer = self.root.after(int(self.dt * 1000), self.update_simulation)

    def update_displays(self):
        """Update all displays"""
        # Update state text
        state_text = f"Pos: ({self.robot.x:.0f}, {self.robot.y:.0f})\nYaw: {np.degrees(self.robot.yaw):.1f}°\nVx: {self.robot.velocity_x:.1f} mm/s"
        self.state_label.config(text=state_text)

        ik_text = "\n".join([f"{leg}: {'OK' if self.robot.ik_failures[leg] == 0 else f'FAIL x{self.robot.ik_failures[leg]}'}"
                            for leg in ['FL', 'FR', 'RL', 'RR']])
        self.ik_label.config(text=ik_text)

        # Update plots for each leg
        for leg in ['FL', 'FR', 'RL', 'RR']:
            self.update_leg_plots(leg)

        self.canvas.draw()

    def update_leg_plots(self, leg_name):
        """Update plots for one leg"""
        angles = self.robot.joint_angles[leg_name]
        x_foot, y_foot, z_foot = self.robot.foot_positions_leg_frame[leg_name]

        # 3D plot
        ax_3d = self.axes[leg_name]['3d']
        ax_3d.clear()

        # Calculate link positions
        theta1, theta2, theta3 = angles
        p0 = np.array([0, 0, 0])
        p1 = np.array([0, L1*np.cos(theta1), L1*np.sin(theta1)])
        p2 = np.array([-L2*np.sin(theta2),
                      L1*np.cos(theta1) + L2*np.sin(theta1)*np.cos(theta2),
                      L1*np.sin(theta1) - L2*np.cos(theta1)*np.cos(theta2)])
        p3 = np.array([x_foot, y_foot, z_foot])

        # Draw leg
        leg_x = [p0[0], p1[0], p2[0], p3[0]]
        leg_y = [p0[1], p1[1], p2[1], p3[1]]
        leg_z = [p0[2], p1[2], p2[2], p3[2]]

        ax_3d.plot(leg_x, leg_y, leg_z, 'r-o', linewidth=3, markersize=6)
        ax_3d.set_xlim(-150, 150)
        ax_3d.set_ylim(-50, 150)
        ax_3d.set_zlim(-300, 50)
        ax_3d.set_title(f"{leg_name} - 3D", fontsize=9, fontweight='bold')

        # XZ plot
        ax_xz = self.axes[leg_name]['xz']
        ax_xz.clear()
        ax_xz.plot(leg_x, leg_z, 'r-o', linewidth=3, markersize=6)
        ax_xz.set_xlim(-150, 150)
        ax_xz.set_ylim(-300, 50)
        ax_xz.grid(True, alpha=0.3)
        ax_xz.set_aspect('equal')
        ax_xz.set_title(f"{leg_name} - XZ", fontsize=9, fontweight='bold')

        # XY plot
        ax_xy = self.axes[leg_name]['xy']
        ax_xy.clear()
        ax_xy.plot(leg_x, leg_y, 'r-o', linewidth=3, markersize=6)
        ax_xy.set_xlim(-150, 150)
        ax_xy.set_ylim(-50, 150)
        ax_xy.grid(True, alpha=0.3)
        ax_xy.set_aspect('equal')
        ax_xy.set_title(f"{leg_name} - XY", fontsize=9, fontweight='bold')

        # Angles plot
        ax_ang = self.axes[leg_name]['angles']
        ax_ang.clear()
        if len(self.angle_history[leg_name]['time']) > 1:
            ax_ang.plot(self.angle_history[leg_name]['time'],
                       self.angle_history[leg_name]['theta1'], 'r-', label='θ1', linewidth=1.5)
            ax_ang.plot(self.angle_history[leg_name]['time'],
                       self.angle_history[leg_name]['theta2'], 'g-', label='θ2', linewidth=1.5)
            ax_ang.plot(self.angle_history[leg_name]['time'],
                       self.angle_history[leg_name]['theta3'], 'b-', label='θ3', linewidth=1.5)
        ax_ang.set_ylim(-180, 180)
        ax_ang.grid(True, alpha=0.3)
        ax_ang.legend(fontsize=6, loc='upper right')
        ax_ang.set_title(f"{leg_name} - Angles", fontsize=9, fontweight='bold')

    def on_closing(self):
        """Handle closing"""
        self.running = False
        if self.update_timer:
            self.root.after_cancel(self.update_timer)
        self.root.destroy()


def main():
    root = tk.Tk()
    app = QuadrupedSimulatorGUIV2(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
