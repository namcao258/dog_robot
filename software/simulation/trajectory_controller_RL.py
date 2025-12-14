"""
Trajectory Controller for Nova-SM3 - Rear Left Leg (RL)
Same as FL trajectory controller (left leg configuration)

Key characteristics:
- Y coordinate: +60mm (outward to left)
- L1 direction: +Y (same as FL)
- No mirroring needed (left leg)

Author: RL Leg Controller
Date: 2025-12-13
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
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


class TrajectoryGenerator:
    """Generate various trajectories for robot leg"""

    @staticmethod
    def walking_gait(t, step_height=50, step_length=100, step_duration=1.0, y_offset=60):
        """
        Generate walking gait trajectory
        Initial config: x=-52.5 (leg backward, elbow pointing back)
        Walking direction: FORWARD (toward x positive)

        Args:
            t: Time (seconds)
            step_height: Maximum height of step (mm)
            step_length: Length of step forward (mm)
            step_duration: Duration of one complete step (seconds)
            y_offset: Lateral offset (mm)

        Returns:
            (x, y, z): Position at time t
        """
        # Normalize time to [0, 1] within step duration
        phase = (t % step_duration) / step_duration

        # Start position: x = -52.5 (initial leg position, backward)
        x_start = -52.5
        # End position: x = x_start + step_length (forward)
        x_end = x_start + step_length

        # Swing phase (0.0 to 0.5): lift leg and move FORWARD
        # Stance phase (0.5 to 1.0): push backward (ground contact)
        if phase < 0.5:
            # Swing phase - lift and move forward (half-ellipse)
            swing_phase = phase * 2  # Map to [0, 1]
            x = x_start + step_length * swing_phase
            z = -222.93 + step_height * np.sin(np.pi * swing_phase)
        else:
            # Stance phase - move backward while on ground
            stance_phase = (phase - 0.5) * 2  # Map to [0, 1]
            x = x_end - step_length * stance_phase
            z = -222.93

        y = y_offset
        return x, y, z

    @staticmethod
    def circle(t, radius=30, center=(-52.5, 60, -222.93), period=2.0):
        """
        Generate circular trajectory

        Args:
            t: Time (seconds)
            radius: Circle radius (mm)
            center: Center point (x, y, z) - default at initial position
            period: Period of one revolution (seconds)

        Returns:
            (x, y, z): Position at time t
        """
        angle = 2 * np.pi * t / period
        x = center[0] + radius * np.cos(angle)
        y = center[1]
        z = center[2] + radius * np.sin(angle)
        return x, y, z

    @staticmethod
    def lissajous(t, amplitude=(40, 30), frequency=(1, 2), phase=(0, np.pi/2), center=(-52.5, 60, -222.93)):
        """
        Generate Lissajous curve trajectory

        Args:
            t: Time (seconds)
            amplitude: (x_amp, z_amp) amplitudes (mm)
            frequency: (x_freq, z_freq) frequency ratios
            phase: (x_phase, z_phase) phase offsets (radians)
            center: Center point (x, y, z) - default at initial position

        Returns:
            (x, y, z): Position at time t
        """
        x = center[0] + amplitude[0] * np.sin(frequency[0] * t + phase[0])
        y = center[1]
        z = center[2] + amplitude[1] * np.sin(frequency[1] * t + phase[1])
        return x, y, z

    @staticmethod
    def square(t, side_length=60, center=(-52.5, 60, -222.93), period=4.0):
        """
        Generate square trajectory

        Args:
            t: Time (seconds)
            side_length: Length of square side (mm)
            center: Center point (x, y, z) - default at initial position
            period: Period for one complete square (seconds)

        Returns:
            (x, y, z): Position at time t
        """
        phase = (t % period) / period  # Normalize to [0, 1]
        half = side_length / 2

        if phase < 0.25:  # Right side
            progress = phase * 4
            x = center[0] + half
            z = center[2] - half + side_length * progress
        elif phase < 0.5:  # Top side
            progress = (phase - 0.25) * 4
            x = center[0] + half - side_length * progress
            z = center[2] + half
        elif phase < 0.75:  # Left side
            progress = (phase - 0.5) * 4
            x = center[0] - half
            z = center[2] + half - side_length * progress
        else:  # Bottom side
            progress = (phase - 0.75) * 4
            x = center[0] - half + side_length * progress
            z = center[2] - half

        y = center[1]
        return x, y, z


class TrajectoryControllerGUI:
    """GUI for trajectory control and visualization"""

    def __init__(self, root):
        self.root = root
        self.root.title("Nova-SM3 Trajectory Controller")
        self.root.geometry("1400x900")

        # Trajectory parameters
        self.trajectory_type = "Walking Gait"
        self.is_playing = False
        self.current_time = 0.0
        self.dt = 0.02  # 50 FPS
        self.animation = None

        # Trajectory data
        self.time_array = None
        self.positions = None
        self.joint_angles = None

        # Initial joint configuration (0, 30°, -30°)
        # This configuration keeps the elbow/knee pointing backward
        self.initial_angles = np.array([0, np.radians(30), np.radians(-30)])
        self.current_joint_angles = self.initial_angles.copy()

        self.setup_ui()
        self.generate_trajectory()

    def setup_ui(self):
        """Setup user interface"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel - Controls
        control_frame = ttk.LabelFrame(main_frame, text="Trajectory Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        # Trajectory selection
        ttk.Label(control_frame, text="Trajectory Type:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", pady=5)
        self.traj_combo = ttk.Combobox(control_frame, values=[
            "Walking Gait", "Circle", "Lissajous", "Square"
        ], state="readonly", width=20)
        self.traj_combo.set("Walking Gait")
        self.traj_combo.grid(row=0, column=1, pady=5, padx=5)
        self.traj_combo.bind("<<ComboboxSelected>>", lambda e: self.on_trajectory_changed())

        # Walking gait parameters
        self.walking_frame = ttk.LabelFrame(control_frame, text="Walking Gait Parameters", padding=10)
        self.walking_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=10)

        ttk.Label(self.walking_frame, text="Step Height (mm):").grid(row=0, column=0, sticky="w")
        self.step_height_var = tk.DoubleVar(value=50)
        ttk.Scale(self.walking_frame, from_=20, to=100, variable=self.step_height_var,
                  orient=tk.HORIZONTAL, command=lambda e: self.on_param_changed()).grid(row=0, column=1, sticky="ew", padx=5)
        self.step_height_label = ttk.Label(self.walking_frame, text="50")
        self.step_height_label.grid(row=0, column=2)

        ttk.Label(self.walking_frame, text="Step Length (mm):").grid(row=1, column=0, sticky="w")
        self.step_length_var = tk.DoubleVar(value=100)
        ttk.Scale(self.walking_frame, from_=50, to=150, variable=self.step_length_var,
                  orient=tk.HORIZONTAL, command=lambda e: self.on_param_changed()).grid(row=1, column=1, sticky="ew", padx=5)
        self.step_length_label = ttk.Label(self.walking_frame, text="100")
        self.step_length_label.grid(row=1, column=2)

        ttk.Label(self.walking_frame, text="Duration (s):").grid(row=2, column=0, sticky="w")
        self.step_duration_var = tk.DoubleVar(value=1.0)
        ttk.Scale(self.walking_frame, from_=0.5, to=3.0, variable=self.step_duration_var,
                  orient=tk.HORIZONTAL, command=lambda e: self.on_param_changed()).grid(row=2, column=1, sticky="ew", padx=5)
        self.step_duration_label = ttk.Label(self.walking_frame, text="1.0")
        self.step_duration_label.grid(row=2, column=2)

        self.walking_frame.columnconfigure(1, weight=1)

        # Circle parameters
        self.circle_frame = ttk.LabelFrame(control_frame, text="Circle Parameters", padding=10)

        ttk.Label(self.circle_frame, text="Radius (mm):").grid(row=0, column=0, sticky="w")
        self.circle_radius_var = tk.DoubleVar(value=30)
        ttk.Scale(self.circle_frame, from_=10, to=80, variable=self.circle_radius_var,
                  orient=tk.HORIZONTAL, command=lambda e: self.on_param_changed()).grid(row=0, column=1, sticky="ew", padx=5)
        self.circle_radius_label = ttk.Label(self.circle_frame, text="30")
        self.circle_radius_label.grid(row=0, column=2)

        ttk.Label(self.circle_frame, text="Period (s):").grid(row=1, column=0, sticky="w")
        self.circle_period_var = tk.DoubleVar(value=2.0)
        ttk.Scale(self.circle_frame, from_=0.5, to=5.0, variable=self.circle_period_var,
                  orient=tk.HORIZONTAL, command=lambda e: self.on_param_changed()).grid(row=1, column=1, sticky="ew", padx=5)
        self.circle_period_label = ttk.Label(self.circle_frame, text="2.0")
        self.circle_period_label.grid(row=1, column=2)

        self.circle_frame.columnconfigure(1, weight=1)

        # Playback controls
        playback_frame = ttk.LabelFrame(control_frame, text="Playback", padding=10)
        playback_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=10)

        self.play_button = ttk.Button(playback_frame, text="▶ Play", command=self.toggle_play, width=15)
        self.play_button.grid(row=0, column=0, padx=5, pady=5)

        ttk.Button(playback_frame, text="⏹ Reset", command=self.reset_animation, width=15).grid(row=0, column=1, padx=5, pady=5)

        # Time slider
        ttk.Label(playback_frame, text="Time:").grid(row=1, column=0, sticky="w", pady=5)
        self.time_var = tk.DoubleVar(value=0)
        self.time_slider = ttk.Scale(playback_frame, from_=0, to=5, variable=self.time_var,
                                     orient=tk.HORIZONTAL, command=self.on_time_changed)
        self.time_slider.grid(row=1, column=1, sticky="ew", padx=5)
        self.time_label = ttk.Label(playback_frame, text="0.00 s")
        self.time_label.grid(row=1, column=2)

        playback_frame.columnconfigure(1, weight=1)

        # Initial configuration
        init_frame = ttk.LabelFrame(control_frame, text="Initial Config (RL - Left Leg)", padding=10)
        init_frame.grid(row=4, column=0, columnspan=2, sticky="ew", pady=10)

        ttk.Label(init_frame, text="Initial Angles (deg):", font=("Arial", 9, "bold")).grid(row=0, column=0, columnspan=2, sticky="w")
        init_angles_text = f"θ1={np.degrees(self.initial_angles[0]):.0f}°, θ2={np.degrees(self.initial_angles[1]):.0f}°, θ3={np.degrees(self.initial_angles[2]):.0f}°"
        ttk.Label(init_frame, text=init_angles_text, font=("Courier", 9), foreground="blue").grid(row=1, column=0, columnspan=2, sticky="w", padx=10)

        ttk.Label(init_frame, text="IK Selection: Minimize joint angle change", font=("Arial", 8), foreground="gray").grid(row=2, column=0, columnspan=2, sticky="w", pady=(5, 0))

        # Current state display
        state_frame = ttk.LabelFrame(control_frame, text="Current State", padding=10)
        state_frame.grid(row=5, column=0, columnspan=2, sticky="ew", pady=10)

        ttk.Label(state_frame, text="Position (mm):", font=("Arial", 9, "bold")).grid(row=0, column=0, columnspan=2, sticky="w")
        self.pos_label = ttk.Label(state_frame, text="x=0.0, y=0.0, z=0.0", font=("Courier", 9))
        self.pos_label.grid(row=1, column=0, columnspan=2, sticky="w", padx=10)

        ttk.Label(state_frame, text="Joint Angles (deg):", font=("Arial", 9, "bold")).grid(row=2, column=0, columnspan=2, sticky="w", pady=(10, 0))
        self.angles_label = ttk.Label(state_frame, text="θ1=0.0°, θ2=0.0°, θ3=0.0°", font=("Courier", 9))
        self.angles_label.grid(row=3, column=0, columnspan=2, sticky="w", padx=10)

        # Generate button
        ttk.Button(control_frame, text="🔄 Regenerate Trajectory",
                   command=self.generate_trajectory).grid(row=6, column=0, columnspan=2, pady=10)

        # Right panel - Visualizations
        viz_frame = ttk.Frame(main_frame)
        viz_frame.grid(row=0, column=1, sticky="nsew")

        # Create matplotlib figures
        self.fig = plt.Figure(figsize=(12, 8))

        # 3D trajectory plot
        self.ax_3d = self.fig.add_subplot(221, projection='3d')
        self.ax_3d.set_title("RL - 3D Trajectory", fontweight='bold')
        self.ax_3d.set_xlabel("X (mm)")
        self.ax_3d.set_ylabel("Y (mm)")
        self.ax_3d.set_zlabel("Z (mm)")

        # XZ projection
        self.ax_xz = self.fig.add_subplot(222)
        self.ax_xz.set_title("XZ Plane Projection", fontweight='bold')
        self.ax_xz.set_xlabel("X (mm)")
        self.ax_xz.set_ylabel("Z (mm)")
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')

        # Joint angles over time
        self.ax_angles = self.fig.add_subplot(223)
        self.ax_angles.set_title("RL - Joint Angles", fontweight='bold')
        self.ax_angles.set_xlabel("Time (s)")
        self.ax_angles.set_ylabel("Angle (degrees)")
        self.ax_angles.grid(True, alpha=0.3)

        # XY projection
        self.ax_xy = self.fig.add_subplot(224)
        self.ax_xy.set_title("XY Plane Projection", fontweight='bold')
        self.ax_xy.set_xlabel("X (mm)")
        self.ax_xy.set_ylabel("Y (mm)")
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')

        self.fig.tight_layout()

        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Configure grid weights
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

    def on_trajectory_changed(self):
        """Handle trajectory type change"""
        self.trajectory_type = self.traj_combo.get()

        # Hide all parameter frames
        self.walking_frame.grid_remove()
        self.circle_frame.grid_remove()

        # Show relevant parameter frame
        if self.trajectory_type == "Walking Gait":
            self.walking_frame.grid()
        elif self.trajectory_type == "Circle":
            self.circle_frame.grid()

        self.generate_trajectory()

    def on_param_changed(self):
        """Handle parameter change"""
        # Update labels
        self.step_height_label.config(text=f"{self.step_height_var.get():.0f}")
        self.step_length_label.config(text=f"{self.step_length_var.get():.0f}")
        self.step_duration_label.config(text=f"{self.step_duration_var.get():.1f}")
        self.circle_radius_label.config(text=f"{self.circle_radius_var.get():.0f}")
        self.circle_period_label.config(text=f"{self.circle_period_var.get():.1f}")

    def select_best_ik_solution(self, x, y, z, current_angles):
        """
        Select best IK solution based on minimum joint angle change

        Args:
            x, y, z: Target position
            current_angles: Current joint angles (theta1, theta2, theta3)

        Returns:
            Best (theta1, theta2, theta3) solution
        """
        # Get all IK solutions
        all_solutions = IK(x, y, z, return_all=True)

        if not all_solutions or len(all_solutions) == 0:
            return None

        # Calculate total angular distance for each solution
        best_solution = None
        min_distance = float('inf')

        for theta1, theta2, theta3, error in all_solutions:
            # Calculate angular distance (sum of absolute differences)
            distance = (abs(theta1 - current_angles[0]) +
                       abs(theta2 - current_angles[1]) +
                       abs(theta3 - current_angles[2]))

            if distance < min_distance:
                min_distance = distance
                best_solution = (theta1, theta2, theta3)

        return best_solution

    def generate_trajectory(self):
        """Generate trajectory based on current parameters"""
        # Stop animation if playing
        if self.is_playing:
            self.toggle_play()

        # Generate time array
        duration = 5.0  # 5 seconds total
        num_points = int(duration / self.dt)
        self.time_array = np.linspace(0, duration, num_points)

        # Generate positions
        self.positions = []
        self.joint_angles = []

        # Reset to initial configuration
        self.current_joint_angles = self.initial_angles.copy()

        traj_gen = TrajectoryGenerator()

        for t in self.time_array:
            # Get position based on trajectory type
            if self.trajectory_type == "Walking Gait":
                x, y, z = traj_gen.walking_gait(
                    t,
                    step_height=self.step_height_var.get(),
                    step_length=self.step_length_var.get(),
                    step_duration=self.step_duration_var.get()
                )
            elif self.trajectory_type == "Circle":
                x, y, z = traj_gen.circle(
                    t,
                    radius=self.circle_radius_var.get(),
                    period=self.circle_period_var.get()
                )
            elif self.trajectory_type == "Lissajous":
                x, y, z = traj_gen.lissajous(t)
            elif self.trajectory_type == "Square":
                x, y, z = traj_gen.square(t)
            else:
                x, y, z = -52.5, 60, -222.93  # Default to initial position

            self.positions.append((x, y, z))

            # Calculate IK - select best solution based on current angles
            result = self.select_best_ik_solution(x, y, z, self.current_joint_angles)
            if result:
                theta1, theta2, theta3 = result
                self.joint_angles.append((theta1, theta2, theta3))
                # Update current angles for next iteration
                self.current_joint_angles = np.array([theta1, theta2, theta3])
            else:
                # If unreachable, use previous angles
                if self.joint_angles:
                    self.joint_angles.append(self.joint_angles[-1])
                else:
                    self.joint_angles.append(tuple(self.initial_angles))

        self.positions = np.array(self.positions)
        self.joint_angles = np.array(self.joint_angles)

        # Update time slider range
        self.time_slider.config(to=duration)

        # Plot trajectory
        self.plot_trajectory()

        # Reset to start
        self.current_time = 0
        self.time_var.set(0)
        self.update_display(0)

    def plot_trajectory(self):
        """Plot the complete trajectory"""
        # Clear all axes
        self.ax_3d.clear()
        self.ax_xz.clear()
        self.ax_xy.clear()
        self.ax_angles.clear()

        x_data = self.positions[:, 0]
        y_data = self.positions[:, 1]
        z_data = self.positions[:, 2]

        # 3D trajectory
        self.ax_3d.plot(x_data, y_data, z_data, 'b-', linewidth=2, alpha=0.6, label='Trajectory')
        self.ax_3d.scatter([0], [0], [0], c='red', s=100, marker='o', label='Hip Joint')
        self.ax_3d.set_xlabel("X (mm)")
        self.ax_3d.set_ylabel("Y (mm)")
        self.ax_3d.set_zlabel("Z (mm)")
        self.ax_3d.set_title("RL - 3D Trajectory", fontweight='bold')
        self.ax_3d.legend()

        # Set equal aspect ratio for 3D plot
        max_range = max(np.ptp(x_data), np.ptp(y_data), np.ptp(z_data))
        mid_x = np.mean(x_data)
        mid_y = np.mean(y_data)
        mid_z = np.mean(z_data)
        self.ax_3d.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        self.ax_3d.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        self.ax_3d.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

        # XZ projection
        self.ax_xz.plot(x_data, z_data, 'b-', linewidth=2, alpha=0.6)
        self.ax_xz.scatter([0], [0], c='red', s=100, marker='o')
        self.ax_xz.set_xlabel("X (mm)")
        self.ax_xz.set_ylabel("Z (mm)")
        self.ax_xz.set_title("XZ Plane Projection", fontweight='bold')
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')

        # XY projection
        self.ax_xy.plot(x_data, y_data, 'b-', linewidth=2, alpha=0.6)
        self.ax_xy.scatter([0], [0], c='red', s=100, marker='o')
        self.ax_xy.set_xlabel("X (mm)")
        self.ax_xy.set_ylabel("Y (mm)")
        self.ax_xy.set_title("XY Plane Projection", fontweight='bold')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')

        # Joint angles
        theta1_deg = np.degrees(self.joint_angles[:, 0])
        theta2_deg = np.degrees(self.joint_angles[:, 1])
        theta3_deg = np.degrees(self.joint_angles[:, 2])

        self.ax_angles.plot(self.time_array, theta1_deg, 'r-', linewidth=2, label='θ1 (Hip)')
        self.ax_angles.plot(self.time_array, theta2_deg, 'g-', linewidth=2, label='θ2 (Knee)')
        self.ax_angles.plot(self.time_array, theta3_deg, 'b-', linewidth=2, label='θ3 (Ankle)')
        self.ax_angles.set_xlabel("Time (s)")
        self.ax_angles.set_ylabel("Angle (degrees)")
        self.ax_angles.set_title("RL - Joint Angles", fontweight='bold')
        self.ax_angles.grid(True, alpha=0.3)
        self.ax_angles.legend()

        self.fig.tight_layout()
        self.canvas.draw()

    def update_display(self, time_index):
        """Update display for current time index"""
        if time_index >= len(self.positions):
            return

        x, y, z = self.positions[time_index]
        theta1, theta2, theta3 = self.joint_angles[time_index]

        # Update labels
        self.pos_label.config(text=f"x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self.angles_label.config(text=f"θ1={np.degrees(theta1):.2f}°, θ2={np.degrees(theta2):.2f}°, θ3={np.degrees(theta3):.2f}°")
        self.time_label.config(text=f"{self.time_array[time_index]:.2f} s")

        # Update 3D plot - draw leg configuration
        self.ax_3d.clear()

        # Plot full trajectory
        self.ax_3d.plot(self.positions[:, 0], self.positions[:, 1], self.positions[:, 2],
                       'b-', linewidth=1, alpha=0.3, label='Trajectory')

        # Plot trajectory up to current point
        self.ax_3d.plot(self.positions[:time_index+1, 0],
                       self.positions[:time_index+1, 1],
                       self.positions[:time_index+1, 2],
                       'b-', linewidth=2, alpha=0.8)

        # Calculate leg link positions using FK
        # Joint 0: Hip (origin)
        p0 = np.array([0, 0, 0])

        # Joint 1: After L1 rotation
        p1 = np.array([0,
                      L1 * np.cos(theta1),
                      L1 * np.sin(theta1)])

        # Joint 2: After L2 extension
        p2 = np.array([-L2 * np.sin(theta2),
                      L1 * np.cos(theta1) + L2 * np.sin(theta1) * np.cos(theta2),
                      L1 * np.sin(theta1) - L2 * np.cos(theta1) * np.cos(theta2)])

        # End effector: After L3 extension
        p3 = np.array([x, y, z])

        # Draw leg links
        leg_x = [p0[0], p1[0], p2[0], p3[0]]
        leg_y = [p0[1], p1[1], p2[1], p3[1]]
        leg_z = [p0[2], p1[2], p2[2], p3[2]]

        self.ax_3d.plot(leg_x, leg_y, leg_z, 'r-', linewidth=4, marker='o',
                       markersize=8, label='Leg Links')

        # Mark current position
        self.ax_3d.scatter([x], [y], [z], c='green', s=200, marker='*',
                          label='Current Position', zorder=5)

        self.ax_3d.set_xlabel("X (mm)")
        self.ax_3d.set_ylabel("Y (mm)")
        self.ax_3d.set_zlabel("Z (mm)")
        self.ax_3d.set_title("RL - 3D Trajectory", fontweight='bold')
        self.ax_3d.legend()

        # Set consistent view
        max_range = 150
        mid_x = np.mean(self.positions[:, 0])
        mid_y = np.mean(self.positions[:, 1])
        mid_z = np.mean(self.positions[:, 2])
        self.ax_3d.set_xlim(mid_x - max_range, mid_x + max_range)
        self.ax_3d.set_ylim(mid_y - max_range, mid_y + max_range)
        self.ax_3d.set_zlim(mid_z - max_range, mid_z + max_range)

        # Update XZ plot
        self.ax_xz.clear()
        self.ax_xz.plot(self.positions[:, 0], self.positions[:, 2], 'b-', linewidth=1, alpha=0.3)
        self.ax_xz.plot(self.positions[:time_index+1, 0], self.positions[:time_index+1, 2],
                       'b-', linewidth=2)
        self.ax_xz.scatter([x], [z], c='green', s=200, marker='*', zorder=5)
        self.ax_xz.plot(leg_x, leg_z, 'r-', linewidth=3, marker='o', markersize=6)
        self.ax_xz.set_xlabel("X (mm)")
        self.ax_xz.set_ylabel("Z (mm)")
        self.ax_xz.set_title("XZ Plane Projection", fontweight='bold')
        self.ax_xz.grid(True, alpha=0.3)
        self.ax_xz.set_aspect('equal')

        # Update XY plot
        self.ax_xy.clear()
        self.ax_xy.plot(self.positions[:, 0], self.positions[:, 1], 'b-', linewidth=1, alpha=0.3)
        self.ax_xy.plot(self.positions[:time_index+1, 0], self.positions[:time_index+1, 1],
                       'b-', linewidth=2)
        self.ax_xy.scatter([x], [y], c='green', s=200, marker='*', zorder=5)
        self.ax_xy.plot(leg_x, leg_y, 'r-', linewidth=3, marker='o', markersize=6)
        self.ax_xy.set_xlabel("X (mm)")
        self.ax_xy.set_ylabel("Y (mm)")
        self.ax_xy.set_title("XY Plane Projection", fontweight='bold')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')

        # Update angles plot - add vertical line at current time
        self.ax_angles.clear()
        theta1_deg = np.degrees(self.joint_angles[:, 0])
        theta2_deg = np.degrees(self.joint_angles[:, 1])
        theta3_deg = np.degrees(self.joint_angles[:, 2])

        self.ax_angles.plot(self.time_array, theta1_deg, 'r-', linewidth=2, label='θ1 (Hip)')
        self.ax_angles.plot(self.time_array, theta2_deg, 'g-', linewidth=2, label='θ2 (Knee)')
        self.ax_angles.plot(self.time_array, theta3_deg, 'b-', linewidth=2, label='θ3 (Ankle)')
        self.ax_angles.axvline(x=self.time_array[time_index], color='black',
                              linestyle='--', linewidth=2, label='Current Time')
        self.ax_angles.set_xlabel("Time (s)")
        self.ax_angles.set_ylabel("Angle (degrees)")
        self.ax_angles.set_title("RL - Joint Angles", fontweight='bold')
        self.ax_angles.grid(True, alpha=0.3)
        self.ax_angles.legend()

        self.canvas.draw()

    def toggle_play(self):
        """Toggle play/pause animation"""
        self.is_playing = not self.is_playing

        if self.is_playing:
            self.play_button.config(text="⏸ Pause")
            self.animate()
        else:
            self.play_button.config(text="▶ Play")
            if self.animation:
                self.animation.event_source.stop()

    def animate(self):
        """Animate the trajectory"""
        def update_frame(frame):
            time_index = int(self.current_time / self.dt)
            if time_index >= len(self.positions):
                # Loop back to start
                self.current_time = 0
                time_index = 0

            self.time_var.set(self.current_time)
            self.update_display(time_index)

            self.current_time += self.dt

            if not self.is_playing:
                return

        # Use matplotlib animation
        self.animation = FuncAnimation(self.fig, update_frame, interval=int(self.dt * 1000),
                                      blit=False, repeat=True)
        self.canvas.draw()

    def reset_animation(self):
        """Reset animation to start"""
        if self.is_playing:
            self.toggle_play()

        self.current_time = 0
        self.time_var.set(0)
        self.update_display(0)

    def on_time_changed(self, value):
        """Handle manual time slider change"""
        if self.is_playing:
            return  # Ignore manual changes during playback

        self.current_time = float(value)
        time_index = int(self.current_time / self.dt)
        if time_index < len(self.positions):
            self.update_display(time_index)


def main():
    root = tk.Tk()
    app = TrajectoryControllerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
