"""
Kinematics Visualizer - FK and IK Testing GUI
Nova-SM3 Quadruped Robot - Front Left Leg

Features:
- Left panel: Forward Kinematics with 3D visualization
- Right panel: Inverse Kinematics with 3D visualization
- Top panel: Robot parameters (L1, L2, L3, homing angles)
- Interactive 3D visualization of leg configuration

Author: Verified implementation
Date: 2025-12-12
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QLineEdit, QPushButton,
                             QGroupBox, QGridLayout, QSplitter)
from PyQt5.QtCore import Qt


class KinematicsVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()

        # Default robot parameters
        self.L1 = 60.0  # Basal link (mm)
        self.L2 = 105.0  # Thigh (mm)
        self.L3 = 132.0  # Calf (mm)
        self.homing = [0.0, 0.0, 0.0]  # Homing angles (degrees)

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Kinematics Visualizer - Nova-SM3 Robot')
        self.setGeometry(100, 100, 1600, 900)

        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # Top panel: Robot parameters
        param_group = self.create_parameter_panel()
        main_layout.addWidget(param_group)

        # Splitter for FK and IK panels
        splitter = QSplitter(Qt.Horizontal)

        # Left panel: Forward Kinematics
        fk_panel = self.create_fk_panel()
        splitter.addWidget(fk_panel)

        # Right panel: Inverse Kinematics
        ik_panel = self.create_ik_panel()
        splitter.addWidget(ik_panel)

        splitter.setSizes([800, 800])
        main_layout.addWidget(splitter)

    def create_parameter_panel(self):
        """Create top parameter input panel"""
        param_group = QGroupBox("Robot Parameters")
        layout = QGridLayout()

        # Link lengths
        layout.addWidget(QLabel("L1 (mm):"), 0, 0)
        self.l1_input = QLineEdit("60.0")
        layout.addWidget(self.l1_input, 0, 1)

        layout.addWidget(QLabel("L2 (mm):"), 0, 2)
        self.l2_input = QLineEdit("105.0")
        layout.addWidget(self.l2_input, 0, 3)

        layout.addWidget(QLabel("L3 (mm):"), 0, 4)
        self.l3_input = QLineEdit("132.0")
        layout.addWidget(self.l3_input, 0, 5)

        # Homing angles
        layout.addWidget(QLabel("Homing θ1 (°):"), 1, 0)
        self.home1_input = QLineEdit("0.0")
        layout.addWidget(self.home1_input, 1, 1)

        layout.addWidget(QLabel("Homing θ2 (°):"), 1, 2)
        self.home2_input = QLineEdit("0.0")
        layout.addWidget(self.home2_input, 1, 3)

        layout.addWidget(QLabel("Homing θ3 (°):"), 1, 4)
        self.home3_input = QLineEdit("0.0")
        layout.addWidget(self.home3_input, 1, 5)

        # Update button
        update_btn = QPushButton("Update Parameters")
        update_btn.clicked.connect(self.update_parameters)
        layout.addWidget(update_btn, 0, 6, 2, 1)

        param_group.setLayout(layout)
        return param_group

    def create_fk_panel(self):
        """Create Forward Kinematics panel"""
        fk_widget = QWidget()
        layout = QVBoxLayout(fk_widget)

        # Title
        title = QLabel("Forward Kinematics")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Input section
        input_group = QGroupBox("Input: Joint Angles (degrees)")
        input_layout = QGridLayout()

        input_layout.addWidget(QLabel("θ1 (Hip):"), 0, 0)
        self.fk_theta1 = QLineEdit("0.0")
        input_layout.addWidget(self.fk_theta1, 0, 1)

        input_layout.addWidget(QLabel("θ2 (Knee):"), 1, 0)
        self.fk_theta2 = QLineEdit("0.0")
        input_layout.addWidget(self.fk_theta2, 1, 1)

        input_layout.addWidget(QLabel("θ3 (Ankle):"), 2, 0)
        self.fk_theta3 = QLineEdit("0.0")
        input_layout.addWidget(self.fk_theta3, 2, 1)

        fk_calc_btn = QPushButton("Calculate FK")
        fk_calc_btn.clicked.connect(self.calculate_fk)
        input_layout.addWidget(fk_calc_btn, 3, 0, 1, 2)

        input_group.setLayout(input_layout)
        layout.addWidget(input_group)

        # Output section
        output_group = QGroupBox("Output: End-Effector Position (mm)")
        output_layout = QGridLayout()

        output_layout.addWidget(QLabel("X:"), 0, 0)
        self.fk_x_output = QLineEdit()
        self.fk_x_output.setReadOnly(True)
        output_layout.addWidget(self.fk_x_output, 0, 1)

        output_layout.addWidget(QLabel("Y:"), 1, 0)
        self.fk_y_output = QLineEdit()
        self.fk_y_output.setReadOnly(True)
        output_layout.addWidget(self.fk_y_output, 1, 1)

        output_layout.addWidget(QLabel("Z:"), 2, 0)
        self.fk_z_output = QLineEdit()
        self.fk_z_output.setReadOnly(True)
        output_layout.addWidget(self.fk_z_output, 2, 1)

        output_group.setLayout(output_layout)
        layout.addWidget(output_group)

        # 3D Visualization
        self.fk_figure = Figure(figsize=(6, 6))
        self.fk_canvas = FigureCanvas(self.fk_figure)
        self.fk_ax = self.fk_figure.add_subplot(111, projection='3d')
        layout.addWidget(self.fk_canvas)

        return fk_widget

    def create_ik_panel(self):
        """Create Inverse Kinematics panel"""
        ik_widget = QWidget()
        layout = QVBoxLayout(ik_widget)

        # Title
        title = QLabel("Inverse Kinematics")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Input section
        input_group = QGroupBox("Input: End-Effector Position (mm)")
        input_layout = QGridLayout()

        input_layout.addWidget(QLabel("X:"), 0, 0)
        self.ik_x = QLineEdit("0.0")
        input_layout.addWidget(self.ik_x, 0, 1)

        input_layout.addWidget(QLabel("Y:"), 1, 0)
        self.ik_y = QLineEdit("60.0")
        input_layout.addWidget(self.ik_y, 1, 1)

        input_layout.addWidget(QLabel("Z:"), 2, 0)
        self.ik_z = QLineEdit("-237.0")  # Z negative = downward
        input_layout.addWidget(self.ik_z, 2, 1)

        ik_calc_btn = QPushButton("Calculate IK")
        ik_calc_btn.clicked.connect(self.calculate_ik)
        input_layout.addWidget(ik_calc_btn, 3, 0, 1, 2)

        input_group.setLayout(input_layout)
        layout.addWidget(input_group)

        # Output section
        output_group = QGroupBox("Output: Joint Angles (degrees)")
        output_layout = QGridLayout()

        output_layout.addWidget(QLabel("θ1 (Hip):"), 0, 0)
        self.ik_theta1_output = QLineEdit()
        self.ik_theta1_output.setReadOnly(True)
        output_layout.addWidget(self.ik_theta1_output, 0, 1)

        output_layout.addWidget(QLabel("θ2 (Knee):"), 1, 0)
        self.ik_theta2_output = QLineEdit()
        self.ik_theta2_output.setReadOnly(True)
        output_layout.addWidget(self.ik_theta2_output, 1, 1)

        output_layout.addWidget(QLabel("θ3 (Ankle):"), 2, 0)
        self.ik_theta3_output = QLineEdit()
        self.ik_theta3_output.setReadOnly(True)
        output_layout.addWidget(self.ik_theta3_output, 2, 1)

        output_group.setLayout(output_layout)
        layout.addWidget(output_group)

        # Solutions info label
        self.ik_solutions_label = QLabel("Number of solutions: -")
        self.ik_solutions_label.setAlignment(Qt.AlignCenter)
        self.ik_solutions_label.setStyleSheet("font-weight: bold; color: blue;")
        layout.addWidget(self.ik_solutions_label)

        # 3D Visualization - now with multiple subplots
        self.ik_figure = Figure(figsize=(8, 4))
        self.ik_canvas = FigureCanvas(self.ik_figure)
        layout.addWidget(self.ik_canvas)

        return ik_widget

    def update_parameters(self):
        """Update robot parameters from inputs"""
        try:
            self.L1 = float(self.l1_input.text())
            self.L2 = float(self.l2_input.text())
            self.L3 = float(self.l3_input.text())
            self.homing = [
                float(self.home1_input.text()),
                float(self.home2_input.text()),
                float(self.home3_input.text())
            ]
            print(f"Parameters updated: L1={self.L1}, L2={self.L2}, L3={self.L3}")
            print(f"Homing: {self.homing}")
        except ValueError:
            print("Error: Invalid parameter values")

    def forward_kinematics(self, theta1_deg, theta2_deg, theta3_deg):
        """Calculate forward kinematics"""
        # Convert to radians
        theta1 = np.radians(theta1_deg)
        theta2 = np.radians(theta2_deg)
        theta3 = np.radians(theta3_deg)

        # FK formulas
        x = -self.L2 * np.sin(theta2) - self.L3 * np.sin(theta2 + theta3)
        y = self.L1 * np.cos(theta1) + self.L2 * np.sin(theta1) * np.cos(theta2) + \
            self.L3 * np.sin(theta1) * np.cos(theta2 + theta3)
        # Z positive = upward
        z = self.L1 * np.sin(theta1) - self.L2 * np.cos(theta1) * np.cos(theta2) - \
            self.L3 * np.cos(theta1) * np.cos(theta2 + theta3)

        return x, y, z

    def inverse_kinematics(self, x, y, z, return_all=False):
        """Calculate inverse kinematics (Z positive = upward)"""
        # Step 1: Calculate z_temp (using -z because Z+ is upward)
        z_temp_squared = y**2 + (-z)**2 - self.L1**2
        if z_temp_squared < 0:
            return None if not return_all else []

        z_temp = -np.sqrt(z_temp_squared)

        # Step 2: Solve theta1 (adjusted for Z+ upward)
        numerator = -(y * z_temp + (-z) * self.L1)
        denominator = y * self.L1 - (-z) * z_temp
        theta1 = np.arctan2(numerator, denominator)

        # Step 3: Calculate 2D reach
        r_2d = np.sqrt(x**2 + z_temp**2)

        # Check reachability
        r_min = abs(self.L2 - self.L3)
        r_max = self.L2 + self.L3
        if r_2d < r_min or r_2d > r_max:
            return None if not return_all else []

        # Step 4 & 5: Solve theta2 and theta3
        cos_theta3 = (r_2d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)

        solutions = []
        for sign in [1, -1]:
            theta3_candidate = sign * np.arccos(cos_theta3)

            angle_to_target = np.arctan2(-x, -z_temp)
            cos_beta = (self.L2**2 + r_2d**2 - self.L3**2) / (2 * self.L2 * r_2d)
            cos_beta = np.clip(cos_beta, -1.0, 1.0)
            beta = np.arccos(cos_beta)

            # Different formula for elbow up vs elbow down
            if sign > 0:  # Elbow up (theta3 > 0)
                theta2_candidate = angle_to_target - beta
            else:  # Elbow down (theta3 < 0)
                theta2_candidate = angle_to_target + beta

            # Verify solution
            x_check = -self.L2 * np.sin(theta2_candidate) - self.L3 * np.sin(theta2_candidate + theta3_candidate)
            z_temp_check = -self.L2 * np.cos(theta2_candidate) - self.L3 * np.cos(theta2_candidate + theta3_candidate)

            error = np.sqrt((x - x_check)**2 + (z_temp - z_temp_check)**2)
            solutions.append((error, theta2_candidate, theta3_candidate))

        solutions.sort(key=lambda s: s[0])

        # Return all solutions or just the best one
        if return_all:
            # Return ALL solutions (both elbow configurations) with their errors
            all_solutions = []
            for error, theta2, theta3 in solutions:
                all_solutions.append((theta1, theta2, theta3, error))
            return all_solutions
        else:
            _, theta2, theta3 = solutions[0]
            return theta1, theta2, theta3

    def calculate_fk(self):
        """Calculate and display FK results"""
        try:
            theta1 = float(self.fk_theta1.text())
            theta2 = float(self.fk_theta2.text())
            theta3 = float(self.fk_theta3.text())

            x, y, z = self.forward_kinematics(theta1, theta2, theta3)

            self.fk_x_output.setText(f"{x:.2f}")
            self.fk_y_output.setText(f"{y:.2f}")
            self.fk_z_output.setText(f"{z:.2f}")

            self.plot_3d_leg(self.fk_ax, self.fk_canvas, theta1, theta2, theta3, "FK")

        except ValueError as e:
            print(f"FK Error: {e}")

    def calculate_ik(self):
        """Calculate and display ALL IK solutions"""
        try:
            x = float(self.ik_x.text())
            y = float(self.ik_y.text())
            z = float(self.ik_z.text())

            # Get all solutions
            all_solutions = self.inverse_kinematics(x, y, z, return_all=True)

            if not all_solutions:
                self.ik_theta1_output.setText("Unreachable")
                self.ik_theta2_output.setText("Unreachable")
                self.ik_theta3_output.setText("Unreachable")
                self.ik_solutions_label.setText("Number of solutions: 0 (Unreachable)")
                print("IK: Target position unreachable")
                return

            # Display the best solution in output fields
            theta1, theta2, theta3, error = all_solutions[0]
            theta1_deg = np.degrees(theta1)
            theta2_deg = np.degrees(theta2)
            theta3_deg = np.degrees(theta3)

            self.ik_theta1_output.setText(f"{theta1_deg:.2f}°")
            self.ik_theta2_output.setText(f"{theta2_deg:.2f}°")
            self.ik_theta3_output.setText(f"{theta3_deg:.2f}°")

            # Update solutions count
            self.ik_solutions_label.setText(f"Number of solutions: {len(all_solutions)}")

            # Clear previous figure and create subplots for all solutions
            self.ik_figure.clear()

            num_solutions = len(all_solutions)
            for i, (t1, t2, t3, err) in enumerate(all_solutions, 1):
                ax = self.ik_figure.add_subplot(1, num_solutions, i, projection='3d')
                t1_deg = np.degrees(t1)
                t2_deg = np.degrees(t2)
                t3_deg = np.degrees(t3)

                title = f"Solution {i}\n(err={err:.4f}mm)"
                self.plot_3d_leg_on_axis(ax, t1_deg, t2_deg, t3_deg, title)

            self.ik_canvas.draw()

        except ValueError as e:
            print(f"IK Error: {e}")

    def plot_3d_leg_on_axis(self, ax, theta1_deg, theta2_deg, theta3_deg, title):
        """Plot 3D visualization of robot leg on a given axis"""
        ax.clear()

        # Convert to radians
        theta1 = np.radians(theta1_deg)
        theta2 = np.radians(theta2_deg)
        theta3 = np.radians(theta3_deg)

        # Calculate joint positions (Z positive = upward)
        # Origin
        origin = np.array([0, 0, 0])

        # Hip joint (after L1)
        hip = np.array([
            0,
            self.L1 * np.cos(theta1),
            self.L1 * np.sin(theta1)  # Z+ upward
        ])

        # Knee joint
        knee = hip + np.array([
            -self.L2 * np.sin(theta2),
            self.L2 * np.sin(theta1) * np.cos(theta2),
            -self.L2 * np.cos(theta1) * np.cos(theta2)  # Z+ upward
        ])

        # End-effector
        x, y, z = self.forward_kinematics(theta1_deg, theta2_deg, theta3_deg)
        end_effector = np.array([x, y, z])

        # Plot links
        # L1 (origin to hip)
        ax.plot3D([origin[0], hip[0]], [origin[1], hip[1]], [origin[2], hip[2]],
                  'b-', linewidth=4, label='L1 (Basal)')

        # L2 (hip to knee)
        ax.plot3D([hip[0], knee[0]], [hip[1], knee[1]], [hip[2], knee[2]],
                  'g-', linewidth=4, label='L2 (Thigh)')

        # L3 (knee to end-effector)
        ax.plot3D([knee[0], end_effector[0]], [knee[1], end_effector[1]],
                  [knee[2], end_effector[2]], 'r-', linewidth=4, label='L3 (Calf)')

        # Plot joints
        ax.scatter(*origin, color='black', s=100, label='Origin')
        ax.scatter(*hip, color='blue', s=100, label='Hip')
        ax.scatter(*knee, color='green', s=100, label='Knee')
        ax.scatter(*end_effector, color='red', s=100, label='End-Effector')

        # Plot coordinate axes
        axis_length = 50
        ax.quiver(0, 0, 0, axis_length, 0, 0, color='red', alpha=0.3, arrow_length_ratio=0.1)
        ax.quiver(0, 0, 0, 0, axis_length, 0, color='green', alpha=0.3, arrow_length_ratio=0.1)
        ax.quiver(0, 0, 0, 0, 0, axis_length, color='blue', alpha=0.3, arrow_length_ratio=0.1)

        # Set labels and limits
        ax.set_xlabel('X', fontsize=8)
        ax.set_ylabel('Y', fontsize=8)
        ax.set_zlabel('Z', fontsize=8)
        ax.set_title(f'{title}\nθ1={theta1_deg:.1f}° θ2={theta2_deg:.1f}° θ3={theta3_deg:.1f}°',
                     fontsize=9)

        # Set equal aspect ratio (Z+ upward)
        max_range = max(self.L1 + self.L2 + self.L3, 300)
        ax.set_xlim([-max_range/2, max_range/2])
        ax.set_ylim([-max_range/4, max_range])
        ax.set_zlim([-max_range/4, max_range])  # Z+ upward

        # Smaller legend for compact view
        ax.legend(fontsize=6, loc='upper left')

    def plot_3d_leg(self, ax, canvas, theta1_deg, theta2_deg, theta3_deg, title):
        """Plot 3D visualization of robot leg (for FK panel)"""
        self.plot_3d_leg_on_axis(ax, theta1_deg, theta2_deg, theta3_deg, title)
        canvas.draw()


def main():
    app = QApplication(sys.argv)
    visualizer = KinematicsVisualizer()
    visualizer.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
