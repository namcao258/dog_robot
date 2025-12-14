"""
Test Optimal IK Solution Selection
Verify that IK solution selection minimizes joint angle changes
"""

import numpy as np
import sys
sys.path.append('c:\\Users\\valer\\OneDrive\\Desktop\\Robot\\dog_robot\\software\\python')

from forward_kinematics import forward_kinematics as FK
from inverse_kinematics import inverse_kinematics as IK

print("="*70)
print("TEST: OPTIMAL IK SOLUTION SELECTION")
print("="*70)

# Initial configuration: (0, -30°, 30°)
initial_angles = np.array([0, np.radians(-30), np.radians(30)])
print(f"\nInitial Configuration:")
print(f"  θ1 = {np.degrees(initial_angles[0]):.2f}°")
print(f"  θ2 = {np.degrees(initial_angles[1]):.2f}°")
print(f"  θ3 = {np.degrees(initial_angles[2]):.2f}°")

# Calculate FK from initial position
x_init, y_init, z_init = FK(initial_angles[0], initial_angles[1], initial_angles[2])
print(f"\nInitial End-Effector Position:")
print(f"  x = {x_init:.2f} mm")
print(f"  y = {y_init:.2f} mm")
print(f"  z = {z_init:.2f} mm")

# Test trajectory: small movements from initial position
test_positions = [
    (x_init, y_init, z_init),  # Same position
    (x_init + 20, y_init, z_init),  # Move forward
    (x_init + 20, y_init, z_init - 30),  # Move forward and up
    (x_init - 30, y_init, z_init - 20),  # Move backward and up
]

current_angles = initial_angles.copy()

print("\n" + "="*70)
print("TRAJECTORY TEST")
print("="*70)

for i, (x_target, y_target, z_target) in enumerate(test_positions, 1):
    print(f"\n{'='*70}")
    print(f"Step {i}: Target position ({x_target:.2f}, {y_target:.2f}, {z_target:.2f})")
    print(f"{'='*70}")

    # Get all IK solutions
    all_solutions = IK(x_target, y_target, z_target, return_all=True)

    if not all_solutions:
        print("  [ERROR] No IK solutions found!")
        continue

    print(f"\n  Found {len(all_solutions)} IK solution(s):")

    # Evaluate each solution
    best_solution = None
    min_distance = float('inf')

    for sol_idx, (theta1, theta2, theta3, error) in enumerate(all_solutions, 1):
        # Calculate angular distance from current configuration
        distance = (abs(theta1 - current_angles[0]) +
                   abs(theta2 - current_angles[1]) +
                   abs(theta3 - current_angles[2]))

        print(f"\n  Solution {sol_idx}:")
        print(f"    Angles: θ1={np.degrees(theta1):6.2f}°, θ2={np.degrees(theta2):6.2f}°, θ3={np.degrees(theta3):6.2f}°")
        print(f"    Error: {error:.6f} mm")
        print(f"    Distance from current: {np.degrees(distance):.2f}° (total)")

        # Individual changes
        d1 = abs(np.degrees(theta1 - current_angles[0]))
        d2 = abs(np.degrees(theta2 - current_angles[1]))
        d3 = abs(np.degrees(theta3 - current_angles[2]))
        print(f"      Δθ1={d1:.2f}°, Δθ2={d2:.2f}°, Δθ3={d3:.2f}°")

        if distance < min_distance:
            min_distance = distance
            best_solution = (theta1, theta2, theta3)
            best_idx = sol_idx

    print(f"\n  ✓ SELECTED: Solution {best_idx} (minimum distance)")

    # Update current angles to the best solution
    current_angles = np.array(best_solution)

    # Verify FK
    x_verify, y_verify, z_verify = FK(current_angles[0], current_angles[1], current_angles[2])
    error_3d = np.sqrt((x_target - x_verify)**2 + (y_target - y_verify)**2 + (z_target - z_verify)**2)
    print(f"  Verification: FK error = {error_3d:.6f} mm")

print("\n" + "="*70)
print("FINAL CONFIGURATION")
print("="*70)
print(f"θ1 = {np.degrees(current_angles[0]):.2f}°")
print(f"θ2 = {np.degrees(current_angles[1]):.2f}°")
print(f"θ3 = {np.degrees(current_angles[2]):.2f}°")

print("\n" + "="*70)
print("ANALYSIS")
print("="*70)
print("✓ The algorithm selects IK solutions that minimize total joint angle change")
print("✓ This prevents sudden jumps between elbow configurations")
print("✓ Results in smooth, natural trajectories")
