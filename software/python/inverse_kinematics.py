"""
Inverse Kinematics (IK) for Nova-SM3 Quadruped Robot
Front Left Leg - 3-DOF

Link lengths:
- L1 = 60 mm (Basal/Hip offset)
- L2 = 105 mm (Thigh)
- L3 = 132 mm (Calf)

Author: Verified implementation
Date: 2025-12-12
"""

import numpy as np
from typing import Tuple, Optional

# Link lengths (mm)
L1 = 60.0   # Basal link (hip offset)
L2 = 105.0  # Thigh
L3 = 132.0  # Calf


def inverse_kinematics(x: float, y: float, z: float, return_all: bool = False) -> Optional[Tuple[float, float, float]]:
    """
    Calculate joint angles from target end-effector position.

    Args:
        x: Target X position (mm) - forward/backward
        y: Target Y position (mm) - lateral
        z: Target Z position (mm) - vertical (positive = upward)
        return_all: If True, return all valid solutions; if False, return best solution

    Returns:
        If return_all=False: (theta1, theta2, theta3) - best solution in radians
        If return_all=True: List of (theta1, theta2, theta3, error) tuples
        Returns None (or empty list) if position is unreachable

    Algorithm:
        1. Calculate z_temp = -√(y² + z² - L1²)
        2. Solve θ1 from rotation equations (adjusted for Z+ up)
        3. Calculate r_2D = √(x² + z_temp²)
        4. Solve θ3 using law of cosines (2 solutions)
        5. Solve θ2 using angle decomposition
        6. Choose solution that matches target x (or return all)
    """
    # Step 1: Calculate z_temp (using -z because Z+ is now upward)
    z_temp_squared = y**2 + (-z)**2 - L1**2

    if z_temp_squared < 0:
        print(f"ERROR: Target unreachable (y²+z²-L1² < 0)")
        return None

    z_temp = -np.sqrt(z_temp_squared)

    # Step 2: Solve theta1
    # Formula: θ1 = atan2(-(y·z_temp + (-z)·L1), y·L1 - (-z)·z_temp)
    # Adjusted for Z+ upward convention
    numerator = -(y * z_temp + (-z) * L1)
    denominator = y * L1 - (-z) * z_temp
    theta1 = np.arctan2(numerator, denominator)

    # Step 3: Calculate 2D reach
    r_2d = np.sqrt(x**2 + z_temp**2)

    # Check reachability
    r_min = abs(L2 - L3)
    r_max = L2 + L3

    if r_2d < r_min:
        print(f"ERROR: Target too close (r_2D={r_2d:.2f} < {r_min:.2f})")
        return None

    if r_2d > r_max:
        print(f"ERROR: Target too far (r_2D={r_2d:.2f} > {r_max:.2f})")
        return None

    # Step 4 & 5: Solve theta2 and theta3
    # Try both elbow configurations (elbow up/down)
    cos_theta3 = (r_2d**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)

    solutions = []
    for sign in [1, -1]:
        theta3_candidate = sign * np.arccos(cos_theta3)

        # Calculate theta2: angle from -Z axis to target, +/- interior angle
        angle_to_target = np.arctan2(-x, -z_temp)
        cos_beta = (L2**2 + r_2d**2 - L3**2) / (2 * L2 * r_2d)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)

        # Different formula for elbow up vs elbow down
        if sign > 0:  # Elbow up (theta3 > 0)
            theta2_candidate = angle_to_target - beta
        else:  # Elbow down (theta3 < 0)
            theta2_candidate = angle_to_target + beta

        # Verify solution produces correct (x, z_temp)
        x_check = -L2 * np.sin(theta2_candidate) - L3 * np.sin(theta2_candidate + theta3_candidate)
        z_temp_check = -L2 * np.cos(theta2_candidate) - L3 * np.cos(theta2_candidate + theta3_candidate)

        error = np.sqrt((x - x_check)**2 + (z_temp - z_temp_check)**2)
        solutions.append((error, theta2_candidate, theta3_candidate))

    # Sort solutions by error
    solutions.sort(key=lambda s: s[0])

    # Return all solutions or just the best one
    if return_all:
        # Return ALL solutions (both elbow configurations) with their errors
        # Even if error is slightly higher due to numerical precision
        all_solutions = []
        for error, theta2, theta3 in solutions:
            all_solutions.append((theta1, theta2, theta3, error))
        return all_solutions if all_solutions else None
    else:
        # Return best solution
        _, theta2, theta3 = solutions[0]
        return theta1, theta2, theta3


if __name__ == "__main__":
    # Test examples
    print("="*60)
    print("INVERSE KINEMATICS TEST")
    print("="*60)

    test_cases = [
        ("Straight down", 0, 60, -237),  # Z negative = downward
        ("Mid-range", -100, 80, 120),     # Z positive = upward
        ("Extended forward", -150, 60, 150),  # Z positive = upward
    ]

    for name, x_t, y_t, z_t in test_cases:
        print(f"\n{name}:")
        print(f"  Target: x={x_t}, y={y_t}, z={z_t}")

        # Get best solution
        result = inverse_kinematics(x_t, y_t, z_t, return_all=False)
        if result:
            t1, t2, t3 = result
            print(f"  Best Solution: θ1={np.degrees(t1):.2f}°, θ2={np.degrees(t2):.2f}°, θ3={np.degrees(t3):.2f}°")
        else:
            print(f"  No solution found")

        # Get all solutions
        all_solutions = inverse_kinematics(x_t, y_t, z_t, return_all=True)
        if all_solutions:
            print(f"  All {len(all_solutions)} solution(s):")
            for i, (t1, t2, t3, err) in enumerate(all_solutions, 1):
                print(f"    [{i}] θ1={np.degrees(t1):.2f}°, θ2={np.degrees(t2):.2f}°, θ3={np.degrees(t3):.2f}° (error={err:.4f}mm)")
        else:
            print(f"  No valid solutions")
