"""
Forward Kinematics (FK) for Nova-SM3 Quadruped Robot
Front Left Leg - 3-DOF

Link lengths:
- L1 = 60 mm (Basal/Hip offset)
- L2 = 105 mm (Thigh)
- L3 = 132 mm (Calf)

Author: Verified implementation
Date: 2025-12-12
"""

import numpy as np
from typing import Tuple

# Link lengths (mm)
L1 = 60.0   # Basal link (hip offset)
L2 = 105.0  # Thigh
L3 = 132.0  # Calf


def forward_kinematics(theta1: float, theta2: float, theta3: float) -> Tuple[float, float, float]:
    """
    Calculate end-effector position from joint angles.

    Args:
        theta1: Hip angle (radians) - rotation around X-axis
        theta2: Knee angle (radians) - rotation around Y-axis
        theta3: Ankle angle (radians) - rotation around Y-axis

    Returns:
        (x, y, z): End-effector position in mm
            x: Forward/backward (negative = forward)
            y: Lateral (positive = outward)
            z: Vertical (positive = upward)

    Formulas:
        x = -L2·sin(θ2) - L3·sin(θ2 + θ3)
        y = L1·cos(θ1) + L2·sin(θ1)·cos(θ2) + L3·sin(θ1)·cos(θ2 + θ3)
        z = L1·sin(θ1) - L2·cos(θ1)·cos(θ2) - L3·cos(θ1)·cos(θ2 + θ3)
    """
    x = -L2 * np.sin(theta2) - L3 * np.sin(theta2 + theta3)

    y = L1 * np.cos(theta1) + L2 * np.sin(theta1) * np.cos(theta2) + \
        L3 * np.sin(theta1) * np.cos(theta2 + theta3)

    # Z positive = upward (negated from original derivation)
    z = L1 * np.sin(theta1) - L2 * np.cos(theta1) * np.cos(theta2) - \
        L3 * np.cos(theta1) * np.cos(theta2 + theta3)

    return x, y, z


if __name__ == "__main__":
    # Test examples
    print("="*60)
    print("FORWARD KINEMATICS TEST")
    print("="*60)

    test_cases = [
        ("Straight down", 0, 0, 0),
        ("Leg sideways", np.radians(90), 0, 0),
        ("Knee bent", 0, np.radians(-45), np.radians(90)),
    ]

    for name, t1, t2, t3 in test_cases:
        x, y, z = forward_kinematics(t1, t2, t3)
        print(f"\n{name}:")
        print(f"  Input:  θ1={np.degrees(t1):.1f}°, θ2={np.degrees(t2):.1f}°, θ3={np.degrees(t3):.1f}°")
        print(f"  Output: x={x:.2f}, y={y:.2f}, z={z:.2f} mm")
