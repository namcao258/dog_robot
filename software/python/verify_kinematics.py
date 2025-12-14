"""
Test and Verification Script for Forward & Inverse Kinematics
Robot: Nova-SM3 Quadruped Dog - Front Left Leg

This script validates the FK and IK formulas with multiple test cases.
"""

import numpy as np
from typing import Tuple

# ============================================================================
# CONSTANTS
# ============================================================================

L1 = 60.0   # mm - Basal link (hip offset)
L2 = 105.0  # mm - Thigh
L3 = 132.0  # mm - Calf

# ============================================================================
# FORWARD KINEMATICS
# ============================================================================

def forward_kinematics(theta1: float, theta2: float, theta3: float) -> Tuple[float, float, float]:
    """
    Forward Kinematics: Calculate end-effector position from joint angles

    Args:
        theta1: Hip angle (radians)
        theta2: Knee angle (radians)
        theta3: Ankle angle (radians)

    Returns:
        (x, y, z): End-effector position in mm
        Z positive = upward
    """
    x = -L2 * np.sin(theta2) - L3 * np.sin(theta2 + theta3)

    y = L1 * np.cos(theta1) + L2 * np.sin(theta1) * np.cos(theta2) + L3 * np.sin(theta1) * np.cos(theta2 + theta3)

    # Z positive = upward (negated from original derivation)
    z = L1 * np.sin(theta1) - L2 * np.cos(theta1) * np.cos(theta2) - L3 * np.cos(theta1) * np.cos(theta2 + theta3)

    return x, y, z

# ============================================================================
# INVERSE KINEMATICS
# ============================================================================

def inverse_kinematics(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Inverse Kinematics: Calculate joint angles from end-effector position

    Args:
        x, y, z: Target position in mm (Z positive = upward)

    Returns:
        (theta1, theta2, theta3): Joint angles in radians
        Returns None if position is unreachable
    """
    # Step 1: Calculate z_temp (using -z because Z+ is now upward)
    z_temp_squared = y**2 + (-z)**2 - L1**2

    if z_temp_squared < 0:
        print(f"  [FAIL] ERROR: Target unreachable (z_temp imaginary)")
        return None

    z_temp = -np.sqrt(z_temp_squared)  # Negative because z_temp = -L2*cos(θ2) - L3*cos(θ2+θ3)

    # Step 2: Solve theta1 (adjusted for Z+ upward)
    # From derivation: sin(θ1) = -(y·z_temp + (-z)·L1)/(L1² + z_temp²)
    #                  cos(θ1) = (y·L1 - (-z)·z_temp)/(L1² + z_temp²)
    numerator = -(y * z_temp + (-z) * L1)
    denominator = y * L1 - (-z) * z_temp
    theta1 = np.arctan2(numerator, denominator)

    # Step 3: Calculate r_2D
    r_2d = np.sqrt(x**2 + z_temp**2)

    # Check reachability
    r_min = abs(L2 - L3)
    r_max = L2 + L3

    if r_2d < r_min:
        print(f"  [FAIL] ERROR: Target too close (r_2D={r_2d:.2f} < {r_min:.2f})")
        return None

    if r_2d > r_max:
        print(f"  [FAIL] ERROR: Target too far (r_2D={r_2d:.2f} > {r_max:.2f})")
        return None

    # Step 4 & 5: Solve theta3 and theta2
    # Try both elbow configurations (positive and negative theta3)
    cos_theta3 = (r_2d**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)  # Clip for numerical safety

    solutions = []
    for sign in [1, -1]:
        theta3_candidate = sign * np.arccos(cos_theta3)

        # Calculate theta2: angle from -Z axis to target, minus interior angle
        angle_to_target = np.arctan2(-x, -z_temp)  # Angle from -Z axis (downward)
        cos_beta = (L2**2 + r_2d**2 - L3**2) / (2 * L2 * r_2d)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        theta2_candidate = angle_to_target - beta

        # Verify this solution produces the correct (x, z_temp)
        x_check = -L2 * np.sin(theta2_candidate) - L3 * np.sin(theta2_candidate + theta3_candidate)
        z_temp_check = -L2 * np.cos(theta2_candidate) - L3 * np.cos(theta2_candidate + theta3_candidate)

        error = np.sqrt((x - x_check)**2 + (z_temp - z_temp_check)**2)
        solutions.append((error, theta2_candidate, theta3_candidate))

    # Choose solution with smallest error
    solutions.sort(key=lambda s: s[0])
    _, theta2, theta3 = solutions[0]

    return theta1, theta2, theta3

# ============================================================================
# TEST CASES
# ============================================================================

def test_fk_basic():
    """Test FK with basic known configurations"""
    print("\n" + "="*70)
    print("TEST 1: FORWARD KINEMATICS - Basic Configurations")
    print("="*70)

    test_cases = [
        {
            "name": "Leg straight down (all zeros)",
            "angles": (0, 0, 0),
            "expected": (0, 60, -(105 + 132))  # x=0, y=L1, z=-(L2+L3) (downward = negative z)
        },
        {
            "name": "Leg extended sideways (theta1=90°)",
            "angles": (np.radians(90), 0, 0),
            "expected": (0, 105 + 132, 60)  # Z positive when leg points up
        },
        {
            "name": "Knee bent 90° down",
            "angles": (0, np.radians(-45), np.radians(90)),
            "expected": None  # Will calculate
        },
        {
            "name": "Hip 30°, Knee -45°, Ankle 90°",
            "angles": (np.radians(30), np.radians(-45), np.radians(90)),
            "expected": None
        }
    ]

    for i, test in enumerate(test_cases, 1):
        print(f"\n[Test {i}] {test['name']}")
        theta1, theta2, theta3 = test['angles']
        print(f"  Input:  θ1={np.degrees(theta1):.1f}°, θ2={np.degrees(theta2):.1f}°, θ3={np.degrees(theta3):.1f}°")

        x, y, z = forward_kinematics(theta1, theta2, theta3)
        print(f"  Output: x={x:.2f}, y={y:.2f}, z={z:.2f} mm")

        if test['expected']:
            ex, ey, ez = test['expected']
            error = np.sqrt((x-ex)**2 + (y-ey)**2 + (z-ez)**2)
            print(f"  Expected: x={ex:.2f}, y={ey:.2f}, z={ez:.2f} mm")
            print(f"  Error: {error:.4f} mm")
            if error < 0.01:
                print("  [OK] PASS")
            else:
                print("  [FAIL] FAIL")
        else:
            print("  [OK] Calculated")

def test_ik_basic():
    """Test IK with basic known configurations"""
    print("\n" + "="*70)
    print("TEST 2: INVERSE KINEMATICS - Basic Configurations")
    print("="*70)

    test_cases = [
        {
            "name": "Straight down position",
            "position": (0, 60, -237),  # Z negative = downward
            "expected": (0, 0, 0)
        },
        {
            "name": "Mid-range position",
            "position": (-100, 80, 120),  # Z positive = upward
            "expected": None  # Will verify with FK
        },
        {
            "name": "Extended forward",
            "position": (-150, 60, 150),  # Z positive = upward
            "expected": None
        },
        {
            "name": "Close position",
            "position": (-50, 70, 80),  # Z positive = upward
            "expected": None
        }
    ]

    for i, test in enumerate(test_cases, 1):
        print(f"\n[Test {i}] {test['name']}")
        x, y, z = test['position']
        print(f"  Target: x={x:.2f}, y={y:.2f}, z={z:.2f} mm")

        result = inverse_kinematics(x, y, z)

        if result is None:
            print("  [FAIL] Position unreachable")
            continue

        theta1, theta2, theta3 = result
        print(f"  Solution: θ1={np.degrees(theta1):.2f}°, θ2={np.degrees(theta2):.2f}°, θ3={np.degrees(theta3):.2f}°")

        if test['expected']:
            e1, e2, e3 = test['expected']
            error = np.sqrt((theta1-e1)**2 + (theta2-e2)**2 + (theta3-e3)**2)
            print(f"  Expected: θ1={np.degrees(e1):.2f}°, θ2={np.degrees(e2):.2f}°, θ3={np.degrees(e3):.2f}°")
            print(f"  Angular error: {np.degrees(error):.4f}°")
            if error < 0.01:
                print("  [OK] PASS")
            else:
                print("  [FAIL] FAIL")

def test_fk_ik_roundtrip():
    """Test FK -> IK -> FK roundtrip consistency"""
    print("\n" + "="*70)
    print("TEST 3: FK -> IK -> FK ROUNDTRIP TEST")
    print("="*70)
    print("Verify that IK(FK(angles)) returns the same position")

    test_angles = [
        (0, 0, 0),
        (np.radians(30), np.radians(-45), np.radians(90)),
        (np.radians(-20), np.radians(-60), np.radians(120)),
        (np.radians(45), np.radians(-30), np.radians(60)),
        (np.radians(-45), np.radians(-90), np.radians(90)),
    ]

    passed = 0
    failed = 0

    for i, (t1, t2, t3) in enumerate(test_angles, 1):
        print(f"\n[Test {i}] θ1={np.degrees(t1):.1f}°, θ2={np.degrees(t2):.1f}°, θ3={np.degrees(t3):.1f}°")

        # FK: angles -> position
        x, y, z = forward_kinematics(t1, t2, t3)
        print(f"  FK:  → x={x:.2f}, y={y:.2f}, z={z:.2f}")

        # IK: position -> angles
        result = inverse_kinematics(x, y, z)
        if result is None:
            print(f"  [FAIL] IK failed (position unreachable)")
            failed += 1
            continue

        t1_calc, t2_calc, t3_calc = result
        print(f"  IK:  → θ1={np.degrees(t1_calc):.2f}°, θ2={np.degrees(t2_calc):.2f}°, θ3={np.degrees(t3_calc):.2f}°")

        # FK again: angles -> position
        x_verify, y_verify, z_verify = forward_kinematics(t1_calc, t2_calc, t3_calc)
        print(f"  FK2: → x={x_verify:.2f}, y={y_verify:.2f}, z={z_verify:.2f}")

        # Check position error
        pos_error = np.sqrt((x - x_verify)**2 + (y - y_verify)**2 + (z - z_verify)**2)
        print(f"  Position error: {pos_error:.6f} mm")

        if pos_error < 0.01:
            print("  [OK] PASS")
            passed += 1
        else:
            print("  [FAIL] FAIL")
            failed += 1

    print(f"\n{'='*70}")
    print(f"Results: {passed} passed, {failed} failed")
    print(f"{'='*70}")

def test_ik_fk_roundtrip():
    """Test IK -> FK -> IK roundtrip consistency"""
    print("\n" + "="*70)
    print("TEST 4: IK -> FK -> IK ROUNDTRIP TEST")
    print("="*70)
    print("Verify that FK(IK(position)) returns the same angles")

    test_positions = [
        (-100, 80, 120),   # Z positive = upward
        (-150, 70, 100),   # Z positive = upward
        (-80, 90, 180),    # Z positive = upward
        (-120, 65, 140),   # Z positive = upward
        (-50, 100, 100),   # Z positive = upward
    ]

    passed = 0
    failed = 0

    for i, (x, y, z) in enumerate(test_positions, 1):
        print(f"\n[Test {i}] x={x:.1f}, y={y:.1f}, z={z:.1f}")

        # IK: position -> angles
        result = inverse_kinematics(x, y, z)
        if result is None:
            print(f"  [WARN]  Position unreachable, skipping")
            continue

        t1, t2, t3 = result
        print(f"  IK:  → θ1={np.degrees(t1):.2f}°, θ2={np.degrees(t2):.2f}°, θ3={np.degrees(t3):.2f}°")

        # FK: angles -> position
        x_calc, y_calc, z_calc = forward_kinematics(t1, t2, t3)
        print(f"  FK:  → x={x_calc:.2f}, y={y_calc:.2f}, z={z_calc:.2f}")

        # Check position error
        pos_error = np.sqrt((x - x_calc)**2 + (y - y_calc)**2 + (z - z_calc)**2)
        print(f"  Position error: {pos_error:.6f} mm")

        if pos_error < 0.01:
            print("  [OK] PASS")
            passed += 1
        else:
            print("  [FAIL] FAIL")
            failed += 1

    print(f"\n{'='*70}")
    print(f"Results: {passed} passed, {failed} failed")
    print(f"{'='*70}")

def test_boundary_cases():
    """Test edge cases and boundaries"""
    print("\n" + "="*70)
    print("TEST 5: BOUNDARY AND EDGE CASES")
    print("="*70)

    print("\n[Test 5.1] Maximum reach - straight down")
    x, y, z = (0, 60, -(L2 + L3))  # Z negative = downward
    print(f"  Position: x={x}, y={y}, z={z} (straight down)")
    result = inverse_kinematics(x, y, z)
    if result:
        print(f"  [OK] Reachable")
    else:
        print(f"  [FAIL] Should be reachable!")

    print("\n[Test 5.2] Minimum reach - folded")
    x, y, z = (0, 60, -abs(L2 - L3))  # Z negative = downward
    print(f"  Position: x={x}, y={y}, z={z} (folded)")
    result = inverse_kinematics(x, y, z)
    if result:
        print(f"  [OK] Reachable")
    else:
        print(f"  [FAIL] Should be reachable!")

    print("\n[Test 5.3] Too far away")
    x, y, z = (-300, 60, 100)  # Z positive = upward
    print(f"  Position: x={x}, y={y}, z={z}")
    result = inverse_kinematics(x, y, z)
    if result is None:
        print(f"  [OK] Correctly rejected")
    else:
        print(f"  [FAIL] Should be unreachable!")

    print("\n[Test 5.4] Too close (lateral)")
    x, y, z = (0, 30, 100)  # y < L1, Z positive = upward
    print(f"  Position: x={x}, y={y}, z={z}")
    result = inverse_kinematics(x, y, z)
    if result is None:
        print(f"  [OK] Correctly rejected")
    else:
        print(f"  [FAIL] Should be unreachable!")

# ============================================================================
# MAIN TEST RUNNER
# ============================================================================

def run_all_tests():
    """Run all test suites"""
    print("\n" + "="*70)
    print("KINEMATICS VERIFICATION TEST SUITE")
    print("Robot: Nova-SM3 Quadruped Dog - Front Left Leg")
    print("="*70)
    print(f"\nConstants:")
    print(f"  L1 (Basal)  = {L1} mm")
    print(f"  L2 (Thigh)  = {L2} mm")
    print(f"  L3 (Calf)   = {L3} mm")
    print(f"  Max reach   = {L2 + L3} mm")
    print(f"  Min reach   = {abs(L2 - L3)} mm")

    # Run all tests
    test_fk_basic()
    test_ik_basic()
    test_fk_ik_roundtrip()
    test_ik_fk_roundtrip()
    test_boundary_cases()

    print("\n" + "="*70)
    print("ALL TESTS COMPLETED")
    print("="*70)

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    run_all_tests()
