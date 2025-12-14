"""
Test IK Error Calculation
Verify that when we use FK angles in IK, error should be 0
"""

import numpy as np
import sys
sys.path.append('c:\\Users\\valer\\OneDrive\\Desktop\\Robot\\dog_robot\\software\\python')

from forward_kinematics import forward_kinematics as FK
from inverse_kinematics import inverse_kinematics as IK

# Link lengths
L1 = 60.0
L2 = 105.0
L3 = 132.0

print("="*70)
print("TEST: IK ERROR VERIFICATION")
print("="*70)

# Test case: bạn đã thử với position (-52.50, 60.00, -222.93)
# Hãy tìm angles từ FK tạo ra position này
test_angles = [
    (0, np.radians(-30), np.radians(-60)),
    (0, np.radians(-30), np.radians(60)),
]

for idx, (t1, t2, t3) in enumerate(test_angles, 1):
    print(f"\n{'='*70}")
    print(f"TEST {idx}: θ1={np.degrees(t1):.2f}°, θ2={np.degrees(t2):.2f}°, θ3={np.degrees(t3):.2f}°")
    print(f"{'='*70}")

    # Step 1: FK to get position
    x_fk, y_fk, z_fk = FK(t1, t2, t3)
    print(f"FK Result: x={x_fk:.4f}, y={y_fk:.4f}, z={z_fk:.4f}")

    # Step 2: IK to get angles back
    solutions = IK(x_fk, y_fk, z_fk, return_all=True)

    if solutions:
        print(f"\nIK Found {len(solutions)} solution(s):")
        for i, (t1_ik, t2_ik, t3_ik, error) in enumerate(solutions, 1):
            print(f"\n  Solution {i}:")
            print(f"    Angles: θ1={np.degrees(t1_ik):.4f}°, θ2={np.degrees(t2_ik):.4f}°, θ3={np.degrees(t3_ik):.4f}°")
            print(f"    Error: {error:.6f} mm")

            # Step 3: Verify by FK
            x_verify, y_verify, z_verify = FK(t1_ik, t2_ik, t3_ik)
            print(f"    FK Verify: x={x_verify:.4f}, y={y_verify:.4f}, z={z_verify:.4f}")

            # Full 3D error
            error_3d = np.sqrt((x_fk - x_verify)**2 + (y_fk - y_verify)**2 + (z_fk - z_verify)**2)
            print(f"    Full 3D Error: {error_3d:.6f} mm")

            # Check if this matches original angles
            angle_diff = abs(np.degrees(t3) - np.degrees(t3_ik))
            if angle_diff < 0.01:  # Same solution
                print(f"    [OK] MATCHES original angles!")
            else:
                print(f"    [  ] Different elbow configuration")
    else:
        print("IK FAILED - No solutions found!")

print("\n" + "="*70)
print("ANALYSIS:")
print("="*70)
print("If error > 0 for matching angles → BUG in IK calculation")
print("If error = 0 → IK is correct")
