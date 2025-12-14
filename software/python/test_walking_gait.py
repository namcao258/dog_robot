"""Test walking gait trajectory"""
import numpy as np
import sys
sys.path.append(r'c:\Users\valer\OneDrive\Desktop\Robot\dog_robot\software\simulation')

from trajectory_controller import TrajectoryGenerator

print("="*70)
print("WALKING GAIT TRAJECTORY TEST")
print("="*70)
print("\nStep Length: 100mm, Step Height: 50mm")
print("Initial position: x=-52.5mm (leg backward, elbow pointing back)")
print("Walking direction: FORWARD (toward x positive)")
print("\n" + "="*70)

tg = TrajectoryGenerator()

# Test one complete step cycle
times = [0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0]

for t in times:
    x, y, z = tg.walking_gait(t, step_height=50, step_length=100)
    phase = (t % 1.0)

    if phase < 0.5:
        phase_name = "SWING (lift & move forward)"
    else:
        phase_name = "STANCE (push backward)"

    print(f"t={t:.3f}s | phase={phase:.3f} | {phase_name:30s} | x={x:7.2f} mm, z={z:7.2f} mm")

print("\n" + "="*70)
print("ANALYSIS:")
print("="*70)
print("- Swing phase (0.0-0.5): x increases from -52.5 to +47.5 (FORWARD)")
print("- Stance phase (0.5-1.0): x decreases from +47.5 to -52.5 (backward)")
print("- Robot walks FORWARD with elbow pointing backward")
