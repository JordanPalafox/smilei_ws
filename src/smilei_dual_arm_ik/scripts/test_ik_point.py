#!/usr/bin/env python3
"""
Test script to check if a specific point is reachable with IK

Usage:
  python3 test_ik_point.py 0.23 0.13 0.21
"""

import sys
import os
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'smilei_dual_arm_ik'))

from inverse_kinematics_solver import InverseKinematicsSolver

def test_point(x, y, z):
    """Test if a point is reachable"""

    # Load IK solver
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'robot_parameters.yaml')
    ik_solver = InverseKinematicsSolver(config_path)

    target = np.array([x, y, z])

    print("="*60)
    print(f"Testing point: [{x:.3f}, {y:.3f}, {z:.3f}]")
    print("="*60)

    # Print joint limits
    print("\nJoint Limits (radians):")
    for i in range(4):
        lower_deg = np.degrees(ik_solver.joint_limits[i, 0])
        upper_deg = np.degrees(ik_solver.joint_limits[i, 1])
        print(f"  joint_{i}: [{ik_solver.joint_limits[i, 0]:.3f}, {ik_solver.joint_limits[i, 1]:.3f}] "
              f"rad  ({lower_deg:.1f}°, {upper_deg:.1f}°)")

    # Try solving IK with improved method
    print("\n🔍 Solving IK with 10 attempts + global optimizer fallback...")
    solution = ik_solver.solve_ik_multiple_attempts(target, num_attempts=10, use_global_if_needed=True)

    print("\n" + "="*60)
    if solution['success']:
        print("✅ SUCCESS! Point is reachable!")
        print("="*60)
        print(f"\nJoint angles (radians):")
        for i, angle in enumerate(solution['joint_angles']):
            print(f"  joint_{i}: {angle:.4f} rad  ({np.degrees(angle):.2f}°)")

        print(f"\nPosition error: {solution['position_error']:.6f}m ({solution['position_error']*100:.2f}cm)")

        # Verify FK
        achieved = solution['achieved_position']
        print(f"\nTarget position:   [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]")
        print(f"Achieved position: [{achieved[0]:.4f}, {achieved[1]:.4f}, {achieved[2]:.4f}]")
        print(f"Difference:        [{achieved[0]-target[0]:+.4f}, {achieved[1]-target[1]:+.4f}, {achieved[2]-target[2]:+.4f}]")

    else:
        print("❌ FAILED - Point may be unreachable!")
        print("="*60)

        achieved = solution['achieved_position']
        print(f"\nTarget position:   [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]")
        print(f"Closest achieved:  [{achieved[0]:.4f}, {achieved[1]:.4f}, {achieved[2]:.4f}]")
        print(f"Difference:        [{achieved[0]-target[0]:+.4f}, {achieved[1]-target[1]:+.4f}, {achieved[2]-target[2]:+.4f}]")
        print(f"\nDistance: {solution['position_error']:.6f}m ({solution['position_error']*100:.2f}cm)")

        print("\n🔍 Possible reasons:")
        print("  1. Point is outside robot's workspace")
        print("  2. Point is too close to singularities")
        print("  3. Joint limits prevent reaching this point")
        print("  4. Optimization got stuck in local minimum (try running again)")

        print("\nBest joint angles found (radians):")
        for i, angle in enumerate(solution['joint_angles']):
            at_limit = ""
            if abs(angle - ik_solver.joint_limits[i, 0]) < 0.01:
                at_limit = " ⚠️ AT LOWER LIMIT"
            elif abs(angle - ik_solver.joint_limits[i, 1]) < 0.01:
                at_limit = " ⚠️ AT UPPER LIMIT"
            print(f"  joint_{i}: {angle:.4f} rad  ({np.degrees(angle):.2f}°){at_limit}")

    print("="*60)

    return solution['success']

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: python3 test_ik_point.py <x> <y> <z>")
        print("\nExample:")
        print("  python3 test_ik_point.py 0.23 0.13 0.21")
        print("\nTip: Use the interactive marker (press 'P') to get coordinates")
        sys.exit(1)

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        success = test_point(x, y, z)
        sys.exit(0 if success else 1)

    except ValueError as e:
        print(f"Error: Invalid coordinates. Please provide numbers.")
        print(f"  {e}")
        sys.exit(1)
