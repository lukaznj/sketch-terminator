#!/usr/bin/env python3

import math
from sketch_terminator.kinematics import Kinematics

def test_kinematics_consistency():
    kin = Kinematics()
    
    # Define reference points in reach
    test_points = [
        [0.2, 0.0, 0.0],
        [0.25, 0.1, -0.02],
        [0.15, -0.15, 0.04],
        [0.3, 0.05, 0.01]
    ]
    
    print("Verifying kinematics consistency:")
    for pt in test_points:
        try:
            # 1. Solve IK to get joint angles
            joints = kin.get_ik(pt[0], pt[1], pt[2])
            
            # 2. Solve DK using the joint angles
            # Note: joints returned by IK are [beta_msg, gama_msg, alpha_msg]
            rec_pt = kin.get_dk(joints[0], joints[1], joints[2])
            
            # 3. Check differences
            diff_x = abs(rec_pt[0] - pt[0])
            diff_y = abs(rec_pt[1] - pt[1])
            diff_z = abs(rec_pt[2] - pt[2])
            
            print(f"Target: {pt}")
            print(f"Computed Joints: [beta={joints[0]:.4f}, gama={joints[1]:.4f}, alpha={joints[2]:.4f}]")
            print(f"Recovered: {[round(c, 4) for c in rec_pt]}")
            print(f"Errors: dx={diff_x:.6f}, dy={diff_y:.6f}, dz={diff_z:.6f}")
            
            assert diff_x < 1e-4, f"X diff {diff_x} exceeds threshold"
            assert diff_y < 1e-4, f"Y diff {diff_y} exceeds threshold"
            assert diff_z < 1e-4, f"Z diff {diff_z} exceeds threshold"
            print("✓ SUCCESS\n")
        except Exception as e:
            print(f"✗ FAILED for target {pt}: {e}\n")

if __name__ == '__main__':
    test_kinematics_consistency()
