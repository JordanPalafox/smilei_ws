#!/usr/bin/env python3
"""
Inverse Kinematics Solver for Dual Arm Robot

Solves IK for both left and right arms independently
"""

import numpy as np
from scipy.optimize import minimize, differential_evolution
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation


class InverseKinematicsDualArm:
    """
    Solves inverse kinematics for dual 4-DOF robot arms
    Each arm has 4 joints and can be controlled independently
    """

    def __init__(self, robot_params_file=None):
        """
        Initialize IK solver with robot parameters

        Args:
            robot_params_file: Path to YAML file with robot parameters
        """
        # Load robot parameters
        if robot_params_file is None:
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')

        self.load_robot_parameters(robot_params_file)
        self.PI = np.pi

        # Joint limits - Right Arm
        self.right_joint_limits = np.array([
            [self.right_joint0_lower, self.right_joint0_upper],
            [self.right_joint1_lower, self.right_joint1_upper],
            [self.right_joint2_lower, self.right_joint2_upper],
            [self.right_joint3_lower, self.right_joint3_upper]
        ])

        # Joint limits - Left Arm
        self.left_joint_limits = np.array([
            [self.left_joint0_lower, self.left_joint0_upper],
            [self.left_joint1_lower, self.left_joint1_upper],
            [self.left_joint2_lower, self.left_joint2_upper],
            [self.left_joint3_lower, self.left_joint3_upper]
        ])

    def load_robot_parameters(self, yaml_file):
        """Load robot geometric parameters from YAML file"""
        with open(yaml_file, 'r') as f:
            config = yaml.safe_load(f)

        params = config.get('robot_geometry', {})

        # Load all geometric parameters
        self.neck_height = params.get('neck_height', 0.34)
        self.base_height = params.get('base_height', 0.34)
        self.link0_length = params.get('link0_length', 0.10)
        self.link0_width = params.get('link0_width', 0.0)
        self.link1_length = params.get('link1_length', 0.15)
        self.link1_width = params.get('link1_width', -0.30)
        self.link2_length = params.get('link2_length', 0.25)
        self.link2_width = params.get('link2_width', 0.20)
        self.link3_length = params.get('link3_length', -0.20)
        self.link4_length = params.get('link4_length', -0.08)
        self.ee_offset = params.get('ee_offset', 0.0)

        # Load joint limits - Right Arm
        self.right_joint0_lower = params.get('right_joint0_lower', -np.pi)
        self.right_joint0_upper = params.get('right_joint0_upper', np.pi)
        self.right_joint1_lower = params.get('right_joint1_lower', -np.pi/2)
        self.right_joint1_upper = params.get('right_joint1_upper', np.pi/2)
        self.right_joint2_lower = params.get('right_joint2_lower', -2.0)
        self.right_joint2_upper = params.get('right_joint2_upper', 2.0)
        self.right_joint3_lower = params.get('right_joint3_lower', -np.pi)
        self.right_joint3_upper = params.get('right_joint3_upper', np.pi)

        # Load joint limits - Left Arm
        self.left_joint0_lower = params.get('left_joint0_lower', -np.pi)
        self.left_joint0_upper = params.get('left_joint0_upper', np.pi)
        self.left_joint1_lower = params.get('left_joint1_lower', -np.pi/2)
        self.left_joint1_upper = params.get('left_joint1_upper', np.pi/2)
        self.left_joint2_lower = params.get('left_joint2_lower', -2.0)
        self.left_joint2_upper = params.get('left_joint2_upper', 2.0)
        self.left_joint3_lower = params.get('left_joint3_lower', -np.pi)
        self.left_joint3_upper = params.get('left_joint3_upper', np.pi)

    def create_transform(self, xyz, rpy):
        """Create 4x4 homogeneous transformation matrix"""
        T = np.eye(4)
        T[:3, 3] = xyz
        r = Rotation.from_euler('xyz', rpy)
        T[:3, :3] = r.as_matrix()
        return T

    def create_rotation_z(self, angle):
        """Create rotation matrix around Z axis"""
        c = np.cos(angle)
        s = np.sin(angle)
        R = np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ])
        T = np.eye(4)
        T[:3, :3] = R
        return T

    def forward_kinematics_right_arm(self, joint_angles):
        """
        Calculate forward kinematics for RIGHT arm

        Args:
            joint_angles: Array of 4 joint angles

        Returns:
            4x4 transformation matrix from base_link to right_end_effector
        """
        # 1. base_link → neck_base (fixed)
        T = self.create_transform([0, 0, self.neck_height], [0, 0, 0])

        # 2. neck_base → right_link_1 (right_joint_0)
        T_fixed_0 = self.create_transform(
            [self.link0_length, 0, 0],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_0 = self.create_rotation_z(joint_angles[0])
        T = T @ T_fixed_0 @ T_joint_0

        # 3. right_link_1 → right_link_2 (right_joint_1)
        T_fixed_1 = self.create_transform(
            [self.link1_width, 0, self.link1_length],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_1 = self.create_rotation_z(joint_angles[1])
        T = T @ T_fixed_1 @ T_joint_1

        # 4. right_link_2 → right_link_3 (right_joint_2)
        # URDF: origin xyz="0 ${link2_length} ${link2_width}" rpy="${-PI_2} ${-PI_2} 0"
        T_fixed_2 = self.create_transform(
            [0, self.link2_length, self.link2_width],
            [-self.PI/2, -self.PI/2, 0]
        )
        T_joint_2 = self.create_rotation_z(joint_angles[2])
        T = T @ T_fixed_2 @ T_joint_2

        # 5. right_link_3 → right_link_4 (right_joint_3)
        # URDF: origin xyz="0 0 ${link3_length}" rpy="${PI_2} 0 ${PI_2}"
        T_fixed_3 = self.create_transform(
            [0, 0, self.link3_length],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_3 = self.create_rotation_z(joint_angles[3])
        T = T @ T_fixed_3 @ T_joint_3

        # 6. right_link_4 → right_end_effector (fixed)
        # URDF: origin xyz="0 ${link4_length + ee_offset} 0" rpy="${PI_2} 0 ${PI_2}"
        T_ee = self.create_transform(
            [0, self.link4_length + self.ee_offset, 0],
            [self.PI/2, 0, self.PI/2]
        )
        T = T @ T_ee

        return T

    def forward_kinematics_left_arm(self, joint_angles):
        """
        Calculate forward kinematics for LEFT arm (mirrored)

        Args:
            joint_angles: Array of 4 joint angles

        Returns:
            4x4 transformation matrix from base_link to left_end_effector
        """
        # 1. base_link → neck_base (fixed)
        T = self.create_transform([0, 0, self.neck_height], [0, 0, 0])

        # 2. neck_base → left_link_1 (left_joint_0)
        T_fixed_0 = self.create_transform(
            [-self.link0_length, 0, 0],
            [self.PI/2, -self.PI/2, -self.PI/2]
        )
        T_joint_0 = self.create_rotation_z(joint_angles[0])
        T = T @ T_fixed_0 @ T_joint_0

        # 3. left_link_1 → left_link_2 (left_joint_1)
        T_fixed_1 = self.create_transform(
            [0, self.link1_width, self.link1_length],
            [-self.PI/2, -self.PI/2, 0]
        )
        T_joint_1 = self.create_rotation_z(joint_angles[1])
        T = T @ T_fixed_1 @ T_joint_1

        # 4. left_link_2 → left_link_3 (left_joint_2)
        # URDF: origin xyz="${link2_length} 0 ${link2_width}" rpy="${PI_2} 0 ${PI_2}"
        T_fixed_2 = self.create_transform(
            [self.link2_length, 0, self.link2_width],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_2 = self.create_rotation_z(joint_angles[2])
        T = T @ T_fixed_2 @ T_joint_2

        # 5. left_link_3 → left_link_4 (left_joint_3)
        # URDF: origin xyz="0 0 ${link3_length}" rpy="${-PI_2} ${-PI_2} 0"
        T_fixed_3 = self.create_transform(
            [0, 0, self.link3_length],
            [-self.PI/2, -self.PI/2, 0]
        )
        T_joint_3 = self.create_rotation_z(joint_angles[3])
        T = T @ T_fixed_3 @ T_joint_3

        # 6. left_link_4 → left_end_effector (fixed)
        # URDF: origin xyz="${link4_length + ee_offset} 0 0" rpy="0 ${-PI_2} ${-PI_2}"
        T_ee = self.create_transform(
            [self.link4_length + self.ee_offset, 0, 0],
            [0, -self.PI/2, -self.PI/2]
        )
        T = T @ T_ee

        return T

    def position_error_right_arm(self, joint_angles, target_position):
        """
        Calculate position error for right arm

        Args:
            joint_angles: Array of 4 joint angles
            target_position: Target [x, y, z] position

        Returns:
            Position error (distance)
        """
        T = self.forward_kinematics_right_arm(joint_angles)
        current_position = T[:3, 3]
        error = np.linalg.norm(current_position - target_position)
        return error

    def position_error_left_arm(self, joint_angles, target_position):
        """
        Calculate position error for left arm

        Args:
            joint_angles: Array of 4 joint angles
            target_position: Target [x, y, z] position

        Returns:
            Position error (distance)
        """
        T = self.forward_kinematics_left_arm(joint_angles)
        current_position = T[:3, 3]
        error = np.linalg.norm(current_position - target_position)
        return error

    def solve_ik_right_arm(self, target_position, initial_guess=None, method='SLSQP'):
        """
        Solve inverse kinematics for right arm

        Args:
            target_position: Target [x, y, z] position
            initial_guess: Initial joint angles (if None, uses zeros)
            method: Optimization method ('SLSQP', 'L-BFGS-B', or 'differential_evolution')

        Returns:
            Dictionary with solution details
        """
        if initial_guess is None:
            initial_guess = np.array([0.0, 0.0, 0.0, 0.0])

        # Bounds for joint angles - Right Arm
        bounds = [(self.right_joint_limits[i, 0], self.right_joint_limits[i, 1]) for i in range(4)]

        if method == 'differential_evolution':
            result = differential_evolution(
                self.position_error_right_arm,
                bounds,
                args=(target_position,),
                maxiter=1000,
                tol=1e-6,
                seed=42
            )
        else:
            result = minimize(
                self.position_error_right_arm,
                initial_guess,
                args=(target_position,),
                method=method,
                bounds=bounds,
                options={'maxiter': 1000}
            )

        # Verify solution
        final_error = self.position_error_right_arm(result.x, target_position)
        success = final_error < 0.01  # 1cm tolerance

        return {
            'success': success,
            'joint_angles': result.x,
            'position_error': final_error,
            'message': result.message if hasattr(result, 'message') else 'Optimization complete',
            'achieved_position': self.forward_kinematics_right_arm(result.x)[:3, 3]
        }

    def solve_ik_left_arm(self, target_position, initial_guess=None, method='SLSQP'):
        """
        Solve inverse kinematics for left arm

        Args:
            target_position: Target [x, y, z] position
            initial_guess: Initial joint angles (if None, uses zeros)
            method: Optimization method ('SLSQP', 'L-BFGS-B', or 'differential_evolution')

        Returns:
            Dictionary with solution details
        """
        if initial_guess is None:
            initial_guess = np.array([0.0, 0.0, 0.0, 0.0])

        # Bounds for joint angles - Left Arm
        bounds = [(self.left_joint_limits[i, 0], self.left_joint_limits[i, 1]) for i in range(4)]

        if method == 'differential_evolution':
            result = differential_evolution(
                self.position_error_left_arm,
                bounds,
                args=(target_position,),
                maxiter=1000,
                tol=1e-6,
                seed=42
            )
        else:
            result = minimize(
                self.position_error_left_arm,
                initial_guess,
                args=(target_position,),
                method=method,
                bounds=bounds,
                options={'maxiter': 1000}
            )

        # Verify solution
        final_error = self.position_error_left_arm(result.x, target_position)
        success = final_error < 0.01  # 1cm tolerance

        return {
            'success': success,
            'joint_angles': result.x,
            'position_error': final_error,
            'message': result.message if hasattr(result, 'message') else 'Optimization complete',
            'achieved_position': self.forward_kinematics_left_arm(result.x)[:3, 3]
        }

    def solve_ik_right_arm_multiple_attempts(self, target_position, num_attempts=10, use_global_if_needed=True):
        """
        Solve IK for right arm with multiple random initial guesses

        Args:
            target_position: Target [x, y, z] position
            num_attempts: Number of attempts with different initial guesses
            use_global_if_needed: If True, try global optimizer if local fails

        Returns:
            Best solution found
        """
        best_solution = None
        best_error = float('inf')

        for i in range(num_attempts):
            if i == 0:
                initial_guess = np.zeros(4)
            else:
                initial_guess = np.array([
                    np.random.uniform(self.right_joint_limits[j, 0], self.right_joint_limits[j, 1])
                    for j in range(4)
                ])

            solution = self.solve_ik_right_arm(target_position, initial_guess, method='SLSQP')

            if solution['position_error'] < best_error:
                best_error = solution['position_error']
                best_solution = solution

            if solution['success']:
                break

        # Try global optimization if needed
        if not best_solution['success'] and use_global_if_needed:
            global_solution = self.solve_ik_right_arm(target_position, None, method='differential_evolution')
            if global_solution['position_error'] < best_error:
                best_solution = global_solution

        return best_solution

    def solve_ik_left_arm_multiple_attempts(self, target_position, num_attempts=10, use_global_if_needed=True):
        """
        Solve IK for left arm with multiple random initial guesses

        Args:
            target_position: Target [x, y, z] position
            num_attempts: Number of attempts with different initial guesses
            use_global_if_needed: If True, try global optimizer if local fails

        Returns:
            Best solution found
        """
        best_solution = None
        best_error = float('inf')

        for i in range(num_attempts):
            if i == 0:
                initial_guess = np.zeros(4)
            else:
                initial_guess = np.array([
                    np.random.uniform(self.left_joint_limits[j, 0], self.left_joint_limits[j, 1])
                    for j in range(4)
                ])

            solution = self.solve_ik_left_arm(target_position, initial_guess, method='SLSQP')

            if solution['position_error'] < best_error:
                best_error = solution['position_error']
                best_solution = solution

            if solution['success']:
                break

        # Try global optimization if needed
        if not best_solution['success'] and use_global_if_needed:
            global_solution = self.solve_ik_left_arm(target_position, None, method='differential_evolution')
            if global_solution['position_error'] < best_error:
                best_solution = global_solution

        return best_solution

    def solve_ik_both_arms(self, right_target, left_target, num_attempts=10, use_global_if_needed=True):
        """
        Solve IK for both arms simultaneously

        Args:
            right_target: Target [x, y, z] position for right arm
            left_target: Target [x, y, z] position for left arm
            num_attempts: Number of attempts per arm
            use_global_if_needed: If True, try global optimizer if local fails

        Returns:
            Dictionary with solutions for both arms
        """
        right_solution = self.solve_ik_right_arm_multiple_attempts(
            right_target, num_attempts, use_global_if_needed
        )
        left_solution = self.solve_ik_left_arm_multiple_attempts(
            left_target, num_attempts, use_global_if_needed
        )

        return {
            'right_arm': right_solution,
            'left_arm': left_solution,
            'both_successful': right_solution['success'] and left_solution['success']
        }


if __name__ == '__main__':
    # Test the dual arm IK solver
    solver = InverseKinematicsDualArm()

    print("Testing Dual Arm IK Solver")
    print("="*60)

    # Test right arm
    right_target = np.array([0.3, -0.2, 0.4])
    print(f"Right Arm Target: {right_target}")
    right_solution = solver.solve_ik_right_arm_multiple_attempts(right_target)
    print(f"  Success: {right_solution['success']}")
    print(f"  Joint angles (deg): {np.rad2deg(right_solution['joint_angles'])}")
    print(f"  Achieved position: {right_solution['achieved_position']}")
    print(f"  Position error: {right_solution['position_error']:.6f} m")
    print()

    # Test left arm
    left_target = np.array([-0.3, -0.2, 0.4])
    print(f"Left Arm Target: {left_target}")
    left_solution = solver.solve_ik_left_arm_multiple_attempts(left_target)
    print(f"  Success: {left_solution['success']}")
    print(f"  Joint angles (deg): {np.rad2deg(left_solution['joint_angles'])}")
    print(f"  Achieved position: {left_solution['achieved_position']}")
    print(f"  Position error: {left_solution['position_error']:.6f} m")
    print()

    # Test both arms simultaneously
    print("Testing Both Arms Simultaneously")
    print("-"*60)
    both_solution = solver.solve_ik_both_arms(right_target, left_target)
    print(f"Both arms successful: {both_solution['both_successful']}")
    print()
