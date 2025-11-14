#!/usr/bin/env python3
"""
Inverse Kinematics Solver for the single arm robot

Uses numerical optimization (scipy.optimize) to solve IK
"""

import numpy as np
from scipy.optimize import minimize, differential_evolution
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation


class InverseKinematicsSolver:
    """
    Solves inverse kinematics for the 4-DOF robot arm
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

        # Joint limits (from robot parameters)
        self.joint_limits = np.array([
            [self.joint0_lower, self.joint0_upper],
            [self.joint1_lower, self.joint1_upper],
            [self.joint2_lower, self.joint2_upper],
            [self.joint3_lower, self.joint3_upper]
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

        # Load joint limits
        self.joint0_lower = params.get('joint0_lower', -np.pi)
        self.joint0_upper = params.get('joint0_upper', np.pi)
        self.joint1_lower = params.get('joint1_lower', -np.pi/2)
        self.joint1_upper = params.get('joint1_upper', np.pi/2)
        self.joint2_lower = params.get('joint2_lower', -2.0)
        self.joint2_upper = params.get('joint2_upper', 2.0)
        self.joint3_lower = params.get('joint3_lower', -np.pi)
        self.joint3_upper = params.get('joint3_upper', np.pi)

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

    def forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics using URDF transformations

        Args:
            joint_angles: Array of 4 joint angles [q0, q1, q2, q3]

        Returns:
            4x4 transformation matrix from base_link to end_effector
        """
        # 1. base_link → neck_base (fixed)
        T = self.create_transform([0, 0, self.neck_height], [0, 0, 0])

        # 2. neck_base → link_1 (joint_0 with rotation)
        T_fixed_0 = self.create_transform(
            [self.link0_length, 0, 0],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_0 = self.create_rotation_z(joint_angles[0])
        T = T @ T_fixed_0 @ T_joint_0

        # 3. link_1 → link_2 (joint_1 with rotation)
        T_fixed_1 = self.create_transform(
            [self.link1_width, 0, self.link1_length],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_1 = self.create_rotation_z(joint_angles[1])
        T = T @ T_fixed_1 @ T_joint_1

        # 4. link_2 → link_3 (joint_2 with rotation)
        # URDF: origin xyz="0 ${link2_length} ${link2_width}" rpy="${-PI_2} ${-PI_2} 0"
        T_fixed_2 = self.create_transform(
            [0, self.link2_length, self.link2_width],
            [-self.PI/2, -self.PI/2, 0]
        )
        T_joint_2 = self.create_rotation_z(joint_angles[2])
        T = T @ T_fixed_2 @ T_joint_2

        # 5. link_3 → link_4 (joint_3 with rotation)
        # URDF: origin xyz="0 0 ${link3_length}" rpy="${PI_2} 0 ${PI_2}"
        T_fixed_3 = self.create_transform(
            [0, 0, self.link3_length],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_3 = self.create_rotation_z(joint_angles[3])
        T = T @ T_fixed_3 @ T_joint_3

        # 6. link_4 → end_effector (fixed)
        # URDF: origin xyz="0 ${link4_length + ee_offset} 0" rpy="${PI_2} 0 ${PI_2}"
        T_ee = self.create_transform(
            [0, self.link4_length + self.ee_offset, 0],
            [self.PI/2, 0, self.PI/2]
        )
        T = T @ T_ee

        return T

    def position_error(self, joint_angles, target_position):
        """
        Calculate position error between FK result and target

        Args:
            joint_angles: Array of 4 joint angles
            target_position: Target [x, y, z] position

        Returns:
            Position error (squared distance)
        """
        T = self.forward_kinematics(joint_angles)
        current_position = T[:3, 3]
        error = np.linalg.norm(current_position - target_position)
        return error

    def solve_ik(self, target_position, initial_guess=None, method='SLSQP'):
        """
        Solve inverse kinematics for a target position

        Args:
            target_position: Target [x, y, z] position
            initial_guess: Initial joint angles (if None, uses zeros)
            method: Optimization method ('SLSQP', 'L-BFGS-B', or 'differential_evolution')

        Returns:
            Dictionary with:
                - 'success': Boolean indicating if solution found
                - 'joint_angles': Solution joint angles
                - 'position_error': Final position error
                - 'message': Status message
        """
        if initial_guess is None:
            initial_guess = np.array([0.0, 0.0, 0.0, 0.0])

        # Bounds for joint angles
        bounds = [(self.joint_limits[i, 0], self.joint_limits[i, 1]) for i in range(4)]

        if method == 'differential_evolution':
            # Use differential evolution (global optimization)
            result = differential_evolution(
                self.position_error,
                bounds,
                args=(target_position,),
                maxiter=1000,
                tol=1e-6,
                seed=42
            )
        else:
            # Use local optimization
            result = minimize(
                self.position_error,
                initial_guess,
                args=(target_position,),
                method=method,
                bounds=bounds,
                options={'maxiter': 1000}
            )

        # Verify solution
        final_error = self.position_error(result.x, target_position)
        success = final_error < 0.01  # 1cm tolerance

        return {
            'success': success,
            'joint_angles': result.x,
            'position_error': final_error,
            'message': result.message if hasattr(result, 'message') else 'Optimization complete',
            'achieved_position': self.forward_kinematics(result.x)[:3, 3]
        }

    def solve_ik_multiple_attempts(self, target_position, num_attempts=10, use_global_if_needed=True):
        """
        Try to solve IK with multiple random initial guesses

        Args:
            target_position: Target [x, y, z] position
            num_attempts: Number of attempts with different initial guesses (default: 10)
            use_global_if_needed: If True, try global optimizer if local fails (default: True)

        Returns:
            Best solution found (same format as solve_ik)
        """
        best_solution = None
        best_error = float('inf')

        for i in range(num_attempts):
            if i == 0:
                # First attempt: start from zero
                initial_guess = np.zeros(4)
            else:
                # Random initial guess within joint limits
                initial_guess = np.array([
                    np.random.uniform(self.joint_limits[j, 0], self.joint_limits[j, 1])
                    for j in range(4)
                ])

            solution = self.solve_ik(target_position, initial_guess, method='SLSQP')

            if solution['position_error'] < best_error:
                best_error = solution['position_error']
                best_solution = solution

            # If we found a good solution, stop early
            if solution['success']:
                break

        # If local optimization failed and we're allowed, try global optimization
        if not best_solution['success'] and use_global_if_needed:
            # Try differential evolution (global optimizer)
            global_solution = self.solve_ik(target_position, None, method='differential_evolution')

            if global_solution['position_error'] < best_error:
                best_solution = global_solution

        return best_solution

    def solve_ik_for_waypoints(self, waypoints):
        """
        Solve IK for a list of waypoint positions

        Args:
            waypoints: List of [x, y, z] positions

        Returns:
            List of solutions (one per waypoint)
        """
        solutions = []
        previous_angles = None

        for i, waypoint in enumerate(waypoints):
            # Use previous solution as initial guess for continuity
            initial_guess = previous_angles if previous_angles is not None else None

            solution = self.solve_ik_multiple_attempts(waypoint, num_attempts=3)
            solutions.append(solution)

            if solution['success']:
                previous_angles = solution['joint_angles']

        return solutions


if __name__ == '__main__':
    # Test the IK solver
    solver = InverseKinematicsSolver()

    # Test position (should be reachable)
    target = np.array([0.5, 0.0, 0.7])

    print("Testing IK Solver")
    print("="*60)
    print(f"Target position: {target}")
    print()

    solution = solver.solve_ik_multiple_attempts(target)

    print(f"Success: {solution['success']}")
    print(f"Joint angles (deg): {np.rad2deg(solution['joint_angles'])}")
    print(f"Achieved position: {solution['achieved_position']}")
    print(f"Position error: {solution['position_error']:.6f} m")
    print()

    # Verify with forward kinematics
    T = solver.forward_kinematics(solution['joint_angles'])
    print(f"FK verification: {T[:3, 3]}")
