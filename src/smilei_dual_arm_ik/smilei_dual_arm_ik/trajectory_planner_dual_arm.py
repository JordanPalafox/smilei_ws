#!/usr/bin/env python3
"""
Trajectory Planner for Dual Arm Robot

Plans smooth trajectories for both arms independently or simultaneously
"""

import numpy as np
from scipy.interpolate import CubicSpline, interp1d


class TrajectoryPlannerDualArm:
    """
    Plans smooth trajectories for dual arm robot
    Can plan for right arm, left arm, or both arms simultaneously
    """

    def __init__(self, interpolation_method='cubic'):
        """
        Initialize trajectory planner

        Args:
            interpolation_method: 'linear' or 'cubic'
        """
        self.interpolation_method = interpolation_method

    def plan_single_arm_trajectory(self, waypoint_angles, num_steps_per_segment=50):
        """
        Plan trajectory for a single arm (4 joints)

        Args:
            waypoint_angles: List of joint angle arrays (one per waypoint) [num_waypoints x 4]
            num_steps_per_segment: Number of interpolation steps between waypoints

        Returns:
            Dictionary with trajectory information
        """
        if len(waypoint_angles) < 2:
            raise ValueError("Need at least 2 waypoints to plan trajectory")

        waypoint_angles = np.array(waypoint_angles)
        num_waypoints = len(waypoint_angles)
        num_joints = waypoint_angles.shape[1]

        # Create time stamps for waypoints
        waypoint_times = np.arange(num_waypoints, dtype=float)

        # Total number of interpolated points
        total_steps = (num_waypoints - 1) * num_steps_per_segment + 1

        # Interpolation times
        interp_times = np.linspace(0, num_waypoints - 1, total_steps)

        # Interpolate each joint independently
        trajectory = np.zeros((total_steps, num_joints))

        for joint_idx in range(num_joints):
            joint_waypoints = waypoint_angles[:, joint_idx]

            if self.interpolation_method == 'linear':
                interp_func = interp1d(
                    waypoint_times,
                    joint_waypoints,
                    kind='linear'
                )
            elif self.interpolation_method == 'cubic':
                interp_func = CubicSpline(
                    waypoint_times,
                    joint_waypoints,
                    bc_type='clamped'
                )
            else:
                raise ValueError(f"Unknown interpolation method: {self.interpolation_method}")

            trajectory[:, joint_idx] = interp_func(interp_times)

        # Find indices of original waypoints in the trajectory
        waypoint_indices = [i * num_steps_per_segment for i in range(num_waypoints)]

        return {
            'trajectory': trajectory,
            'waypoint_indices': waypoint_indices,
            'num_points': total_steps,
            'num_waypoints': num_waypoints
        }

    def plan_dual_arm_trajectory(self, right_waypoint_angles, left_waypoint_angles,
                                  num_steps_per_segment=50, synchronized=True):
        """
        Plan trajectories for both arms

        Args:
            right_waypoint_angles: List of joint angles for right arm [num_waypoints x 4]
            left_waypoint_angles: List of joint angles for left arm [num_waypoints x 4]
            num_steps_per_segment: Number of interpolation steps between waypoints
            synchronized: If True, both arms move through waypoints at the same time

        Returns:
            Dictionary with trajectories for both arms
        """
        if synchronized:
            # Verify both arms have same number of waypoints
            if len(right_waypoint_angles) != len(left_waypoint_angles):
                raise ValueError("For synchronized motion, both arms must have same number of waypoints")

        # Plan trajectory for right arm
        right_result = self.plan_single_arm_trajectory(
            right_waypoint_angles,
            num_steps_per_segment
        )

        # Plan trajectory for left arm
        left_result = self.plan_single_arm_trajectory(
            left_waypoint_angles,
            num_steps_per_segment
        )

        if synchronized:
            # Ensure both trajectories have same length
            min_length = min(right_result['num_points'], left_result['num_points'])
            right_result['trajectory'] = right_result['trajectory'][:min_length]
            left_result['trajectory'] = left_result['trajectory'][:min_length]

        return {
            'right_arm': right_result,
            'left_arm': left_result,
            'synchronized': synchronized,
            'total_points': right_result['num_points'] if synchronized else None
        }

    def plan_cartesian_trajectory(self, waypoint_positions, num_steps_per_segment=50):
        """
        Plan smooth trajectory through waypoints in Cartesian space

        Args:
            waypoint_positions: List of [x, y, z] positions
            num_steps_per_segment: Number of interpolation steps between waypoints

        Returns:
            Array of interpolated positions [num_points x 3]
        """
        if len(waypoint_positions) < 2:
            raise ValueError("Need at least 2 waypoints")

        waypoint_positions = np.array(waypoint_positions)
        num_waypoints = len(waypoint_positions)

        # Create time stamps
        waypoint_times = np.arange(num_waypoints, dtype=float)
        total_steps = (num_waypoints - 1) * num_steps_per_segment + 1
        interp_times = np.linspace(0, num_waypoints - 1, total_steps)

        # Interpolate each dimension
        interpolated_positions = np.zeros((total_steps, 3))

        for dim in range(3):
            dim_waypoints = waypoint_positions[:, dim]

            if self.interpolation_method == 'linear':
                interp_func = interp1d(waypoint_times, dim_waypoints, kind='linear')
            elif self.interpolation_method == 'cubic':
                interp_func = CubicSpline(waypoint_times, dim_waypoints, bc_type='clamped')
            else:
                raise ValueError(f"Unknown interpolation method: {self.interpolation_method}")

            interpolated_positions[:, dim] = interp_func(interp_times)

        return interpolated_positions

    def calculate_trajectory_duration(self, trajectory, max_velocity=1.0):
        """
        Calculate time duration for trajectory based on velocity limits

        Args:
            trajectory: Array of joint angles [num_points x num_joints]
            max_velocity: Maximum joint velocity (rad/s)

        Returns:
            Array of time stamps for each point
        """
        num_points = len(trajectory)

        if num_points < 2:
            return np.array([0.0])

        # Calculate angular differences between consecutive points
        diffs = np.diff(trajectory, axis=0)
        max_diffs = np.max(np.abs(diffs), axis=1)

        # Time for each segment based on max velocity
        segment_times = max_diffs / max_velocity

        # Cumulative time
        times = np.zeros(num_points)
        times[1:] = np.cumsum(segment_times)

        return times

    def add_velocity_profile(self, trajectory, max_velocity=1.0, max_acceleration=2.0):
        """
        Apply velocity profile to trajectory

        Args:
            trajectory: Array of joint angles [num_points x num_joints]
            max_velocity: Maximum joint velocity (rad/s)
            max_acceleration: Maximum joint acceleration (rad/s²)

        Returns:
            Dictionary with trajectory and timing information
        """
        times = self.calculate_trajectory_duration(trajectory, max_velocity)

        return {
            'trajectory': trajectory,
            'times': times,
            'duration': times[-1],
            'num_points': len(trajectory)
        }


if __name__ == '__main__':
    # Test dual arm trajectory planner
    planner = TrajectoryPlannerDualArm(interpolation_method='cubic')

    print("Testing Dual Arm Trajectory Planner")
    print("="*60)

    # Create test waypoints for right arm
    right_waypoints = [
        np.array([0.0, 0.0, 0.0, 0.0]),
        np.array([0.5, 0.3, 0.2, 0.1]),
        np.array([1.0, 0.5, 0.4, 0.2]),
        np.array([0.0, 0.0, 0.0, 0.0]),
    ]

    # Create test waypoints for left arm (mirrored)
    left_waypoints = [
        np.array([0.0, 0.0, 0.0, 0.0]),
        np.array([-0.5, 0.3, 0.2, 0.1]),
        np.array([-1.0, 0.5, 0.4, 0.2]),
        np.array([0.0, 0.0, 0.0, 0.0]),
    ]

    print(f"Right arm waypoints: {len(right_waypoints)}")
    print(f"Left arm waypoints: {len(left_waypoints)}")
    print()

    # Test synchronized dual arm planning
    print("Planning synchronized dual arm trajectory...")
    result = planner.plan_dual_arm_trajectory(
        right_waypoints,
        left_waypoints,
        num_steps_per_segment=20,
        synchronized=True
    )

    print(f"Synchronized: {result['synchronized']}")
    print(f"Total points: {result['total_points']}")
    print(f"Right arm trajectory shape: {result['right_arm']['trajectory'].shape}")
    print(f"Left arm trajectory shape: {result['left_arm']['trajectory'].shape}")
    print()

    # Show first few points
    print("First 3 trajectory points:")
    print("Right arm:")
    for i in range(min(3, len(result['right_arm']['trajectory']))):
        print(f"  Point {i}: {np.rad2deg(result['right_arm']['trajectory'][i])}")
    print("Left arm:")
    for i in range(min(3, len(result['left_arm']['trajectory']))):
        print(f"  Point {i}: {np.rad2deg(result['left_arm']['trajectory'][i])}")
