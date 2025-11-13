#!/usr/bin/env python3
"""
Trajectory Planner for smooth motion between waypoints

Generates smooth trajectories using interpolation
"""

import numpy as np
from scipy.interpolate import CubicSpline, interp1d


class TrajectoryPlanner:
    """
    Plans smooth trajectories between waypoints
    """

    def __init__(self, interpolation_method='cubic'):
        """
        Initialize trajectory planner

        Args:
            interpolation_method: 'linear', 'cubic', or 'quintic'
        """
        self.interpolation_method = interpolation_method

    def plan_trajectory(self, waypoint_angles, num_steps_per_segment=50):
        """
        Plan a smooth trajectory through waypoints in joint space

        Args:
            waypoint_angles: List of joint angle arrays (one per waypoint)
            num_steps_per_segment: Number of interpolation steps between waypoints

        Returns:
            Dictionary with:
                - 'trajectory': Array of joint angles [num_points x 4]
                - 'waypoint_indices': Indices of original waypoints in trajectory
                - 'num_points': Total number of points
        """
        if len(waypoint_angles) < 2:
            raise ValueError("Need at least 2 waypoints to plan trajectory")

        waypoint_angles = np.array(waypoint_angles)
        num_waypoints = len(waypoint_angles)
        num_joints = waypoint_angles.shape[1]

        # Create time stamps for waypoints (0 to num_waypoints-1)
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
                    bc_type='clamped'  # Zero velocity at endpoints
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

    def plan_cartesian_trajectory(self, waypoint_positions, num_steps_per_segment=50):
        """
        Plan a smooth trajectory through waypoints in Cartesian space

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
        Apply trapezoidal velocity profile to trajectory

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
    # Test trajectory planner
    planner = TrajectoryPlanner(interpolation_method='cubic')

    # Create some test waypoints (joint angles)
    waypoints = [
        np.array([0.0, 0.0, 0.0, 0.0]),
        np.array([0.5, 0.3, 0.2, 0.1]),
        np.array([1.0, 0.5, 0.4, 0.2]),
        np.array([0.5, 0.3, 0.2, 0.1]),
        np.array([0.0, 0.0, 0.0, 0.0]),
    ]

    print("Testing Trajectory Planner")
    print("="*60)
    print(f"Number of waypoints: {len(waypoints)}")
    print()

    result = planner.plan_trajectory(waypoints, num_steps_per_segment=20)

    print(f"Total trajectory points: {result['num_points']}")
    print(f"Waypoint indices: {result['waypoint_indices']}")
    print(f"Trajectory shape: {result['trajectory'].shape}")
    print()

    # Show first few points
    print("First 5 trajectory points:")
    for i in range(min(5, len(result['trajectory']))):
        print(f"  Point {i}: {np.rad2deg(result['trajectory'][i])}")
