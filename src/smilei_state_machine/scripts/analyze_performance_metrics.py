#!/usr/bin/env python3
"""
Performance Metrics Analyzer

Analyzes gesture execution performance metrics and generates comparison reports
between Cartesian and Joint control modes.
"""

import yaml
import os
import sys
from pathlib import Path
from collections import defaultdict
import numpy as np


class PerformanceAnalyzer:
    def __init__(self, metrics_directory):
        self.metrics_dir = Path(metrics_directory)
        self.metrics = []
        self.load_all_metrics()

    def load_all_metrics(self):
        """Load all metric files from directory"""
        if not self.metrics_dir.exists():
            print(f"❌ Metrics directory not found: {self.metrics_dir}")
            return

        yaml_files = list(self.metrics_dir.glob("*.yaml"))

        # Filter out summary and latest files
        metric_files = [f for f in yaml_files
                       if 'summary' not in f.name and 'latest' not in f.name]

        for filepath in metric_files:
            try:
                with open(filepath, 'r') as f:
                    data = yaml.safe_load(f)
                    if data:
                        self.metrics.append(data)
            except Exception as e:
                print(f"⚠️  Failed to load {filepath.name}: {e}")

        print(f"✅ Loaded {len(self.metrics)} metric files")

    def group_by_mode(self):
        """Group metrics by control mode"""
        cartesian = [m for m in self.metrics if m.get('control_mode') == 'cartesian']
        joint = [m for m in self.metrics if m.get('control_mode') == 'joint']
        return cartesian, joint

    def group_by_gesture(self):
        """Group metrics by gesture name"""
        by_gesture = defaultdict(lambda: {'cartesian': [], 'joint': []})

        for metric in self.metrics:
            gesture = metric.get('gesture_name')
            mode = metric.get('control_mode')
            if gesture and mode:
                by_gesture[gesture][mode].append(metric)

        return dict(by_gesture)

    def compute_statistics(self, metrics_list, field_path):
        """Compute mean, std, min, max for a nested field

        field_path: list like ['timing_ms', 'ik_solve_time']
        """
        values = []
        for metric in metrics_list:
            val = metric
            for key in field_path:
                val = val.get(key) if isinstance(val, dict) else None
                if val is None:
                    break
            if val is not None and val > 0:
                values.append(val)

        if not values:
            return None

        return {
            'mean': np.mean(values),
            'std': np.std(values),
            'min': np.min(values),
            'max': np.max(values),
            'count': len(values)
        }

    def print_comparison_report(self):
        """Print comprehensive comparison report"""
        cartesian, joint = self.group_by_mode()

        print("\n" + "="*80)
        print("PERFORMANCE COMPARISON REPORT: CARTESIAN vs JOINT")
        print("="*80)
        print(f"\nTotal executions: {len(self.metrics)}")
        print(f"  - Cartesian mode: {len(cartesian)}")
        print(f"  - Joint mode: {len(joint)}")

        # Timing comparison
        print("\n" + "-"*80)
        print("INITIALIZATION TIMING COMPARISON (ms)")
        print("-"*80)

        timing_fields = [
            ('IK Solve Time', ['timing_ms', 'ik_solve_time']),
            ('Trajectory Planning', ['timing_ms', 'trajectory_planning_time']),
            ('Total Initialization', ['timing_ms', 'total_initialization_time']),
            ('Cache Load Time', ['timing_ms', 'cache_load_time']),
        ]

        print(f"\n{'Metric':<30} {'Cartesian (ms)':<25} {'Joint (ms)':<25}")
        print("-"*80)

        for metric_name, field_path in timing_fields:
            cart_stats = self.compute_statistics(cartesian, field_path)
            joint_stats = self.compute_statistics(joint, field_path)

            cart_str = f"{cart_stats['mean']:.2f} ± {cart_stats['std']:.2f}" if cart_stats else "N/A"
            joint_str = f"{joint_stats['mean']:.2f} ± {joint_stats['std']:.2f}" if joint_stats else "N/A"

            print(f"{metric_name:<30} {cart_str:<25} {joint_str:<25}")

        # Speedup calculation
        cart_init = self.compute_statistics(cartesian, ['timing_ms', 'total_initialization_time'])
        joint_init = self.compute_statistics(joint, ['timing_ms', 'total_initialization_time'])

        if cart_init and joint_init:
            speedup = cart_init['mean'] / joint_init['mean']
            print(f"\n{'Speedup Factor (Joint/Cartesian):':<30} {speedup:.2f}x")

        # Cartesian-specific metrics
        if cartesian:
            print("\n" + "-"*80)
            print("CARTESIAN MODE SPECIFIC METRICS")
            print("-"*80)

            ik_success = self.compute_statistics(cartesian, ['cartesian_metrics', 'ik_success_count'])
            ik_failed = self.compute_statistics(cartesian, ['cartesian_metrics', 'ik_failed_count'])

            if ik_success and ik_failed:
                total = ik_success['mean'] + ik_failed['mean']
                success_rate = (ik_success['mean'] / total * 100) if total > 0 else 0
                print(f"IK Success Rate: {success_rate:.1f}%")
                print(f"  - Average successes per gesture: {ik_success['mean']:.1f}")
                print(f"  - Average failures per gesture: {ik_failed['mean']:.1f}")

        # Joint-specific metrics
        if joint:
            print("\n" + "-"*80)
            print("JOINT MODE SPECIFIC METRICS")
            print("-"*80)

            joint_fields = [
                ('Max Joint Velocity (rad/s)', ['joint_metrics', 'max_joint_velocity_rad_s']),
                ('Max Joint Acceleration (rad/s²)', ['joint_metrics', 'max_joint_acceleration_rad_s2']),
                ('Velocity Discontinuities', ['joint_metrics', 'velocity_discontinuities']),
            ]

            for metric_name, field_path in joint_fields:
                stats = self.compute_statistics(joint, field_path)
                if stats:
                    print(f"{metric_name:<35} {stats['mean']:.3f} ± {stats['std']:.3f}")

        # Per-gesture breakdown
        print("\n" + "-"*80)
        print("PER-GESTURE BREAKDOWN")
        print("-"*80)

        by_gesture = self.group_by_gesture()

        for gesture_name, modes in sorted(by_gesture.items()):
            print(f"\nGesture: {gesture_name}")
            print(f"  Cartesian executions: {len(modes['cartesian'])}")
            print(f"  Joint executions: {len(modes['joint'])}")

            if modes['cartesian']:
                cart_init = self.compute_statistics(modes['cartesian'],
                                                   ['timing_ms', 'total_initialization_time'])
                if cart_init:
                    print(f"  Cartesian init time: {cart_init['mean']:.1f} ± {cart_init['std']:.1f} ms")

            if modes['joint']:
                joint_init = self.compute_statistics(modes['joint'],
                                                    ['timing_ms', 'total_initialization_time'])
                if joint_init:
                    print(f"  Joint init time: {joint_init['mean']:.1f} ± {joint_init['std']:.1f} ms")

        print("\n" + "="*80)

    def generate_latex_table(self):
        """Generate LaTeX table for paper"""
        cartesian, joint = self.group_by_mode()

        print("\n" + "="*80)
        print("LATEX TABLE FOR PAPER")
        print("="*80)
        print("\n% Comparison Table - Cartesian vs Joint Execution")
        print("\\begin{table}[htbp]")
        print("\\caption{Performance Comparison: Cartesian vs Joint Execution}")
        print("\\begin{center}")
        print("\\begin{tabular}{|l|c|c|}")
        print("\\hline")
        print("\\textbf{Metric} & \\textbf{Cartesian} & \\textbf{Joint} \\\\")
        print("\\hline")

        # Initialization timing
        cart_init = self.compute_statistics(cartesian, ['timing_ms', 'total_initialization_time'])
        joint_init = self.compute_statistics(joint, ['timing_ms', 'total_initialization_time'])

        if cart_init and joint_init:
            print(f"Initialization time (ms) & {cart_init['mean']:.1f} & {joint_init['mean']:.1f} \\\\")

        # IK time
        cart_ik = self.compute_statistics(cartesian, ['timing_ms', 'ik_solve_time'])
        if cart_ik:
            print(f"IK solving time (ms) & {cart_ik['mean']:.1f} & 0.0 \\\\")

        # Planning time
        cart_plan = self.compute_statistics(cartesian, ['timing_ms', 'trajectory_planning_time'])
        joint_plan = self.compute_statistics(joint, ['timing_ms', 'trajectory_planning_time'])

        if cart_plan and joint_plan:
            print(f"Trajectory planning (ms) & {cart_plan['mean']:.1f} & {joint_plan['mean']:.1f} \\\\")

        print("\\hline")

        # Joint metrics
        joint_vel = self.compute_statistics(joint, ['joint_metrics', 'max_joint_velocity_rad_s'])
        joint_acc = self.compute_statistics(joint, ['joint_metrics', 'max_joint_acceleration_rad_s2'])

        if joint_vel:
            print(f"Max joint velocity (rad/s) & N/A & {joint_vel['mean']:.2f} \\\\")
        if joint_acc:
            print(f"Max joint accel (rad/s$^2$) & N/A & {joint_acc['mean']:.2f} \\\\")

        print("\\hline")
        print("\\end{tabular}")
        print("\\label{tab:performance_comparison}")
        print("\\end{center}")
        print("\\end{table}")
        print()

    def save_summary_markdown(self, output_file='metrics_report.md'):
        """Save report as markdown file"""
        output_path = self.metrics_dir / output_file

        with open(output_path, 'w') as f:
            f.write("# Gesture Performance Metrics Report\n\n")
            f.write(f"Generated: {Path.cwd()}\n\n")

            cartesian, joint = self.group_by_mode()

            f.write(f"## Summary\n\n")
            f.write(f"- Total executions: {len(self.metrics)}\n")
            f.write(f"- Cartesian mode: {len(cartesian)}\n")
            f.write(f"- Joint mode: {len(joint)}\n\n")

            f.write("## Timing Comparison\n\n")
            f.write("| Metric | Cartesian (ms) | Joint (ms) |\n")
            f.write("|--------|----------------|------------|\n")

            cart_init = self.compute_statistics(cartesian, ['timing_ms', 'total_initialization_time'])
            joint_init = self.compute_statistics(joint, ['timing_ms', 'total_initialization_time'])

            if cart_init and joint_init:
                f.write(f"| Initialization | {cart_init['mean']:.2f} ± {cart_init['std']:.2f} | "
                       f"{joint_init['mean']:.2f} ± {joint_init['std']:.2f} |\n")

            # Add more metrics...

            f.write("\n## Per-Gesture Breakdown\n\n")
            by_gesture = self.group_by_gesture()

            for gesture, modes in sorted(by_gesture.items()):
                f.write(f"### {gesture}\n\n")
                f.write(f"- Cartesian: {len(modes['cartesian'])} executions\n")
                f.write(f"- Joint: {len(modes['joint'])} executions\n\n")

        print(f"\n✅ Markdown report saved to: {output_path}")


def main():
    # Default metrics directory
    metrics_dir = Path.home() / 'smilei_ws/src/smilei_state_machine/performance_metrics'

    # Allow custom directory as argument
    if len(sys.argv) > 1:
        metrics_dir = Path(sys.argv[1])

    print(f"📊 Analyzing metrics from: {metrics_dir}\n")

    analyzer = PerformanceAnalyzer(metrics_dir)

    if not analyzer.metrics:
        print("❌ No metrics found. Execute some gestures first!")
        return

    # Print comparison report
    analyzer.print_comparison_report()

    # Generate LaTeX table
    analyzer.generate_latex_table()

    # Save markdown report
    analyzer.save_summary_markdown()


if __name__ == '__main__':
    main()
