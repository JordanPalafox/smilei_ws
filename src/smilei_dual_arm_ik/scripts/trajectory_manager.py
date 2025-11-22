#!/usr/bin/env python3
"""
Interactive Trajectory Manager

Manages trajectory cache files in the trajectories directory.
Allows viewing, renaming, and deleting trajectory files.

Keyboard Controls:
- L: List all trajectory files
- R: Rename a trajectory file
- D: Delete a trajectory file
- C: Clear all trajectory cache files
- ESC: Exit
"""

import os
import sys
import termios
import tty
from pathlib import Path


class TrajectoryManager:
    def __init__(self):
        # Path to trajectories directory
        self.trajectories_dir = os.path.expanduser(
            '~/smilei_ws/install/smilei_dual_arm_ik/share/smilei_dual_arm_ik/config/trajectories'
        )

        # Create directory if it doesn't exist
        os.makedirs(self.trajectories_dir, exist_ok=True)

        print("=" * 60)
        print("📁 TRAJECTORY CACHE MANAGER")
        print("=" * 60)
        print(f"Directory: {self.trajectories_dir}\n")

    def get_key(self):
        """Get a single keypress from the user"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            # Handle escape sequences (arrow keys, etc.)
            if ch == '\x1b':
                ch = sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def list_trajectories(self):
        """List all trajectory files in the directory"""
        print("\n" + "=" * 60)
        print("📋 TRAJECTORY FILES")
        print("=" * 60)

        # Get all .yaml files
        trajectory_files = sorted([f for f in os.listdir(self.trajectories_dir) if f.endswith('.yaml')])

        if not trajectory_files:
            print("❌ No trajectory files found")
            return []

        print(f"Found {len(trajectory_files)} trajectory file(s):\n")
        for i, filename in enumerate(trajectory_files, 1):
            filepath = os.path.join(self.trajectories_dir, filename)
            file_size = os.path.getsize(filepath)
            file_size_kb = file_size / 1024
            print(f"  {i}. {filename:<40} ({file_size_kb:.2f} KB)")

        print()
        return trajectory_files

    def rename_trajectory(self):
        """Rename a trajectory file"""
        print("\n" + "=" * 60)
        print("✏️  RENAME TRAJECTORY")
        print("=" * 60)

        # List current files
        trajectory_files = self.list_trajectories()
        if not trajectory_files:
            return

        # Get original filename
        print("Enter the current filename (without .yaml extension):")
        print("(or press Enter to cancel)")
        old_name = input(">>> ").strip()

        if not old_name:
            print("❌ Cancelled")
            return

        # Add .yaml extension if not provided
        if not old_name.endswith('.yaml'):
            old_name += '.yaml'

        old_path = os.path.join(self.trajectories_dir, old_name)

        if not os.path.exists(old_path):
            print(f"❌ File '{old_name}' not found!")
            return

        # Get new filename
        print("\nEnter the new filename (without .yaml extension):")
        new_name = input(">>> ").strip()

        if not new_name:
            print("❌ Cancelled")
            return

        # Add .yaml extension if not provided
        if not new_name.endswith('.yaml'):
            new_name += '.yaml'

        new_path = os.path.join(self.trajectories_dir, new_name)

        # Check if new name already exists
        if os.path.exists(new_path):
            print(f"⚠️  File '{new_name}' already exists!")
            print("Overwrite? (y/n):")
            confirm = input(">>> ").strip().lower()
            if confirm != 'y':
                print("❌ Cancelled")
                return

        # Rename the file
        try:
            os.rename(old_path, new_path)
            print(f"✅ Renamed: '{old_name}' → '{new_name}'")
        except Exception as e:
            print(f"❌ Error renaming file: {e}")

    def delete_trajectory(self):
        """Delete a trajectory file"""
        print("\n" + "=" * 60)
        print("🗑️  DELETE TRAJECTORY")
        print("=" * 60)

        # List current files
        trajectory_files = self.list_trajectories()
        if not trajectory_files:
            return

        # Get filename to delete
        print("Enter the filename to delete (without .yaml extension):")
        print("(or press Enter to cancel)")
        filename = input(">>> ").strip()

        if not filename:
            print("❌ Cancelled")
            return

        # Add .yaml extension if not provided
        if not filename.endswith('.yaml'):
            filename += '.yaml'

        filepath = os.path.join(self.trajectories_dir, filename)

        if not os.path.exists(filepath):
            print(f"❌ File '{filename}' not found!")
            return

        # Confirm deletion
        print(f"\n⚠️  Are you sure you want to delete '{filename}'?")
        print("This action cannot be undone! (y/n):")
        confirm = input(">>> ").strip().lower()

        if confirm != 'y':
            print("❌ Cancelled")
            return

        # Delete the file
        try:
            os.remove(filepath)
            print(f"✅ Deleted: '{filename}'")
        except Exception as e:
            print(f"❌ Error deleting file: {e}")

    def clear_all_trajectories(self):
        """Clear all trajectory cache files"""
        print("\n" + "=" * 60)
        print("🗑️  CLEAR ALL TRAJECTORIES")
        print("=" * 60)

        # List current files
        trajectory_files = [f for f in os.listdir(self.trajectories_dir) if f.endswith('.yaml')]

        if not trajectory_files:
            print("❌ No trajectory files to clear")
            return

        print(f"Found {len(trajectory_files)} trajectory file(s)")

        # Confirm deletion
        print(f"\n⚠️  Are you sure you want to delete ALL {len(trajectory_files)} trajectory files?")
        print("This action cannot be undone! (y/n):")
        confirm = input(">>> ").strip().lower()

        if confirm != 'y':
            print("❌ Cancelled")
            return

        # Delete all files
        deleted_count = 0
        for filename in trajectory_files:
            filepath = os.path.join(self.trajectories_dir, filename)
            try:
                os.remove(filepath)
                deleted_count += 1
            except Exception as e:
                print(f"❌ Error deleting '{filename}': {e}")

        print(f"✅ Deleted {deleted_count} trajectory file(s)")

    def show_menu(self):
        """Display the main menu"""
        print("\n" + "-" * 60)
        print("KEYBOARD CONTROLS:")
        print("-" * 60)
        print("  L - List all trajectory files")
        print("  R - Rename a trajectory file")
        print("  D - Delete a trajectory file")
        print("  C - Clear all trajectory cache files")
        print("  ESC - Exit")
        print("-" * 60)
        print("Press a key...")

    def run(self):
        """Main loop"""
        self.show_menu()

        while True:
            key = self.get_key()

            if key == '\x1b':  # ESC key
                print("\n👋 Exiting Trajectory Manager...")
                break

            elif key.lower() == 'l':
                self.list_trajectories()
                self.show_menu()

            elif key.lower() == 'r':
                self.rename_trajectory()
                self.show_menu()

            elif key.lower() == 'd':
                self.delete_trajectory()
                self.show_menu()

            elif key.lower() == 'c':
                self.clear_all_trajectories()
                self.show_menu()

            else:
                print(f"\n⚠️  Unknown key: '{key}' - Use L/R/D/C/ESC")
                self.show_menu()


def main():
    try:
        manager = TrajectoryManager()
        manager.run()
    except KeyboardInterrupt:
        print("\n\n👋 Interrupted - Exiting...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
