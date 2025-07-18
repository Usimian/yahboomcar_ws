#!/usr/bin/env python3
"""
Shutdown script for autonomous driving system
Forcefully terminates any remaining processes
"""

import os
import signal
import subprocess
import time
import sys

def get_process_pids(process_names):
    """Get PIDs of processes matching the given names"""
    pids = []
    for name in process_names:
        try:
            # Use pgrep to find process IDs
            result = subprocess.run(['pgrep', '-f', name], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                for pid in result.stdout.strip().split('\n'):
                    if pid:
                        pids.append(int(pid))
        except Exception as e:
            print(f"Error finding processes for {name}: {e}")
    return pids

def kill_processes(pids, signal_type=signal.SIGTERM):
    """Kill processes by PID"""
    killed = []
    for pid in pids:
        try:
            os.kill(pid, signal_type)
            killed.append(pid)
            print(f"Sent signal {signal_type} to process {pid}")
        except ProcessLookupError:
            print(f"Process {pid} already terminated")
        except PermissionError:
            print(f"Permission denied to kill process {pid}")
        except Exception as e:
            print(f"Error killing process {pid}: {e}")
    return killed

def main():
    """Main shutdown function"""
    print("Autonomous Drive Shutdown Script")
    print("=" * 40)
    
    # List of process names to terminate
    process_names = [
                    'auto_navigator',
            'auto_control',
        'debug_monitor',
        'Mcnamu_driver_X3',
        'base_node_X3',
        'sllidar_node',
        'imu_filter_madgwick_node',
        'ekf_node',
        'yahboom_joy_X3',
        'joy_node',
        'robot_state_publisher',
        'joint_state_publisher',
        'static_transform_publisher',
        'rviz2'
    ]
    
    print("Searching for running processes...")
    pids = get_process_pids(process_names)
    
    if not pids:
        print("No autonomous drive processes found running")
        return
    
    print(f"Found {len(pids)} processes to terminate: {pids}")
    
    # First try graceful shutdown with SIGTERM
    print("\nStep 1: Sending SIGTERM (graceful shutdown)...")
    killed_graceful = kill_processes(pids, signal.SIGTERM)
    
    if killed_graceful:
        print("Waiting 3 seconds for graceful shutdown...")
        time.sleep(3)
        
        # Check which processes are still running
        remaining_pids = get_process_pids(process_names)
        if remaining_pids:
            print(f"\nStep 2: {len(remaining_pids)} processes still running, sending SIGKILL...")
            kill_processes(remaining_pids, signal.SIGKILL)
            
            # Final check
            time.sleep(1)
            final_pids = get_process_pids(process_names)
            if final_pids:
                print(f"Warning: {len(final_pids)} processes still running: {final_pids}")
            else:
                print("All processes successfully terminated")
        else:
            print("All processes terminated gracefully")
    
    # Also try to kill any ROS-related processes
    print("\nStep 3: Cleaning up ROS processes...")
    ros_commands = [
        ['pkill', '-f', 'ros2'],
        ['pkill', '-f', 'rclpy'],
        ['pkill', '-f', 'launch']
    ]
    
    for cmd in ros_commands:
        try:
            subprocess.run(cmd, capture_output=True)
        except Exception as e:
            print(f"Error running {' '.join(cmd)}: {e}")
    
    print("\nShutdown complete!")
    print("You can now safely restart the autonomous driving system")

if __name__ == '__main__':
    main() 