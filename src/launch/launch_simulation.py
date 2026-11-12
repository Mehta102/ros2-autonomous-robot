#!/usr/bin/env python3
# Launch script for starting the Gazebo simulation
# Starts Gazebo and the ROS bridge

import os
import subprocess
import time
import signal

def main():
    # find the project directories
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    world_file = os.path.join(project_dir, "worlds", "robot_world.sdf")
    model_dir = os.path.join(project_dir, "models")

    # set environment variables so Gazebo can find the models
    env = os.environ.copy()
    resource_path = env.get("IGN_GAZEBO_RESOURCE_PATH", "")
    env["IGN_GAZEBO_RESOURCE_PATH"] = f"{model_dir}:{resource_path}"
    gz_resource = env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = f"{model_dir}:{gz_resource}"

    print("=" * 60)
    print("Starting simulation")
    print(f"World file: {world_file}")
    print(f"Model directory: {model_dir}")
    print("=" * 60)

    processes = []

    try:
        # start Gazebo
        print("\nLaunching Gazebo...")
        gz_proc = subprocess.Popen(["ign", "gazebo", "-r", world_file], env=env)
        processes.append(gz_proc)
        time.sleep(5)  # wait for gazebo to start

        # start ROS bridge
        print("Starting ROS bridge...")
        bridge_args = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/camera_pan_cmd@std_msgs/msg/Float64]ignition.msgs.Double",
            "/world/robot_world/model/my_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
        ]
        bridge_proc = subprocess.Popen(bridge_args, env=env)
        processes.append(bridge_proc)

        print("\nSimulation running!")
        print("Topics: /cmd_vel, /odom, /camera, /lidar, /camera_pan_cmd")
        print("Press Ctrl+C to stop\n")

        gz_proc.wait()

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for proc in processes:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        print("Done")

if __name__ == "__main__":
    main()
