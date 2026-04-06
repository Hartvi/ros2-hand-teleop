#!/usr/bin/env python3
import subprocess
import sys

from pathlib import Path


cwd = Path(__file__).parent.parent.resolve()


def bash(cmd: str):
    print("RUNNING:", cmd)
    return subprocess.run(
        ["bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && " + cmd], cwd=cwd
    )


def stop_stale_runtime_processes():
    # Prevent duplicate /clock publishers and duplicated node graphs between runs.
    patterns = [
        "ros2 launch hand_publisher hand_publisher_launch.py",
        "gz sim",
        "ros_gz_bridge",
        "parameter_bridge",
        "ros_gz_sim create",
        "rviz2",
    ]
    for pattern in patterns:
        subprocess.run(["pkill", "-f", pattern], cwd=cwd)


args = sys.argv[1:]

clean = "clean" in args
if "clean" in args:
    args.remove("clean")
build = "build" in args
if "build" in args:
    args.remove("build")
run = "run" in args
if "run" in args:
    args.remove("run")
kill = "kill" in args
if "kill" in args:
    args.remove("kill")

if clean:
    subprocess.run(["rm", "-r", "build"], cwd=cwd)
    subprocess.run(["rm", "-r", "install"], cwd=cwd)
    subprocess.run(["rm", "-r", "log"], cwd=cwd)

if build:
    output = bash("colcon build --symlink-install")
    print(output)

if kill:
    stop_stale_runtime_processes()
    subprocess.run(["ros2", "daemon", "stop"], cwd=cwd)
    subprocess.run(
        [
            "bash",
            "-lc",
            'pgrep -af "ros2|gz sim|rviz2|parameter_bridge|trac_ik_node|hand_points_node|hand_publisher_node|hand_frame_node|controller_node|joint_state_merger|gripper_publisher|spawner|controller_manager" || true',
        ],
        cwd=cwd,
    )

if run:
    stop_stale_runtime_processes()
    bash(
        """
source ./install/setup.bash &&
unset GTK_PATH &&
ros2 launch hand_publisher hand_publisher_launch.py """
        + (" ".join(args) if args else "")
    )
