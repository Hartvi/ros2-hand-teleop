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

if clean:
    subprocess.run(["rm", "-r", "build"], cwd=cwd)
    subprocess.run(["rm", "-r", "install"], cwd=cwd)
    subprocess.run(["rm", "-r", "log"], cwd=cwd)

if build:
    output = bash("colcon build --symlink-install")
    print(output)

if run:
    bash(
        """
source ./install/setup.bash &&
unset GTK_PATH &&
ros2 launch hand_publisher hand_publisher_launch.py """
        + (" ".join(args) if args else "")
    )
