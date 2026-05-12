# ROS2 hand teleoperation & data collection pipeline

## New video
https://www.youtube.com/watch?v=Nm4BNguZxqY

This project is a ROS2-based hand teleoperation and robotic data collection framework that allows a user to control a robot arm using only hand movements captured from a webcam. The long-term goal is to use the system to generate training data for Vision-Language-Action (VLA) models and then evaluate those models inside the same robotics environment.

The pipeline works by capturing webcam images, detecting hand landmarks with MediaPipe, estimating the 3D position and orientation of the hand, and converting that into robot end-effector targets. These targets are passed through inverse kinematics using TRAC-IK to generate robot joint commands, which are then executed either on a simulated robot in Gazebo or potentially on real hardware.

The system already supports:
- Real-time hand teleoperation at roughly 10 fps
- Transfer into Gazebo simulation
- Robot state recording for dataset generation
- Multiple robot configurations (e.g. Kinova and Panda)
- Visualization through RViz2
- Sound-level enabled teleoperation interactions

The main unfinished parts are focused on machine learning integration:
- converting collected demonstrations into formats like LeRobot datasets,
- finetuning VLA models on the generated demonstrations,
- and testing learned policies directly in the same teleoperation/simulation framework.

Future planned work includes:
- Isaac Sim integration,
- MoveIt integration - **WiP**
- improved URDF/Xacro robot models - **WiP**
- action-server-based teleoperation - **WiP**
- and potentially rewriting performance-critical parts in C++.

## How to launch
- `python utils.py build` - build the project
- `python utils.py run` - run the project
- `python utils.py clean` - remove the log & install & build directories (sometimes needed when build fails)
- `python utils.py clean build run` - all of the above
- `python utils.py run robot:=panda` - run with panda robot instead of kinova (no gripper atm)

### Specs
- Distance estimate done using symbolic regression:
  - `dists = list(segment dist on each finger)`
  - `mean_dist = mean(dists)`
  - distance: `D*(B/(A * np.mean(finger digit segment length (each finger, each segment)))**C)`
    - A,B,C,D estimated by trial and error
- Effective range: 30-90 cm +-1 cm (for my hand)

### Launching rviz2 independently
- `unset GTK_PATH`
- `ros2 run rviz2 rviz2`

Requirements:
- Python 3.12.3
- ROS2 Jazzy
- `sudo apt install ros-${ROS_DISTRO}-trac-ik`
- `sudo apt install ros-jazzy-moveit-resources-panda-description`
- requirements.txt

run with gazebo UI:
- `mkdir ros_teleop && mv ros2-hand-teleop src && mv src ros_teleop/ && cd ros_teleop/src`
- `./utils.py clean build run use_gz:=true gz_gui:=true world:=src/my_world.sdf robot:=panda`
