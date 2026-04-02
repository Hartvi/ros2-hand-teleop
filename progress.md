# Progress


### TODOs
- optimize the initial IK such that:
  1. first perform the IK
  2. then optimize --- something like scipy.optimize --- the position such that the gripper fingers appropach the hand positions indices 4 & 8
    - To achieve this we will need to do fast FK and check that the fingers are indeed at the requested positions (might want to use KDL & return the FK function as python-bound so that scipy.optimize.minimize can use it)
- save data
- create data set scene
- automatically detect finger & hand points to align with urdf gripper/ee pose


### DONE
- add gazebo/newer gazebo(fusion?) simulation
- create parameter for base to finger distance for each config
- connect gripper urdf to robot urdf
- smooth out movements
- fix hand transform to correspond to the correct rotation
- set camera transform
- get position and dir from hand points
- visualize robotics arm URDFs
- inverse kinematics on robot arm from pose or just position

