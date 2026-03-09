gazebo control
- `panda_controllers.yml` - list all the joints and controllers that we want to control
- in launch file - list all the necessary controllers from `panda_controllers.yml` - this is probably a one to one correspondence with the controllers listed in all our controller yml files
- in our urdf we have to add a `ros2_control` block for each joint that we want to control
- gazebo doesn't handle mimic joints, so we have to work with that

