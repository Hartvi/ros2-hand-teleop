def inject_gz_ros2_control(
    urdf_xml: str, robot_name: str, controllers_yaml_abs: str
) -> str:
    match robot_name:
        case "panda":
            ros2_control_block = f"""
<ros2_control name="gz_ros2_control" type="system">
    <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <joint name="panda_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    </joint>
    <joint name="panda_joint2">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    </joint>
    <joint name="panda_joint3">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    </joint>
    <joint name="panda_joint4">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    </joint>
    <joint name="panda_joint5">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    </joint>
    <joint name="panda_joint6">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    </joint>
    <joint name="panda_joint7">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    </joint>
    <joint name="panda_finger_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    </joint>
</ros2_control>

<gazebo>
  <plugin
    name="gz_ros2_control::GazeboSimROS2ControlPlugin"
    filename="libgz_ros2_control-system.so">
    <parameters>{controllers_yaml_abs}</parameters>
  </plugin>
</gazebo>
"""
        case _:
            raise ValueError(f"Robot {robot_name} unknown model.")

    close_tag = "</robot>"
    idx = urdf_xml.rfind(close_tag)
    if idx == -1:
        raise RuntimeError("URDF does not contain </robot> close tag")
    return urdf_xml[:idx] + ros2_control_block + urdf_xml[idx:]
