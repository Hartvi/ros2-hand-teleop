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
    <joint name="panda_finger_joint2">
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


def inject_gz_camera(
    urdf_xml: str,
    parent_link: str = "panda_hand",
    sensor_name: str = "ee_camera",
    image_topic: str = "/panda/ee_camera/image_raw",
    camera_info_topic: str = "/panda/ee_camera/camera_info",
) -> str:
    camera_block = f"""
<gazebo reference=\"{parent_link}\">
    <sensor name=\"{sensor_name}\" type=\"camera\">
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>30</update_rate>
        <pose>-0.06 0.0 0.03 0 -1.571 0</pose>
        <camera>
            <camera_info_topic>{camera_info_topic}</camera_info_topic>
            <horizontal_fov>1.089</horizontal_fov>
            <image>
                <format>R8G8B8</format>
                <width>640</width>
                <height>480</height>
            </image>
            <clip>
                <near>0.05</near>
                <far>8.0</far>
            </clip>
        </camera>
        <topic>{image_topic}</topic>
    </sensor>
</gazebo>
"""

    close_tag = "</robot>"
    idx = urdf_xml.rfind(close_tag)
    if idx == -1:
        raise RuntimeError("URDF does not contain </robot> close tag")
    return urdf_xml[:idx] + camera_block + urdf_xml[idx:]
