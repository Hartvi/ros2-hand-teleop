from setuptools import find_packages, setup
import os
from glob import glob

package_name = "hand_publisher"


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
]

# Install all files under urdf/ (including meshes, model.config, etc.)
for root, _, files in os.walk("urdf"):
    if not files:
        continue
    install_dir = os.path.join("share", package_name, root)
    file_paths = [os.path.join(root, f) for f in files]
    data_files.append((install_dir, file_paths))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
        "mediapipe<0.10.30",
        "scipy==1.17.0",
    ],
    zip_safe=True,
    maintainer="hartvi",
    maintainer_email="j.hartvich@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hand_publisher_node=hand_publisher_node.hand_publisher_node:main",
            "hand_image_node=hand_publisher_node.hand_image_node:main",
            "hand_points_node=hand_publisher_node.hand_points_node:main",
            "hand_frame_node=hand_publisher_node.hand_frame_node:main",
            "controller_node=hand_publisher_node.controller_node:main",
            "joint_state_merger=hand_publisher_node.joint_state_merger:main",
            "gripper_publisher=hand_publisher_node.gripper_publisher:main",
        ],
    },
)
