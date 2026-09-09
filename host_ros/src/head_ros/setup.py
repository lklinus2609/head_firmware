from glob import glob
import os

from setuptools import setup

package_name = "head_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/head_ros"]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "head_bridge = head_ros.bridge:main",
        "headctl = head_ros.headctl:main",
        "head_proprioception_collector = head_ros.collector:main",
        "head_proprioception_gui = head_ros.collector_gui:main",
    ]},
)
