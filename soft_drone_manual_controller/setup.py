from glob import glob
import os
from setuptools import setup

package_name = "soft_drone_manual_controller"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "EXTRACTION_NOTES.md", "LICENSE"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Teddy",
    maintainer_email="noreply@example.com",
    description="Manual DJI RC quadrotor controller with six-IMU structural observer/DIMD support.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "manual_controller = soft_drone_manual_controller.manual_controller_6imu:main",
        ],
    },
)
