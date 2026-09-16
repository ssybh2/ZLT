from setuptools import find_packages, setup

package_name = "sn2031674_manual_rc"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/manual_rc.yaml"]),
        ("share/" + package_name + "/launch", ["launch/manual_rc.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ssybh2",
    maintainer_email="maintainer@example.com",
    description="Manual DJI RC to DShot controller for EtherCAT slave sn2031674",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "manual_rc_dshot = sn2031674_manual_rc.manual_rc_dshot:main",
        ],
    },
)
