from setuptools import find_packages, setup

package_name = "spider_gait"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SPIDAR300",
    maintainer_email="bramavtoroj@gmail.com",
    description="Leg kinematics, tripod gait and the /joint_command adapter for the SPIDAR300 hexapod.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gait_node = spider_gait.gait_node:main",
            "joint_command_adapter = spider_gait.joint_command_adapter:main",
            "keyboard_teleop = spider_gait.keyboard_teleop:main",
            "pi_bridge = spider_gait.pi_bridge:main",
        ],
    },
)
