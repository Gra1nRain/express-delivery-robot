from setuptools import setup


package_name = "competition_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="agilex",
    maintainer_email="agilex@todo.todo",
    description="Competition mission state machine and ROS adapter.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_node = competition_mission.mission_node:main",
            "arm_task_simulator_node = competition_mission.arm_task_simulator_node:main",
            "piper_arm_task_node = competition_mission.piper_arm_task_node:main",
        ],
    },
)
