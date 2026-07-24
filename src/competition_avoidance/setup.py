from glob import glob
from setuptools import setup


package_name = "competition_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agilex",
    maintainer_email="agilex@todo.todo",
    description="Additive static and dynamic avoidance for the competition car.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "avoidance_manager_node = competition_avoidance.avoidance_manager_node:main",
            "odometry_adapter_node = competition_avoidance.odometry_adapter_node:main",
        ],
    },
)
