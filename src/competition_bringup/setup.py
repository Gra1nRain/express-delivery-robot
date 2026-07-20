from glob import glob
from setuptools import setup

package_name = "competition_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    entry_points={
        "console_scripts": [
            "fastlio_anchor_node = competition_bringup.fastlio_anchor_node:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agilex",
    maintainer_email="agilex@todo.todo",
    description="Launch and field-validation entry points for the competition car.",
    license="Apache-2.0",
)
