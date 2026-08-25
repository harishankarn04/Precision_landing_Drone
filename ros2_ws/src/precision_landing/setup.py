from setuptools import find_packages, setup

package_name = "precision_landing"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hari Shankar N",
    maintainer_email="info@cloudties.in",
    description="Beacon-based autonomous precision landing for UAVs — ROS 2 side.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mavlink_bridge = precision_landing.mavlink_bridge:main",
            "fake_target = precision_landing.fake_target:main",
        ],
    },
)
