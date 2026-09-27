from setuptools import find_packages, setup

package_name = "auv_ground_station"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/ground_station.launch.py",
                "launch/full_stack.launch.py",
                "launch/full_sim_stack.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description=(
        "Ground Station GUI: telemetry/status monitoring, comms-failure "
        "indication, and vehicle command sending (PySide6)."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ground_station = auv_ground_station.app:main",
        ],
    },
)
