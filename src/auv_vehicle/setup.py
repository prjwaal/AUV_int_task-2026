from setuptools import find_packages, setup

package_name = "auv_vehicle"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/telemetry.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description=(
        "Vehicle-side nodes: state/telemetry, command interface, "
        "mission management."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telemetry_publisher = auv_vehicle.telemetry_publisher:main",
            "vehicle_node = auv_vehicle.vehicle_node:main",
        ],
    },
)
