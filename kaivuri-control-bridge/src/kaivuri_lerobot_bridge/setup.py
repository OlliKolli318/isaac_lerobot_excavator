from setuptools import find_packages, setup


package_name = "kaivuri_lerobot_bridge"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Excavator maintainers",
    maintainer_email="olli.kangasniemi@savonia.fi",
    description="Safety-oriented ROS 2 control bridge for the Isaac LeRobot excavator.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "control_bridge = kaivuri_lerobot_bridge.control_bridge:main",
        ],
    },
)
