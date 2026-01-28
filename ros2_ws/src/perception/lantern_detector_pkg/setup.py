from setuptools import find_packages, setup

package_name = "lantern_detector_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/lantern_detector.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="autonomous-systems",
    maintainer_email="autonomous@example.com",
    description="Lantern detector node using semantic and depth images.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lantern_detector = lantern_detector_pkg.lantern_detector_node:main",
            "lantern_detection_logger = lantern_detector_pkg.lantern_detector_node:logger_main",
        ],
    },
)
