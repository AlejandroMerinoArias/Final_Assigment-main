from setuptools import setup

package_name = "perception_fusion_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/perception_fusion_pkg"]),
        ("share/perception_fusion_pkg", ["package.xml"]),
        ("share/perception_fusion_pkg/launch", ["launch/lantern_fusion.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="Fuses lantern detections into a world-frame POI list for planning.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lantern_fusion_node = perception_fusion_pkg.lantern_fusion_node:main",
        ],
    },
)