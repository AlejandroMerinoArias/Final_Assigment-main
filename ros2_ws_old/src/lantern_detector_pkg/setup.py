from setuptools import setup

package_name = "lantern_detector_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/lantern_detector_pkg"]),
        ("share/lantern_detector_pkg", ["package.xml"]),
        ("share/lantern_detector_pkg/launch", ["launch/lantern_detector.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="Detect lanterns in semantic RGB imagery and publish 3D positions.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lantern_detector = lantern_detector_pkg.lantern_detector_node:main",
        ],
    },
)