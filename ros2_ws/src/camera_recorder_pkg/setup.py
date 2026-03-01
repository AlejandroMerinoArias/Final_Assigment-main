from setuptools import setup

package_name = "camera_recorder_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/camera_recorder_pkg"]),
        ("share/camera_recorder_pkg", ["package.xml"]),
        ("share/camera_recorder_pkg/launch", ["launch/camera_recorder.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="Records RGB, depth, and semantic camera streams to video files.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_recorder = camera_recorder_pkg.camera_recorder_node:main",
        ],
    },
)
