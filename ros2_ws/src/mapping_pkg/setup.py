from setuptools import setup

package_name = "mapping_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mapping_pkg"]),
        ("share/mapping_pkg", ["package.xml"]),
        ("share/mapping_pkg/launch", ["launch/mapping.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Student",
    maintainer_email="student@example.com",
    description="Depth-based mapping node that builds a simple voxel occupancy map.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mapping_node = mapping_pkg.mapping_node:main",
        ],
    },
)