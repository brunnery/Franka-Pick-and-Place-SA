from glob import glob

from setuptools import setup

package_name = "sa_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="brunnery",
    maintainer_email="brunnery99@gmail.com",
    description="Nut and stud finder (vision_node), camera controls.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_node = sa_vision.vision_node:main",
            "apply_camera_controls = sa_vision.camera_controls:main",
        ],
    },
)
