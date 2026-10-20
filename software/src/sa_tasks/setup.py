from glob import glob

from setuptools import setup

package_name = "sa_tasks"

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
    description="Pick (step 1) and place/screw (step 2) sequences, pose teaching.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pick = sa_tasks.task_node:main_pick",
            "place_screw = sa_tasks.task_node:main_place",
            "full = sa_tasks.task_node:main_full",
            "teach_poses = sa_tasks.teach_poses:main",
        ],
    },
)
