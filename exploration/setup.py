import os

from setuptools import find_packages, setup

package_name = "exploration"

config_files = [os.path.join("config", filename) for filename in os.listdir("config")]
setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), config_files),
    ],
    install_requires=["setuptools", "numpy", "matplotlib", "scikit-fuzzy"],
    zip_safe=True,
    maintainer="pontus",
    maintainer_email="psn19003@student.mdu.se",
    description="Package used for turtlebot3 exploration using A* star for path navigation and fuzzy for local obstacle avoidance",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "exploration_node = exploration.explore_node:main",
            "computation_node = exploration.computation_node:main",
            "draw_node = exploration.draw_node:main",
            "astar_node = exploration.astar_node:main",
        ],
    },
)
