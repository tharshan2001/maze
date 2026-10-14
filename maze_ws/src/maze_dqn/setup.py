from setuptools import find_packages, setup

package_name = "maze_dqn"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/dqn.yaml"]),
        ("share/" + package_name + "/models", []),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tharshan2001",
    maintainer_email="tharshan2001@users.noreply.github.com",
    description="Deep Q-Learning navigation for TurtleBot3 in the Gazebo maze.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dqn_nav_node = maze_dqn.dqn_nav_node:main",
            "train = maze_dqn.train:main",
        ],
    },
)
