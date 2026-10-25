from glob import glob

from setuptools import setup

package_name = "debris_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UREX",
    maintainer_email="evanyeo2006@gmail.com",
    description="Detect the debris, centre the camera on it, publish its coordinates.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tracker_node = debris_tracker.tracker_node:main",
            "fake_camera_node = debris_tracker.fake_camera_node:main",
            "eval_node = debris_tracker.eval_node:main",
        ],
    },
)
