from setuptools import setup

package_name = "tesla_frames"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tfoldi",
    maintainer_email="tfoldi@xsi.hu",
    description="Frames, markers and TF2 definitions for Tesla Model 3",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tesla_markers_node = tesla_frames.tesla_markers_node:main",
            "tesla_tf2_publisher = tesla_frames.tesla_tf2_publisher:main",
        ],
    },
)
