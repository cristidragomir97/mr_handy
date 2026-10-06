from glob import glob
from setuptools import setup

setup(
    name="handy101_mujoco",
    version="0.1.0",
    packages=["handy101_mujoco"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/handy101_mujoco"]),
        ("share/handy101_mujoco", ["package.xml"]),
        ("share/handy101_mujoco/launch", glob("launch/*.launch.py")),
        ("share/handy101_mujoco/worlds", glob("worlds/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cdr",
    maintainer_email="cdr@example.com",
    description="URDF-derived handy101 MuJoCo assembly",
    license="UNLICENSED",
)
