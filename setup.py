from setuptools import setup
# from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


settings = generate_distutils_setup(
    packages=[
        "src"
    ],
)

setup(requires=["numpy",
                "empy",
                "pyyaml",
                "catkin_pkg",
                "rospkg"], **settings)
