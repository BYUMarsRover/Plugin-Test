#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

SETUP_ARGS = generate_distutils_setup(
    packages=['rover_plugins'],
    package_dir={'': 'src'}
)

setup(**SETUP_ARGS)
