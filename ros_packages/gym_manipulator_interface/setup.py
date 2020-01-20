#!/usr/bin/env python

from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(name='gym_manipulator_interface',
		packages=['ManipulatorAction',
				'action_queue_server'],
		package_dir={'':'src'}
)

setup(**setup_args)
