#!/usr/bin/env python
from setuptools import setup, find_packages

setup(
      name='manipulator_training',
      version='0.1',
      description='Training RL agents on a Manipulator for different physical tasks.',
      author='Karl Ylvisaker',
      author_email='km.ylvisaker@gmail.com',
      packages = find_packages(),
      install_requires=['gym', 'numpy<=1.11.1', 'rospy', 'gym_manipulator_interface']#And any other dependencies required
)
