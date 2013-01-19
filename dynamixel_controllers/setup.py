#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
           scripts=[
               'nodes/controller_manager.py',
               'nodes/controller_spawner.py',
               ],
           packages=['dynamixel_controllers'],
           package_dir={'': 'src'}
          )

setup(**d)
