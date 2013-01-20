#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
           scripts=[
               'scripts/change_id.py',
               'scripts/info_dump.py',
               'scripts/set_servo_config.py',
               'scripts/set_torque.py',
               ],
           packages=['dynamixel_driver'],
           package_dir={'': 'src'}
          )

setup(**d)
