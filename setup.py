#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_setup = generate_distutils_setup(
    packages=['floor_detection'],
    package_dir={'floor_detection': 'common/floor_detection'}
)

setup(**package_setup)
