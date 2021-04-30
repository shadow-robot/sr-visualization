from __future__ import absolute_import
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sr_gui_muscle_driver_bootloader'],
    package_dir={'': 'src'})

setup(**setup_args)
