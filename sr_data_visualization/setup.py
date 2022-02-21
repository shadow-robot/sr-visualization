from __future__ import absolute_import
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['sr_data_visualization'],
    package_dir={'': 'src'},
    scripts=['scripts/sr_data_visualizer_plugin']
)

setup(**setup_args)
