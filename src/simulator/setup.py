from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['simulator'],
	package_dir={'': 'simulator'}
)

setup(**setup_args)