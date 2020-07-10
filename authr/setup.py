from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
	packages=['authr_tools'],
    scripts=[''],
	package_dir={'':'src_ros'}
	)

setup(**setup_args)
