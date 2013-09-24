from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['wpi_planning_utilities'],
    package_dir={'': 'src'}
)

setup(**d)
