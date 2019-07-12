## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['polygon_coverage_benchmark'],
    package_dir={'':'python'},
    scripts=['scripts/plot_results']
    )

setup(**setup_args)
