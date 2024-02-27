from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['resource_manager'],
    package_dir={'': 'src'},
    scripts=['src/resource_manager/resource_manager_sga.py']
)

setup(**setup_args)
