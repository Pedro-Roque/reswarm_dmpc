from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Export source python files
setup_args = generate_distutils_setup(
    packages=['ibfc'],
    package_dir={'': 'src'},
)

setup(**setup_args)
