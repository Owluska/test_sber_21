from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['src.scripts',  'src.scripts.data'],
    package_dir = {'':'src'},
    requires = ['roslib', 'rospy', 'std_msgs']
)

setup(**setup_args)