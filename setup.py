from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_thump'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy'],
    scripts=['scripts/rqt_thump']
)

setup(**d)