#!/usr/bin/python2.7
from setuptools import setup

setup(
    name='roshammer',
    version='0.0.1',
    description='A fault injection tool for Robot Operating System (ROS)',
    long_description='TBA',
    # need to modify to have multiple authors!
    author='Chris Timperley, Jam M. Hernandez Q.',
    author_email='christimperley@googlemail.com, jamarck96@gmail.com',
    url='https://github.com/ChrisTimperley/roshammer',
    license='mit',
    dependency_links=['https://hg.python.org/cpython/raw-file/2.7/Lib/xmlrpclib.py#egg=xmlrpclib-2.7'],
    install_required=['xmlrpclib'],
    packages=['roshammer'],
    entry_points = {
        'console_scripts': [ 'roshammer = roshammer.roshammer:main' ]
    }
)
