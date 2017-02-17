#!/usr/bin/python3
from setuptools import setup

setup(
    name='roshammer',
    version='0.0.1',
    description='A fault injection tool for Robot Operating System (ROS)',
    long_description='TBA',
    # need to modify to have multiple authors!
    author='Chris Timperley',
    author_email='christimperley@googlemail.com',
    url='https://github.com/ChrisTimperley/roshammer',
    license='mit',
    packages=['roshammer'],
    entry_points = {
        'console_scripts': [ 'roshammer = roshammer.roshammer:main' ]
    }
)
