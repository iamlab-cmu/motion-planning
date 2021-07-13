
"""Setup script for motion-planning"""

from setuptools import setup

requirements = [
    'urdfpy',
    'pybullet',
]

setup(name='motion_planning',
        version='0.1.0',
        author='IAM Lab',
        author_email='svats@andrew.cmu.edu',
        package_dir = {'': '.'},
        packages=['motion_planning'],
        install_requires=requirements
        )
