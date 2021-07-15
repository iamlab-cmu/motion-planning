
"""Setup script for motion-planning"""

from setuptools import setup

requirements = ["numpy", #needs to be installed before pybullet
                "pybullet",
                "pillar_state",
                "urdfpy", 
                "hydra_core",
                "autolab_core",
                "omegaconf"
]

setup(name='motion_planning',
        version='0.1.0',
        author='IAM Lab',
        author_email='svats@andrew.cmu.edu, alagrass@andrew.cmu.edu',
        package_dir = {'': '.'},
        packages=['motion_planning'],
        install_requires=requirements
        )
