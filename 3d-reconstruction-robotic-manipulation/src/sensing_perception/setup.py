from setuptools import setup
from glob import glob
import os

package_name = 'sensing_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yufeng',
    maintainer_email='k22020061@kcl.ac.uk',
    description='SAP coursework package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'main = sensing_perception.main:main',
        'calibrate = sensing_perception.calibration.calibrate:main',
        'sfm = sensing_perception.sfm_node:main',
        ],
    },

)
