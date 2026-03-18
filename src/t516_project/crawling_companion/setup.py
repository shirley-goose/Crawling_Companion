from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'crawling_companion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shirley&Suzy',
    maintainer_email='xhe09@uw.edu',
    description='Interactive robot for palying with crawling baby',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crawling_companion = baby.baby:main'
        ],
    },
)
