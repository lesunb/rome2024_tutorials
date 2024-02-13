from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rome_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),
         glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'params/'),
         glob('params/*')),
        (os.path.join('share', package_name, 'maps/'),
         glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='v',
    maintainer_email='v@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
