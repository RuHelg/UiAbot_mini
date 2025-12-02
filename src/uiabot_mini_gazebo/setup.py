from setuptools import setup
import os
from glob import glob

package_name = 'uiabot_mini_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # Add optional folders if they exist
    if os.path.exists('launch'):
        data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py')))
    if os.path.exists('config'):
        data_files.append((os.path.join('share', package_name, 'config'), glob('config/*')))
    if os.path.exists('worlds'):
        data_files.append((os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')))
    if os.path.exists('urdf'):
        data_files.append((os.path.join('share', package_name, 'urdf'), glob('urdf/*'))),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group-x',
    maintainer_email='Group-x@uia.no',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
