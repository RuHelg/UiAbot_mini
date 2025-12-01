from setuptools import find_packages, setup
import glob
import os

package_name = 'uiabot_mini_description'

urdf_files = glob.glob(os.path.join('urdf', '*'))
mesh_files = glob.glob(os.path.join('meshes', '*'))

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', urdf_files),
        ('share/' + package_name + '/meshes', mesh_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group6',
    maintainer_email='group6@group6.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
