from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'uiabot_mini_gazebo'

# Helper to get all files in models directory preserving structure
def get_data_files(directory, install_dir):
    data_files = []
    for root, dirs, files in os.walk(directory):
        if files:
            rel_path = os.path.relpath(root, directory)
            install_path = os.path.join(install_dir, rel_path)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_path, file_list))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name), glob('*.sdf')),
    ] + get_data_files('models', os.path.join('share', package_name, 'models')),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group-x',
    maintainer_email='Group-x@uia.no',
    description='TODO: Package description',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
